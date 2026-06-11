#include "algorithms/arcflags/arc_flags.hpp"

#include "algorithms/heap_node.hpp"
#include "algorithms/partition.hpp"
#include "algorithms/phast.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"
#include "graph/types.hpp"
#include "routing/routing.hpp"

#include <atomic>
#include <bit>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

namespace transport {

namespace {
// Backward Dijkstras batched per forward sweep. Larger values reduce sweep count but raise
// peak memory proportionally (V * kBatchSize * sizeof(Distance) bytes per work item).
// kBatchSize=1 keeps per-thread memory at V*4 bytes (~68 MB for Poland) instead of V*8*4 (~547 MB),
// avoiding swap pressure when running 16 threads on a 31 GiB machine.
constexpr size_t kBatchSize = 1;

void validate_arcflags_parameters(uint16_t regions, PartitionMethod partition_method, uint32_t threads) {
    if (regions == 0 || regions > 64) {
        throw std::invalid_argument("arcflags: regions must be in [1, 64]");
    }
    if (threads == 0) {
        throw std::invalid_argument("arcflags: threads must be >= 1");
    }
    if (partition_method == PartitionMethod::Inertial && !std::has_single_bit(regions)) {
        throw std::invalid_argument("arcflags: inertial partition requires regions to be a power of two");
    }
}

void validate_loaded_arcflags(const Graph &graph, const ArcFlagsPreprocessedData &data) {
    validate_arcflags_parameters(data.regions, data.partition_method, 1);
    if (data.region_of.size() != graph.vertex_count()) {
        throw std::invalid_argument("arcflags: loaded region count does not match graph vertex count");
    }
    if (data.forward_flags.size() != graph.edges.size()) {
        throw std::invalid_argument("arcflags: loaded flag count does not match graph edge count");
    }
    for (const uint16_t region : data.region_of) {
        if (region >= data.regions) {
            throw std::invalid_argument("arcflags: loaded region id out of range");
        }
    }
}

std::vector<std::vector<VertexId>>
find_boundary_vertices_by_region(const Graph &graph, const std::vector<uint16_t> &region_of, uint16_t regions) {
    const VertexId vertices = graph.vertex_count();
    std::vector<bool> is_boundary(vertices, false);
    for (VertexId u = 0; u < vertices; ++u) {
        for (const Edge &e : graph.adjacent_edges(u)) {
            if (region_of[u] != region_of[e.to]) {
                is_boundary[e.to] = true;
            }
        }
    }

    std::vector<std::vector<VertexId>> boundary_by_region(regions);
    for (VertexId v = 0; v < vertices; ++v) {
        if (is_boundary[v]) {
            boundary_by_region[region_of[v]].push_back(v);
        }
    }
    return boundary_by_region;
}

void compute_flags(const Graph &graph, const PhastAlgorithm &phast,
                   const std::vector<std::vector<VertexId>> &boundary_by_region, uint16_t regions, uint32_t threads,
                   std::vector<uint64_t> &forward_flags) {
    struct WorkItem {
        uint32_t region;
        size_t batch_start;
    };
    std::vector<WorkItem> work;
    for (uint32_t region = 0; region < regions; ++region) {
        const auto &bvs = boundary_by_region[region];
        for (size_t batch_start = 0; batch_start < bvs.size(); batch_start += kBatchSize) {
            work.push_back({region, batch_start});
        }
    }
    if (work.empty()) {
        return;
    }

    const VertexId vertices = graph.vertex_count();
    const size_t *offsets = graph.offsets.data();
    const Edge *edges = graph.edges.data();
    uint64_t *flags = forward_flags.data();
    std::atomic<size_t> work_idx{0};

    auto worker = [&]() {
        std::vector<Distance> dist;
        std::vector<VertexId> batch;
        batch.reserve(kBatchSize);

        while (true) {
            const size_t wi = work_idx.fetch_add(1, std::memory_order_relaxed);
            if (wi >= work.size()) {
                break;
            }

            const uint32_t region = work[wi].region;
            const size_t batch_start = work[wi].batch_start;
            const auto &bvs = boundary_by_region[region];
            const size_t batch_size = std::min(kBatchSize, bvs.size() - batch_start);
            const uint64_t mask_bit = 1ULL << region;

            batch.assign(bvs.begin() + static_cast<ptrdiff_t>(batch_start),
                         bvs.begin() + static_cast<ptrdiff_t>(batch_start + batch_size));

            phast.all_to_one_batch(batch, dist);
            const size_t lanes = batch_size;

            for (VertexId u = 0; u < vertices; ++u) {
                const Distance *du = &dist[u * lanes];
                for (size_t k = offsets[u], end = offsets[u + 1]; k < end; ++k) {
                    const Edge &e = edges[k];
                    const Distance *dv = &dist[e.to * lanes];
                    for (size_t lane = 0; lane < lanes; ++lane) {
                        if (du[lane] == kUnreachable || dv[lane] == kUnreachable) {
                            continue;
                        }
                        if (du[lane] == static_cast<Distance>(e.weight) + dv[lane]) {
                            std::atomic_ref<uint64_t>(flags[k]).fetch_or(mask_bit, std::memory_order_relaxed);
                            break;
                        }
                    }
                }
            }
        }
    };

    std::vector<std::thread> pool;
    pool.reserve(threads);
    for (uint32_t t = 0; t < threads; ++t) {
        pool.emplace_back(worker);
    }
    for (auto &th : pool) {
        th.join();
    }
}
} // namespace

ArcFlagsAlgorithm::ArcFlagsAlgorithm(const Graph &graph, PhastAlgorithm &&phast, uint16_t regions,
                                     PartitionMethod partition_method, uint32_t threads,
                                     std::span<const NodeCoord> coords)
    : graph_(graph), phast_(std::move(phast)), regions_(regions), partition_method_(partition_method),
      threads_(threads), coords_(coords), dist_(graph.vertex_count(), kUnreachable) {
    validate_arcflags_parameters(regions, partition_method, threads);
    if (phast_->vertex_count() != graph.vertex_count()) {
        throw std::invalid_argument("arcflags: PHAST vertex count does not match graph");
    }
}

ArcFlagsAlgorithm::ArcFlagsAlgorithm(const Graph &graph, ArcFlagsPreprocessedData &&data)
    : graph_(graph), regions_(data.regions), partition_method_(data.partition_method), threads_(1),
      dist_(graph.vertex_count(), kUnreachable) {
    load_preprocessed(std::move(data));
}

std::string_view ArcFlagsAlgorithm::name() const { return "arcflags"; }

void ArcFlagsAlgorithm::preprocess() {
    if (preprocessed_) {
        return;
    }

    if (!phast_) {
        throw std::logic_error("arcflags: PHAST dependency is required to preprocess()");
    }

    load_preprocessed(build_arc_flags(graph_, *phast_, regions_, partition_method_, threads_, coords_));
}

void ArcFlagsAlgorithm::load_preprocessed(ArcFlagsPreprocessedData &&data) {
    validate_loaded_arcflags(graph_, data);
    regions_ = data.regions;
    partition_method_ = data.partition_method;
    region_of_ = std::move(data.region_of);
    forward_flags_ = std::move(data.forward_flags);
    preprocessed_ = true;
}

ArcFlagsPreprocessedData ArcFlagsAlgorithm::export_preprocessed() const {
    if (!preprocessed_) {
        throw std::logic_error("arcflags: preprocess() must be called before export_preprocessed()");
    }
    return ArcFlagsPreprocessedData{
        .regions = regions_,
        .partition_method = partition_method_,
        .region_of = region_of_,
        .forward_flags = forward_flags_,
    };
}

ArcFlagsPreprocessedData build_arc_flags(const Graph &graph, PhastAlgorithm &phast, uint16_t regions,
                                         PartitionMethod partition_method, uint32_t threads,
                                         std::span<const NodeCoord> coords) {
    validate_arcflags_parameters(regions, partition_method, threads);
    if (phast.vertex_count() != graph.vertex_count()) {
        throw std::invalid_argument("arcflags: PHAST vertex count does not match graph");
    }

    ArcFlagsPreprocessedData data;
    data.regions = regions;
    data.partition_method = partition_method;
    data.region_of = build_partition(graph, regions, partition_method, coords);
    data.forward_flags.assign(graph.edges.size(), 0);

    const std::vector<std::vector<VertexId>> boundary_by_region =
        find_boundary_vertices_by_region(graph, data.region_of, regions);
    compute_flags(graph, phast, boundary_by_region, regions, threads, data.forward_flags);

    for (VertexId u = 0; u < graph.vertex_count(); ++u) {
        for (size_t k = graph.offsets[u], end = graph.offsets[u + 1]; k < end; ++k) {
            data.forward_flags[k] |= 1ULL << data.region_of[graph.edges[k].to];
        }
    }

    return data;
}

PathResult ArcFlagsAlgorithm::query(VertexId source, VertexId target) const {
    if (!preprocessed_) {
        throw std::runtime_error("arcflags: preprocess() must be called before query()");
    }

    const uint64_t target_bit = 1ULL << region_of_[target];

    dist_.reset();
    dist_.set(source, 0);

    HeapQueue pq;
    pq.push({0, source});

    QueryStats stats;
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != dist_.get(top.v)) {
            continue;
        }
        ++stats.settled;
        if (top.v == target) {
            break;
        }
        for (size_t k = graph_.offsets[top.v], end = graph_.offsets[top.v + 1]; k < end; ++k) {
            ++stats.relaxed_arcs;
            if (!(forward_flags_[k] & target_bit)) {
                ++stats.pruned_by_flag;
                continue;
            }
            const Edge &e = graph_.edges[k];
            const Distance nd = top.key + static_cast<Distance>(e.weight);
            if (nd < dist_.get(e.to)) {
                dist_.set(e.to, nd);
                pq.push({nd, e.to});
                ++stats.heap_pushes;
            }
        }
    }

    return PathResult{dist_.get(target), stats};
}

} // namespace transport
