#include "algorithms/arcflags/arc_flags.hpp"

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/heap_node.hpp"
#include "algorithms/partition.hpp"
#include "algorithms/phast.hpp"
#include "algorithms/region_flags.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"
#include "graph/types.hpp"
#include "routing/routing.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace transport {

namespace {
// Number of PHAST target lanes per all_to_one_batch call.
constexpr size_t kBatchSize = 8;
} // namespace

ArcFlagsAlgorithm::ArcFlagsAlgorithm(const Graph &graph, uint32_t regions, std::string partition_method,
                                     uint32_t threads)
    : graph_(graph), regions_(regions), partition_method_(parse_partition_method(partition_method)),
      threads_(threads == 0 ? 1 : threads), dist_(graph.vertex_count(), kUnreachable) {
    validate_region_count(regions, "arcflags");
    if (partition_method_ == PartitionMethod::Inertial) {
        validate_inertial_regions(regions, "arcflags");
    }
}

std::string_view ArcFlagsAlgorithm::name() const { return "arcflags"; }

void ArcFlagsAlgorithm::preprocess() {
    if (preprocessed_) {
        return;
    }

    ContractionHierarchyAlgorithm ch_algo(graph_);
    ch_algo.preprocess();
    const ContractionHierarchy &ch = ch_algo.get_ch();

    const PhastAlgorithm phast(ch);
    const VertexId V = graph_.vertex_count();
    const size_t E = graph_.edges.size();

    region_of_ = make_region_assignment(graph_, regions_, partition_method_name(partition_method_));

    // Identify boundary vertices: v is a boundary vertex of its region if any in-neighbor is from a different region.
    std::vector<bool> is_boundary(V, false);
    for (VertexId u = 0; u < V; ++u) {
        for (const Edge &e : graph_.adjacent_edges(u)) {
            if (region_of_[u] != region_of_[e.to]) {
                is_boundary[e.to] = true;
            }
        }
    }

    std::vector<std::vector<VertexId>> boundary_by_region(regions_);
    for (VertexId v = 0; v < V; ++v) {
        if (is_boundary[v]) {
            boundary_by_region[region_of_[v]].push_back(v);
        }
    }

    forward_flags_.assign(E, 0);
    compute_flags(phast, boundary_by_region);

    // Own-region rule: every arc u→v unconditionally gets bit region_of_[v] set.
    for (VertexId u = 0; u < V; ++u) {
        for (uint64_t k = graph_.offsets[u], end = graph_.offsets[u + 1]; k < end; ++k) {
            forward_flags_[static_cast<size_t>(k)] |= region_bit(region_of_[graph_.edges[static_cast<size_t>(k)].to]);
        }
    }

    preprocessed_ = true;
}

void ArcFlagsAlgorithm::compute_flags(const PhastAlgorithm &phast,
                                      const std::vector<std::vector<VertexId>> &boundary_by_region) {
    // Build flat work list: one entry per batch of kBatchSize boundary vertices in the same region.
    struct WorkItem {
        uint32_t region;
        size_t batch_start;
    };
    std::vector<WorkItem> work;
    for (uint32_t R = 0; R < regions_; ++R) {
        const auto &bvs = boundary_by_region[R];
        for (size_t bs = 0; bs < bvs.size(); bs += kBatchSize) {
            work.push_back({R, bs});
        }
    }
    if (work.empty()) {
        return;
    }

    const VertexId V = graph_.vertex_count();
    const uint64_t *offsets = graph_.offsets.data();
    const Edge *edges = graph_.edges.data();
    uint64_t *flags = forward_flags_.data();
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

            const uint32_t R = work[wi].region;
            const size_t bs = work[wi].batch_start;
            const auto &bvs = boundary_by_region[R];
            const size_t batch_size = std::min(kBatchSize, bvs.size() - bs);
            const uint64_t mask_bit = region_bit(R);

            batch.assign(bvs.begin() + static_cast<ptrdiff_t>(bs),
                         bvs.begin() + static_cast<ptrdiff_t>(bs + batch_size));

            phast.all_to_one_batch(batch, dist);
            // dist layout: dist[v * B + lane] = d(v, batch[lane])
            const size_t B = batch_size;

            for (VertexId u = 0; u < V; ++u) {
                const Distance *du = &dist[u * B];
                for (uint64_t k = offsets[u], end = offsets[u + 1]; k < end; ++k) {
                    const Edge &e = edges[static_cast<size_t>(k)];
                    const Distance *dv = &dist[e.to * B];
                    for (size_t lane = 0; lane < B; ++lane) {
                        if (du[lane] == kUnreachable || dv[lane] == kUnreachable) {
                            continue;
                        }
                        // Equality rule: arc u→e.to lies on a shortest path to batch[lane].
                        if (du[lane] == static_cast<Distance>(e.weight) + dv[lane]) {
                            std::atomic_ref<uint64_t>(flags[static_cast<size_t>(k)])
                                .fetch_or(mask_bit, std::memory_order_relaxed);
                            break; // bit R is set; remaining lanes in this batch share the same bit
                        }
                    }
                }
            }
        }
    };

    if (threads_ <= 1) {
        worker();
    } else {
        std::vector<std::thread> pool;
        pool.reserve(threads_);
        for (uint32_t t = 0; t < threads_; ++t) {
            pool.emplace_back(worker);
        }
        for (auto &th : pool) {
            th.join();
        }
    }
}

PathResult ArcFlagsAlgorithm::query(VertexId source, VertexId target) const {
    if (!preprocessed_) {
        throw std::runtime_error("arcflags: preprocess() must be called before query()");
    }

    const uint64_t target_bit = region_bit(region_of_[target]);

    dist_.reset();
    dist_.set(source, 0);

    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<HeapNode>> pq;
    pq.push({0, source});

    uint32_t settled = 0;
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != dist_.get(top.v)) {
            continue;
        }
        ++settled;
        if (top.v == target) {
            break;
        }
        for (uint64_t k = graph_.offsets[top.v], end = graph_.offsets[top.v + 1]; k < end; ++k) {
            if (!(forward_flags_[static_cast<size_t>(k)] & target_bit)) {
                continue;
            }
            const Edge &e = graph_.edges[static_cast<size_t>(k)];
            const Distance nd = top.key + static_cast<Distance>(e.weight);
            if (nd < dist_.get(e.to)) {
                dist_.set(e.to, nd);
                pq.push({nd, e.to});
            }
        }
    }

    return PathResult{dist_.get(target), settled};
}

} // namespace transport
