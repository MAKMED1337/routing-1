#include "algorithms/partition.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#ifdef TRANSPORT_HAVE_KAMINPAR
#if __has_include(<kaminpar.h>)
#include <kaminpar.h>
#else
#include <kaminpar-shm/kaminpar.h>
#endif
#endif

namespace transport {
namespace {

std::vector<uint16_t> partition_grid(const Graph &graph, uint16_t regions, std::span<const NodeCoord> coords) {
    const VertexId V = graph.vertex_count();
    require_matching_coords(coords, V, "grid partition");
    const auto side = static_cast<uint32_t>(std::ceil(std::sqrt(regions)));
    const uint32_t cols = side;
    const uint32_t rows = (static_cast<uint32_t>(regions) + cols - 1) / cols;

    double lat_min = std::numeric_limits<double>::max();
    double lat_max = std::numeric_limits<double>::lowest();
    double lon_min = std::numeric_limits<double>::max();
    double lon_max = std::numeric_limits<double>::lowest();
    for (const NodeCoord &c : coords) {
        lat_min = std::min(lat_min, c.lat);
        lat_max = std::max(lat_max, c.lat);
        lon_min = std::min(lon_min, c.lon);
        lon_max = std::max(lon_max, c.lon);
    }
    const double lat_range = lat_max - lat_min;
    const double lon_range = lon_max - lon_min;

    std::vector<uint16_t> result(V);
    for (VertexId v = 0; v < V; ++v) {
        const uint32_t r = lat_range > 0.0
                               ? std::min(static_cast<uint32_t>((coords[v].lat - lat_min) / lat_range * rows), rows - 1)
                               : 0u;
        const uint32_t c = lon_range > 0.0
                               ? std::min(static_cast<uint32_t>((coords[v].lon - lon_min) / lon_range * cols), cols - 1)
                               : 0u;
        const uint32_t region = r * cols + c;
        result[v] = static_cast<uint16_t>(std::min(region, static_cast<uint32_t>(regions) - 1));
    }
    return result;
}

void kd_split(std::span<const NodeCoord> coords, std::vector<VertexId> &indices, std::vector<uint16_t> &region_ids,
              uint32_t base, uint32_t count, uint32_t depth) {
    if (count == 0) {
        return;
    }
    if (count == 1) {
        for (const VertexId v : indices) {
            region_ids[v] = static_cast<uint16_t>(base);
        }
        return;
    }

    const bool split_lon = (depth % 2 == 0);
    std::sort(indices.begin(), indices.end(), [&](VertexId a, VertexId b) {
        return split_lon ? coords[a].lon < coords[b].lon : coords[a].lat < coords[b].lat;
    });

    const uint32_t half_left = count / 2;
    const uint32_t half_right = count - half_left;
    const size_t split = indices.size() / 2;

    std::vector<VertexId> left(indices.begin(), indices.begin() + static_cast<ptrdiff_t>(split));
    std::vector<VertexId> right(indices.begin() + static_cast<ptrdiff_t>(split), indices.end());

    kd_split(coords, left, region_ids, base, half_left, depth + 1);
    kd_split(coords, right, region_ids, base + half_left, half_right, depth + 1);
}

std::vector<uint16_t> partition_inertial(const Graph &graph, uint16_t regions, std::span<const NodeCoord> coords) {
    if (regions == 0 || (regions & (regions - 1)) != 0) {
        throw std::invalid_argument("inertial partition requires regions to be a power of 2");
    }
    const VertexId V = graph.vertex_count();
    require_matching_coords(coords, V, "inertial partition");
    std::vector<uint16_t> result(V, 0);
    std::vector<VertexId> all(V);
    for (VertexId v = 0; v < V; ++v) {
        all[v] = v;
    }
    kd_split(coords, all, result, 0, static_cast<uint32_t>(regions), 0);
    return result;
}

#ifdef TRANSPORT_HAVE_KAMINPAR
std::vector<uint16_t> partition_kaminpar(const Graph &graph, uint16_t regions) {
    using namespace kaminpar;
    using namespace kaminpar::shm;

    const VertexId V = graph.vertex_count();
    // KaMinPar requires a symmetric (undirected) adjacency; build it by merging forward and reverse edges.
    std::vector<std::vector<NodeID>> sym(V);
    for (VertexId u = 0; u < V; ++u) {
        for (const Edge &e : graph.adjacent_edges(u)) {
            sym[u].push_back(static_cast<NodeID>(e.to));
            sym[e.to].push_back(static_cast<NodeID>(u));
        }
    }
    // Deduplicate and build CSR.
    std::vector<EdgeID> xadj(V + 1);
    std::vector<NodeID> adjncy;
    for (VertexId u = 0; u < V; ++u) {
        std::sort(sym[u].begin(), sym[u].end());
        sym[u].erase(std::unique(sym[u].begin(), sym[u].end()), sym[u].end());
        xadj[u] = static_cast<EdgeID>(adjncy.size());
        adjncy.insert(adjncy.end(), sym[u].begin(), sym[u].end());
    }
    xadj[V] = static_cast<EdgeID>(adjncy.size());

    KaMinPar km(1, create_default_context());
    km.set_output_level(OutputLevel::QUIET);
    km.copy_graph(xadj, adjncy);

    std::vector<BlockID> part(V);
    km.compute_partition(static_cast<BlockID>(regions), part);

    std::vector<uint16_t> result(V);
    for (VertexId v = 0; v < V; ++v) {
        result[v] = static_cast<uint16_t>(part[v]);
    }
    return result;
}
#endif

} // namespace

PartitionMethod parse_partition_method(std::string_view method) {
    if (method == "grid") {
        return PartitionMethod::Grid;
    }
    if (method == "inertial") {
        return PartitionMethod::Inertial;
    }
    if (method == "kaminpar") {
        return PartitionMethod::KaMinPar;
    }
    throw std::invalid_argument("unknown partition method: " + std::string(method));
}

std::string_view partition_method_name(PartitionMethod method) {
    switch (method) {
    case PartitionMethod::Grid:
        return "grid";
    case PartitionMethod::Inertial:
        return "inertial";
    case PartitionMethod::KaMinPar:
        return "kaminpar";
    }
    throw std::invalid_argument("unknown partition method enum value");
}

std::vector<uint16_t> build_partition(const Graph &graph, uint16_t regions, PartitionMethod method,
                                      std::span<const NodeCoord> coords) {
    if (regions == 0) {
        throw std::invalid_argument("regions must be at least 1");
    }
    switch (method) {
    case PartitionMethod::Grid:
        return partition_grid(graph, regions, coords);
    case PartitionMethod::Inertial:
        return partition_inertial(graph, regions, coords);
    case PartitionMethod::KaMinPar:
#ifdef TRANSPORT_HAVE_KAMINPAR
        return partition_kaminpar(graph, regions);
#else
        throw std::runtime_error("KaMinPar support not compiled in (rebuild with -DTRANSPORT_HAVE_KAMINPAR=ON)");
#endif
    }
    throw std::invalid_argument("unknown partition method enum value");
}

} // namespace transport
