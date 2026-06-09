#include "algorithms/partition.hpp"
#include "graph/graph.hpp"

#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

// 4-vertex graph with coordinates and a symmetric ring of edges (0↔1↔2↔3↔0).
// Coordinates spread across a small lat/lon box so grid/inertial partitions produce distinct buckets.
// The directed ring symmetrizes to an undirected 4-cycle, which is the minimum connected graph
// required by KaMinPar.
transport::Graph make_coord_graph() {
    transport::Graph graph;
    graph.coords = {
        {.lat = 10.0, .lon = 20.0},
        {.lat = 10.0, .lon = 21.0},
        {.lat = 11.0, .lon = 21.0},
        {.lat = 11.0, .lon = 20.0},
    };
    // Symmetric ring: 0→1, 1→2, 2→3, 3→0 and back-edges 1→0, 2→1, 3→2, 0→3.
    graph.offsets = {0, 2, 4, 6, 8};
    graph.edges = {
        {.to = 1, .weight = 1}, {.to = 3, .weight = 1}, {.to = 0, .weight = 1}, {.to = 2, .weight = 1},
        {.to = 1, .weight = 1}, {.to = 3, .weight = 1}, {.to = 0, .weight = 1}, {.to = 2, .weight = 1},
    };
    return graph;
}

bool check_valid_partition(const transport::Graph &graph, uint16_t regions, transport::PartitionMethod method) {
    const auto result = transport::build_partition(graph, regions, method);
    if (result.size() != graph.vertex_count()) {
        std::cerr << "partition: wrong result size (regions=" << regions << ")\n";
        return false;
    }
    for (const uint16_t r : result) {
        if (r >= regions) {
            std::cerr << "partition: region id " << r << " out of range [0, " << regions << ")\n";
            return false;
        }
    }
    return true;
}

bool check_partition() {
    const transport::Graph graph = make_coord_graph();

    for (const uint16_t regions : {uint16_t{1}, uint16_t{4}, uint16_t{9}}) {
        if (!check_valid_partition(graph, regions, transport::PartitionMethod::Grid)) {
            return false;
        }
    }

    for (const uint16_t regions : {uint16_t{1}, uint16_t{2}, uint16_t{4}}) {
        if (!check_valid_partition(graph, regions, transport::PartitionMethod::Inertial)) {
            return false;
        }
    }

    // Inertial must throw on non-power-of-2 region count.
    bool threw = false;
    try {
        (void)transport::build_partition(graph, uint16_t{3}, transport::PartitionMethod::Inertial);
    } catch (const std::invalid_argument &) {
        threw = true;
    }
    if (!threw) {
        std::cerr << "partition inertial: expected throw for regions=3 (not a power of 2)\n";
        return false;
    }

#ifdef TRANSPORT_HAVE_KAMINPAR
    for (const uint16_t regions : {uint16_t{2}, uint16_t{4}}) {
        if (!check_valid_partition(graph, regions, transport::PartitionMethod::KaMinPar)) {
            return false;
        }
    }
#endif

    // parse_partition_method / partition_method_name round-trip.
    for (const char *name : {"grid", "inertial", "kaminpar"}) {
        const auto method = transport::parse_partition_method(name);
        if (transport::partition_method_name(method) != name) {
            std::cerr << "partition: name round-trip failed for '" << name << "'\n";
            return false;
        }
    }

    return true;
}

} // namespace

int main() {
    if (!check_partition()) {
        return 1;
    }
    return 0;
}
