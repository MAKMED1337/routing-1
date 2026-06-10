#include "algorithms/partition.hpp"
#include "graph/graph.hpp"
#include "graph_fixtures.hpp"

#include <iostream>
#include <span>
#include <stdexcept>

bool check_valid_partition(const transport::Graph &graph, uint16_t regions, transport::PartitionMethod method,
                           std::span<const transport::NodeCoord> coords = {}) {
    const auto result = transport::build_partition(graph, regions, method, coords);
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
    const auto [graph, coords] = transport::test::make_coord_graph();

    for (const uint16_t regions : {uint16_t{1}, uint16_t{4}, uint16_t{9}}) {
        if (!check_valid_partition(graph, regions, transport::PartitionMethod::Grid, coords)) {
            return false;
        }
    }

    for (const uint16_t regions : {uint16_t{1}, uint16_t{2}, uint16_t{4}}) {
        if (!check_valid_partition(graph, regions, transport::PartitionMethod::Inertial, coords)) {
            return false;
        }
    }

    // Inertial must throw on non-power-of-2 region count.
    bool threw = false;
    try {
        (void)transport::build_partition(graph, uint16_t{3}, transport::PartitionMethod::Inertial, coords);
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

    // Grid and inertial must throw when coordinates are missing or mismatched.
    for (const transport::PartitionMethod method :
         {transport::PartitionMethod::Grid, transport::PartitionMethod::Inertial}) {
        bool coords_threw = false;
        try {
            (void)transport::build_partition(graph, uint16_t{4}, method);
        } catch (const std::invalid_argument &) {
            coords_threw = true;
        }
        if (!coords_threw) {
            std::cerr << "partition: expected throw for missing coordinates\n";
            return false;
        }
    }

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

int main() {
    if (!check_partition()) {
        return 1;
    }
    return 0;
}
