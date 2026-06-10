#include "graph/graph.hpp"
#include "routing_test_utils.hpp"

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <vector>

namespace {

bool check_save_load_round_trip() {
    const transport::Graph graph = transport::test::make_graph(4, {
                                                                      /*0*/ {{1, 1}, {2, 5}},
                                                                      /*1*/ {{2, 1}},
                                                                      /*2*/ {{3, 1}},
                                                                      /*3*/ {},
                                                                  });

    const std::filesystem::path path = std::filesystem::temp_directory_path() / "transport_graph_round_trip.bin";
    if (!transport::save_graph_binary(graph, path.string())) {
        std::cerr << "graph: failed to save graph\n";
        return false;
    }

    const transport::Graph loaded = transport::load_graph_binary(path.string());
    std::filesystem::remove(path);

    if (loaded.vertex_count() != graph.vertex_count()) {
        std::cerr << "graph: vertex count mismatch after round-trip\n";
        return false;
    }
    if (loaded.offsets != graph.offsets) {
        std::cerr << "graph: offsets mismatch after round-trip\n";
        return false;
    }
    if (loaded.edges.size() != graph.edges.size()) {
        std::cerr << "graph: edge count mismatch after round-trip\n";
        return false;
    }
    for (size_t i = 0; i < graph.edges.size(); ++i) {
        if (loaded.edges[i].to != graph.edges[i].to || loaded.edges[i].weight != graph.edges[i].weight) {
            std::cerr << "graph: edge mismatch after round-trip\n";
            return false;
        }
    }
    return true;
}

bool check_coords_save_load_round_trip() {
    const std::vector<transport::NodeCoord> coords = {
        {.lat = 10.0, .lon = 20.0},
        {.lat = 11.5, .lon = -3.25},
    };

    const std::filesystem::path path = std::filesystem::temp_directory_path() / "transport_coords_round_trip.bin";
    if (!transport::save_coords_binary(coords, path.string())) {
        std::cerr << "graph: failed to save coords\n";
        return false;
    }

    const std::vector<transport::NodeCoord> loaded = transport::load_coords_binary(path.string());
    std::filesystem::remove(path);

    if (loaded.size() != coords.size()) {
        std::cerr << "graph: coords count mismatch after round-trip\n";
        return false;
    }
    for (size_t i = 0; i < coords.size(); ++i) {
        if (loaded[i].lat != coords[i].lat || loaded[i].lon != coords[i].lon) {
            std::cerr << "graph: coords value mismatch after round-trip\n";
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    bool ok = true;
    ok &= check_save_load_round_trip();
    ok &= check_coords_save_load_round_trip();
    if (!ok) {
        std::cerr << "graph io tests FAILED\n";
        return 1;
    }
    return 0;
}
