#include "graph/graph_io.hpp"
#include "routing_test_utils.hpp"

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <stdexcept>
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

bool expect_graph_load_failure(const std::filesystem::path &path) {
    try {
        (void)transport::load_graph_binary(path.string());
    } catch (const std::runtime_error &) {
        return true;
    }
    std::cerr << "graph: expected load failure for " << path << "\n";
    return false;
}

bool check_truncated_graph_headers_fail_fast() {
    const transport::Graph graph = transport::test::make_graph(1, {{}});
    const std::filesystem::path dir = std::filesystem::temp_directory_path();
    const std::filesystem::path empty_path = dir / "transport_empty_header.graph";
    const std::filesystem::path magic_only_path = dir / "transport_magic_only.graph";

    if (!transport::save_graph_binary(graph, empty_path.string()) ||
        !transport::save_graph_binary(graph, magic_only_path.string())) {
        std::cerr << "graph: failed to save truncation fixtures\n";
        return false;
    }
    std::filesystem::resize_file(empty_path, 0);
    std::filesystem::resize_file(magic_only_path, sizeof(uint32_t));

    const bool ok = expect_graph_load_failure(empty_path) && expect_graph_load_failure(magic_only_path);
    std::filesystem::remove(empty_path);
    std::filesystem::remove(magic_only_path);
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok &= check_save_load_round_trip();
    ok &= check_coords_save_load_round_trip();
    ok &= check_truncated_graph_headers_fail_fast();
    if (!ok) {
        std::cerr << "graph io tests FAILED\n";
        return 1;
    }
    return 0;
}
