#include "algorithms/alt/alt.hpp"
#include "algorithms/arcflags/arc_flags.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/bidirectional_astar.hpp"
#include "algorithms/bidirectional_dijkstra.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/chase/chase.hpp"
#include "algorithms/hl/hub_labels.hpp"
#include "algorithms/phast.hpp"
#include "graph/graph_io.hpp"
#include "graph/reverse_graph.hpp"
#include "graph_fixtures.hpp"
#include "routing_test_utils.hpp"

#include <filesystem>
#include <iostream>
#include <random>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

using transport::test::check_all_pairs;

transport::Graph make_invalid_offsets_graph() {
    transport::Graph graph;
    graph.vertex_count_ = 2;
    graph.offsets = {0, 2, 1};
    graph.edges.push_back(transport::Edge{
        .to = 1,
        .weight = 100,
    });
    return graph;
}

transport::Graph make_invalid_edge_destination_graph() {
    transport::Graph graph;
    graph.vertex_count_ = 2;
    graph.offsets = {0, 1, 1};
    graph.edges.push_back(transport::Edge{
        .to = 5,
        .weight = 100,
    });
    return graph;
}

bool expect_load_failure(const std::filesystem::path &path) {
    try {
        (void)transport::load_graph_binary(path.string());
    } catch (const std::runtime_error &) {
        return true;
    }
    std::cerr << "expected malformed graph load failure for " << path << "\n";
    return false;
}

bool check_malformed_graph_files_fail_fast() {
    const std::filesystem::path dir = std::filesystem::temp_directory_path();
    const std::filesystem::path invalid_offsets = dir / "transport_invalid_offsets.graph";
    const std::filesystem::path invalid_edge = dir / "transport_invalid_edge.graph";

    if (!transport::save_graph_binary(make_invalid_offsets_graph(), invalid_offsets.string()) ||
        !transport::save_graph_binary(make_invalid_edge_destination_graph(), invalid_edge.string())) {
        std::cerr << "failed to write malformed graph fixtures\n";
        return false;
    }

    const bool ok = expect_load_failure(invalid_offsets) && expect_load_failure(invalid_edge);
    std::filesystem::remove(invalid_offsets);
    std::filesystem::remove(invalid_edge);
    return ok;
}

bool check_all_algorithms(const transport::Graph &graph, std::span<const transport::NodeCoord> coords = {}) {
    std::vector<transport::NodeCoord> zero_coords;
    if (coords.empty()) {
        zero_coords.assign(graph.vertex_count(), transport::NodeCoord{});
        coords = zero_coords;
    }

    auto zero_heuristic = [](transport::VertexId, transport::VertexId) -> transport::Distance { return 0; };
    transport::AStarAlgorithm astar(graph, zero_heuristic);
    std::mt19937 alt_rng{1};
    const transport::Graph reverse = transport::build_reverse_graph(graph);
    transport::alt::LandmarkSet alt_landmarks =
        transport::alt::build_landmarks(graph, reverse, 4, transport::alt::LandmarkStrategy::Farthest, alt_rng);
    transport::AltAlgorithm alt(graph, std::move(alt_landmarks), 2);
    transport::BidirectionalAStarAlgorithm bidi_astar(graph, zero_heuristic);
    transport::BidirectionalDijkstraAlgorithm bidijkstra(graph);
    transport::ContractionHierarchyAlgorithm ch(graph);
    bidi_astar.preprocess();
    bidijkstra.preprocess();
    ch.preprocess();

    // threads=1, threads=2, and threads=16 (more threads than work blocks on small graphs).
    transport::ArcFlagsAlgorithm af1(graph, transport::PhastAlgorithm(ch.get_ch()), 4, transport::PartitionMethod::Grid,
                                     1, coords);
    transport::ArcFlagsAlgorithm af2(graph, transport::PhastAlgorithm(ch.get_ch()), 4, transport::PartitionMethod::Grid,
                                     2, coords);
    transport::ArcFlagsAlgorithm af16(graph, transport::PhastAlgorithm(ch.get_ch()), 4,
                                      transport::PartitionMethod::Grid, 16, coords);
    af1.preprocess();
    af2.preprocess();
    af16.preprocess();

    // core_fraction=0.5 so even small graphs have a non-trivial core; Grid partition, 4 regions.
    transport::ChaseAlgorithm chase(graph, 0.5, 4, transport::PartitionMethod::Grid, coords);
    chase.preprocess();

    // full labels (label_fraction=1.0) and tiered labels (label_fraction=0.5).
    transport::HubLabelsAlgorithm hl_full(graph, 1.0, 4ULL * 1024 * 1024 * 1024);
    transport::HubLabelsAlgorithm hl_tiered(graph, 0.5, 4ULL * 1024 * 1024 * 1024);
    hl_full.preprocess();
    hl_tiered.preprocess();

    return check_all_pairs(graph, astar) && check_all_pairs(graph, alt) && check_all_pairs(graph, bidi_astar) &&
           check_all_pairs(graph, bidijkstra) && check_all_pairs(graph, ch) && check_all_pairs(graph, af1) &&
           check_all_pairs(graph, af2) && check_all_pairs(graph, af16) && check_all_pairs(graph, chase) &&
           check_all_pairs(graph, hl_full) && check_all_pairs(graph, hl_tiered);
}

} // namespace

int main() {
    bool ok = true;
    ok &= check_all_algorithms(transport::test::make_line_graph());
    ok &= check_all_algorithms(transport::test::make_witness_graph());
    ok &= check_all_algorithms(transport::test::make_asymmetric_graph());
    ok &= check_all_algorithms(transport::test::make_disconnected_graph());
    {
        const transport::test::GraphWithCoords grid = transport::test::make_grid_graph(4, 4);
        ok &= check_all_algorithms(grid.graph, grid.coords);
    }
    ok &= check_malformed_graph_files_fail_fast();
    if (!ok) {
        std::cerr << "routing algorithm tests FAILED\n";
        return 1;
    }
    return 0;
}
