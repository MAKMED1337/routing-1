#include "algorithms/alt/alt.hpp"
#include "algorithms/arcflags/arc_flags.hpp"
#include "algorithms/astar.hpp"
#include "algorithms/bidirectional_astar.hpp"
#include "algorithms/bidirectional_dijkstra.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/chase/chase.hpp"
#include "algorithms/hl/hub_labels.hpp"
#include "algorithms/phast.hpp"
#include "graph/reverse_graph.hpp"
#include "graph_fixtures.hpp"
#include "routing_test_utils.hpp"

#include <iostream>
#include <random>
#include <span>
#include <vector>

namespace {

using transport::test::check_all_pairs;

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

    // Shared CH/PHAST built once and injected into every CH-dependent algorithm below.
    const transport::PhastAlgorithm phast(ch.get_ch());

    // threads=1, threads=2, and threads=16 (more threads than work blocks on small graphs).
    transport::ArcFlagsAlgorithm af1(graph, phast, 4, transport::PartitionMethod::Grid, 1, coords);
    transport::ArcFlagsAlgorithm af2(graph, phast, 4, transport::PartitionMethod::Grid, 2, coords);
    transport::ArcFlagsAlgorithm af16(graph, phast, 4, transport::PartitionMethod::Grid, 16, coords);
    af1.preprocess();
    af2.preprocess();
    af16.preprocess();

    // core_fraction=0.5 so even small graphs have a non-trivial core; Grid partition, 4 regions.
    transport::ChaseAlgorithm chase(graph, ch.get_ch(), 0.5, 4, transport::PartitionMethod::Grid, coords);
    chase.preprocess();

    // full labels (label_fraction=1.0) and tiered labels (label_fraction=0.5).
    transport::HubLabelsAlgorithm hl_full(graph, ch.get_ch(), 1.0, 4ULL * 1024 * 1024 * 1024);
    transport::HubLabelsAlgorithm hl_tiered(graph, ch.get_ch(), 0.5, 4ULL * 1024 * 1024 * 1024);
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
    if (!ok) {
        std::cerr << "routing algorithm tests FAILED\n";
        return 1;
    }
    return 0;
}
