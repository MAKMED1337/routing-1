#include "algorithms/astar.hpp"
#include "algorithms/bidirectional_astar.hpp"
#include "algorithms/heuristic.hpp"
#include "graph/graph.hpp"
#include "graph_fixtures.hpp"
#include "routing_test_utils.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <span>
#include <string_view>
#include <utility>

using AlgorithmFactory = std::function<std::unique_ptr<transport::RoutingAlgorithm>(transport::Heuristic)>;

bool check_grid_heuristics(const transport::Graph &graph, std::span<const transport::NodeCoord> coords,
                           std::string_view algorithm_name, const AlgorithmFactory &make_algorithm) {
    auto geometry_heuristic = [coords](transport::VertexId from, transport::VertexId to) -> transport::Distance {
        const double d_row = coords[from].lat - coords[to].lat;
        const double d_col = coords[from].lon - coords[to].lon;
        return static_cast<transport::Distance>(std::floor(std::hypot(d_row, d_col) * 10.0));
    };

    auto manhattan_heuristic = [coords](transport::VertexId from, transport::VertexId to) -> transport::Distance {
        const double d_row = std::fabs(coords[from].lat - coords[to].lat);
        const double d_col = std::fabs(coords[from].lon - coords[to].lon);
        return static_cast<transport::Distance>((d_row + d_col) * 10.0);
    };

    auto geometry_algorithm = make_algorithm(geometry_heuristic);
    geometry_algorithm->preprocess();
    if (!transport::test::check_all_pairs(graph, *geometry_algorithm, "heuristic=geometry")) {
        std::cerr << "failed algorithm=" << algorithm_name << "\n";
        return false;
    }

    auto manhattan_algorithm = make_algorithm(manhattan_heuristic);
    manhattan_algorithm->preprocess();
    if (!transport::test::check_all_pairs(graph, *manhattan_algorithm, "heuristic=manhattan")) {
        std::cerr << "failed algorithm=" << algorithm_name << "\n";
        return false;
    }

    return true;
}

int main() {
    const transport::test::GraphWithCoords fixture = transport::test::make_grid_graph(5, 7);
    const transport::Graph &graph = fixture.graph;
    const std::span<const transport::NodeCoord> coords = fixture.coords;

    auto make_astar = [&graph](transport::Heuristic heuristic) -> std::unique_ptr<transport::RoutingAlgorithm> {
        return std::make_unique<transport::AStarAlgorithm>(graph, std::move(heuristic));
    };
    auto make_bidi_astar = [&graph](transport::Heuristic heuristic) -> std::unique_ptr<transport::RoutingAlgorithm> {
        return std::make_unique<transport::BidirectionalAStarAlgorithm>(graph, std::move(heuristic));
    };

    if (!check_grid_heuristics(graph, coords, "astar", make_astar)) {
        return 1;
    }
    if (!check_grid_heuristics(graph, coords, "bidi_astar", make_bidi_astar)) {
        return 1;
    }
    return 0;
}
