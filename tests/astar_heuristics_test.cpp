#include "algorithms/astar.hpp"
#include "algorithms/bidirectional_astar.hpp"
#include "algorithms/heuristic.hpp"
#include "graph/graph.hpp"
#include "routing_test_utils.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string_view>
#include <utility>
#include <vector>

namespace {

transport::Graph make_grid_graph(uint32_t rows, uint32_t cols) {
    constexpr transport::Weight kStepCost = 10;
    const uint32_t vertices = rows * cols;
    std::vector<std::vector<transport::Edge>> edges(vertices);

    auto vertex = [cols](uint32_t row, uint32_t col) -> transport::VertexId { return row * cols + col; };

    for (uint32_t row = 0; row < rows; ++row) {
        for (uint32_t col = 0; col < cols; ++col) {
            const transport::VertexId from = vertex(row, col);
            if (row > 0) {
                edges[from].push_back({vertex(row - 1, col), kStepCost});
            }
            if (row + 1 < rows) {
                edges[from].push_back({vertex(row + 1, col), kStepCost});
            }
            if (col > 0) {
                edges[from].push_back({vertex(row, col - 1), kStepCost});
            }
            if (col + 1 < cols) {
                edges[from].push_back({vertex(row, col + 1), kStepCost});
            }
        }
    }

    transport::Graph graph = transport::test::make_graph(vertices, edges);
    for (uint32_t row = 0; row < rows; ++row) {
        for (uint32_t col = 0; col < cols; ++col) {
            graph.coords[vertex(row, col)] = {static_cast<double>(row), static_cast<double>(col)};
        }
    }
    return graph;
}

using AlgorithmFactory = std::function<std::unique_ptr<transport::RoutingAlgorithm>(transport::Heuristic)>;

bool check_grid_heuristics(const transport::Graph &graph, std::string_view algorithm_name,
                           const AlgorithmFactory &make_algorithm) {
    auto geometry_heuristic = [&graph](transport::VertexId from, transport::VertexId to) -> transport::Distance {
        const double d_row = graph.coords[from].lat - graph.coords[to].lat;
        const double d_col = graph.coords[from].lon - graph.coords[to].lon;
        return static_cast<transport::Distance>(std::floor(std::hypot(d_row, d_col) * 10.0));
    };

    auto manhattan_heuristic = [&graph](transport::VertexId from, transport::VertexId to) -> transport::Distance {
        const double d_row = std::fabs(graph.coords[from].lat - graph.coords[to].lat);
        const double d_col = std::fabs(graph.coords[from].lon - graph.coords[to].lon);
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

} // namespace

int main() {
    const transport::Graph graph = make_grid_graph(5, 7);

    auto make_astar = [&graph](transport::Heuristic heuristic) -> std::unique_ptr<transport::RoutingAlgorithm> {
        return std::make_unique<transport::AStarAlgorithm>(graph, std::move(heuristic));
    };
    auto make_bidi_astar = [&graph](transport::Heuristic heuristic) -> std::unique_ptr<transport::RoutingAlgorithm> {
        return std::make_unique<transport::BidirectionalAStarAlgorithm>(graph, std::move(heuristic));
    };

    if (!check_grid_heuristics(graph, "astar", make_astar)) {
        return 1;
    }
    if (!check_grid_heuristics(graph, "bidi_astar", make_bidi_astar)) {
        return 1;
    }
    return 0;
}
