#pragma once

#include "graph/graph.hpp"
#include "routing_test_utils.hpp"

#include <cstdint>
#include <vector>

namespace transport::test {

// Pairs a graph with the external coordinates needed by geometry-dependent algorithms
// (A*, ALT planar, grid/inertial partitioning), since Graph itself no longer stores them.
struct GraphWithCoords {
    Graph graph;
    std::vector<NodeCoord> coords;
};

// --- Graphs used by routing algorithm correctness tests ---

// Linear directed chain: 0→1→2→3 (each edge weight 1).
inline Graph make_line_graph() {
    return make_graph(4, {
                             /*0*/ {{1, 1}},
                             /*1*/ {{2, 1}},
                             /*2*/ {{3, 1}},
                             /*3*/ {},
                         });
}

// 5-vertex graph exercising witness search in CH contraction.
inline Graph make_witness_graph() {
    return make_graph(5, {
                             /*0*/ {{1, 2}, {2, 10}},
                             /*1*/ {{2, 2}, {3, 20}},
                             /*2*/ {{3, 2}},
                             /*3*/ {{4, 2}},
                             /*4*/ {{1, 1}},
                         });
}

// 6-vertex directed graph with multiple asymmetric shortest paths.
inline Graph make_asymmetric_graph() {
    return make_graph(6, {
                             /*0*/ {{1, 1}, {4, 20}},
                             /*1*/ {{2, 1}},
                             /*2*/ {{3, 1}},
                             /*3*/ {{5, 1}},
                             /*4*/ {{3, 1}},
                             /*5*/ {{1, 50}},
                         });
}

// 3-vertex directed chain: 0→1→2.
inline Graph make_short_chain_graph() {
    return make_graph(3, {
                             /*0*/ {{1, 1}},
                             /*1*/ {{2, 1}},
                             /*2*/ {},
                         });
}

// 5-vertex graph with two disconnected components: {0,1,2} and {3,4}.
inline Graph make_disconnected_graph() {
    return make_graph(5, {
                             /*0*/ {{1, 5}},
                             /*1*/ {{2, 5}},
                             /*2*/ {},
                             /*3*/ {{4, 1}},
                             /*4*/ {},
                         });
}

// --- Graphs used by PHAST tests ---

// 4-vertex directed graph where d(a,b) ≠ d(b,a) for many pairs.
// 0→1 (1), 1→2 (1), 2→1 (10). Vertex 3 is isolated.
inline Graph make_directed_asymmetric_graph() {
    return make_graph(4, {
                             /*0*/ {Edge{.to = 1, .weight = 1}},
                             /*1*/ {Edge{.to = 2, .weight = 1}},
                             /*2*/ {Edge{.to = 1, .weight = 10}},
                             /*3*/ {},
                         });
}

// Symmetric 4-vertex ring: 0↔1↔2↔3↔0.
inline Graph make_symmetric_graph() {
    return make_graph(4, {
                             /*0*/ {Edge{.to = 1, .weight = 2}, Edge{.to = 3, .weight = 5}},
                             /*1*/ {Edge{.to = 0, .weight = 2}, Edge{.to = 2, .weight = 3}},
                             /*2*/ {Edge{.to = 1, .weight = 3}, Edge{.to = 3, .weight = 1}},
                             /*3*/ {Edge{.to = 2, .weight = 1}, Edge{.to = 0, .weight = 5}},
                         });
}

// --- Graphs used by partition tests ---

// 4-vertex symmetric ring with geographic coordinates spread over a small lat/lon box.
// Suitable for grid and inertial partitioning tests.
inline GraphWithCoords make_coord_graph() {
    Graph graph;
    graph.vertex_count_ = 4;
    // Symmetric ring: 0→1, 1→2, 2→3, 3→0 and back-edges.
    graph.offsets = {0, 2, 4, 6, 8};
    graph.edges = {
        {.to = 1, .weight = 1}, {.to = 3, .weight = 1}, {.to = 0, .weight = 1}, {.to = 2, .weight = 1},
        {.to = 1, .weight = 1}, {.to = 3, .weight = 1}, {.to = 0, .weight = 1}, {.to = 2, .weight = 1},
    };
    std::vector<NodeCoord> coords = {
        {.lat = 10.0, .lon = 20.0},
        {.lat = 10.0, .lon = 21.0},
        {.lat = 11.0, .lon = 21.0},
        {.lat = 11.0, .lon = 20.0},
    };
    return GraphWithCoords{.graph = std::move(graph), .coords = std::move(coords)};
}

// --- Graphs used by A* heuristic tests ---

// rows×cols grid graph with unit-step edges and coordinates set to (row, col).
inline GraphWithCoords make_grid_graph(uint32_t rows, uint32_t cols) {
    constexpr Weight kStepCost = 10;
    const uint32_t vertices = rows * cols;
    std::vector<std::vector<Edge>> edges(vertices);

    auto vertex = [cols](uint32_t row, uint32_t col) -> VertexId { return row * cols + col; };

    for (uint32_t row = 0; row < rows; ++row) {
        for (uint32_t col = 0; col < cols; ++col) {
            const VertexId from = vertex(row, col);
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

    Graph graph = make_graph(vertices, edges);
    std::vector<NodeCoord> coords(vertices);
    for (uint32_t row = 0; row < rows; ++row) {
        for (uint32_t col = 0; col < cols; ++col) {
            coords[vertex(row, col)] = {static_cast<double>(row), static_cast<double>(col)};
        }
    }
    return GraphWithCoords{.graph = std::move(graph), .coords = std::move(coords)};
}

} // namespace transport::test
