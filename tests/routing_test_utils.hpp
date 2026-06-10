#pragma once

#include "algorithms/dijkstra.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "graph/graph.hpp"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

namespace transport::test {

inline Graph make_graph(uint32_t vertices, const std::vector<std::vector<Edge>> &rows) {
    Graph graph;
    graph.vertex_count_ = vertices;
    graph.offsets.assign(static_cast<size_t>(vertices) + 1, 0);
    for (uint32_t v = 0; v < vertices; ++v) {
        graph.offsets[v + 1] = graph.offsets[v] + rows[v].size();
    }
    for (const std::vector<Edge> &row : rows) {
        graph.edges.insert(graph.edges.end(), row.begin(), row.end());
    }
    return graph;
}

inline bool check_all_pairs(const Graph &graph, const RoutingAlgorithm &algorithm, std::string_view context = {}) {
    const DijkstraAlgorithm dijkstra(graph);
    for (VertexId source = 0; source < graph.vertex_count(); ++source) {
        for (VertexId target = 0; target < graph.vertex_count(); ++target) {
            const Distance expected = dijkstra.query(source, target).distance_units;
            const Distance got = algorithm.query(source, target).distance_units;
            if (expected != got) {
                std::cerr << "mismatch";
                if (!context.empty()) {
                    std::cerr << " " << context;
                }
                std::cerr << " source=" << source << " target=" << target << " dijkstra=" << expected << " "
                          << algorithm.name() << "=" << got << "\n";
                return false;
            }
        }
    }
    return true;
}

} // namespace transport::test
