#include "algorithms/astar.hpp"
#include "algorithms/bidirectional_astar.hpp"
#include "algorithms/bidirectional_dijkstra.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/dijkstra.hpp"
#include "graph/graph.hpp"

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

transport::Graph make_graph(uint32_t vertices, const std::vector<std::vector<transport::Edge>> &rows) {
    transport::Graph graph;
    graph.coords.resize(vertices);
    graph.offsets.assign(static_cast<size_t>(vertices) + 1, 0);
    for (uint32_t v = 0; v < vertices; ++v) {
        graph.offsets[v + 1] = graph.offsets[v] + static_cast<uint64_t>(rows[v].size());
    }
    for (const std::vector<transport::Edge> &row : rows) {
        graph.edges.insert(graph.edges.end(), row.begin(), row.end());
    }
    return graph;
}

bool same_distance(uint64_t a, uint64_t b) { return a == b; }

transport::Graph make_invalid_offsets_graph() {
    transport::Graph graph;
    graph.coords.resize(2);
    graph.offsets = {0, 2, 1};
    graph.edges.push_back(transport::Edge{
        .to = 1,
        .weight = 100,
    });
    return graph;
}

transport::Graph make_invalid_edge_destination_graph() {
    transport::Graph graph;
    graph.coords.resize(2);
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

bool check_all_pairs(const transport::Graph &graph, const transport::RoutingAlgorithm &algorithm) {
    const transport::DijkstraAlgorithm dijkstra(graph);
    for (uint32_t s = 0; s < graph.vertex_count(); ++s) {
        for (uint32_t t = 0; t < graph.vertex_count(); ++t) {
            const transport::PathResult d = dijkstra.query(s, t);
            const transport::PathResult result = algorithm.query(s, t);
            if (!same_distance(d.distance_units, result.distance_units)) {
                std::cerr << "mismatch source=" << s << " target=" << t << " dijkstra=" << d.distance_units << " "
                          << algorithm.name() << "=" << result.distance_units << "\n";
                return false;
            }
        }
    }
    return true;
}

bool check_all_algorithms(const transport::Graph &graph) {
    auto zero_heuristic = [](transport::VertexId, transport::VertexId) -> transport::Distance { return 0; };
    transport::AStarAlgorithm astar(graph, zero_heuristic);
    transport::BidirectionalAStarAlgorithm bidi_astar(graph, zero_heuristic);
    transport::BidirectionalDijkstraAlgorithm bidijkstra(graph);
    transport::ContractionHierarchyAlgorithm ch(graph);
    bidi_astar.preprocess();
    bidijkstra.preprocess();
    ch.preprocess();
    return check_all_pairs(graph, astar) && check_all_pairs(graph, bidi_astar) && check_all_pairs(graph, bidijkstra) &&
           check_all_pairs(graph, ch);
}

} // namespace

int main() {
    const transport::Graph line = make_graph(4, {
                                                    {{1, 1}},
                                                    {{2, 1}},
                                                    {{3, 1}},
                                                    {},
                                                });
    if (!check_all_algorithms(line)) {
        return 1;
    }

    const transport::Graph directed_with_witness = make_graph(5, {
                                                                     {{1, 2}, {2, 10}},
                                                                     {{2, 2}, {3, 20}},
                                                                     {{3, 2}},
                                                                     {{4, 2}},
                                                                     {{1, 1}},
                                                                 });
    if (!check_all_algorithms(directed_with_witness)) {
        return 1;
    }

    const transport::Graph asymmetric = make_graph(6, {
                                                          {{1, 1}, {4, 20}},
                                                          {{2, 1}},
                                                          {{3, 1}},
                                                          {{5, 1}},
                                                          {{3, 1}},
                                                          {{1, 50}},
                                                      });
    if (!check_all_algorithms(asymmetric)) {
        return 1;
    }

    const transport::Graph disconnected = make_graph(5, {
                                                            {{1, 5}},
                                                            {{2, 5}},
                                                            {},
                                                            {{4, 1}},
                                                            {},
                                                        });
    if (!check_all_algorithms(disconnected)) {
        return 1;
    }

    if (!check_malformed_graph_files_fail_fast()) {
        return 1;
    }

    return 0;
}
