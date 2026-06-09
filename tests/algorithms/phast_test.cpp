#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/phast.hpp"
#include "graph/graph.hpp"
#include "routing_test_utils.hpp"

#include <iostream>
#include <vector>

namespace {

using transport::ContractionHierarchy;
using transport::ContractionHierarchyAlgorithm;
using transport::DijkstraAlgorithm;
using transport::Distance;
using transport::Edge;
using transport::Graph;
using transport::VertexId;
using transport::test::make_graph;

// Check phast_all_to_one(target): dist[v] == dijkstra(v, target) for all v.
// Check phast_one_to_all(source): dist[v] == dijkstra(source, v) for all v.
bool check_phast(const Graph &graph, const std::string &label) {
    ContractionHierarchyAlgorithm ch_algo(graph);
    ch_algo.preprocess();
    const ContractionHierarchy &ch = ch_algo.get_ch();
    const std::vector<VertexId> inv_rank = transport::build_inv_rank(ch);

    const DijkstraAlgorithm dijkstra(graph);
    const VertexId V = graph.vertex_count();
    std::vector<Distance> dist(V);

    for (VertexId target = 0; target < V; ++target) {
        transport::phast_all_to_one(ch, inv_rank, target, dist);
        for (VertexId v = 0; v < V; ++v) {
            const Distance expected = dijkstra.query(v, target).distance_units;
            if (dist[v] != expected) {
                std::cerr << "phast_all_to_one[" << label << "] mismatch: v=" << v << " target=" << target
                          << " expected=" << expected << " got=" << dist[v] << "\n";
                return false;
            }
        }
    }

    for (VertexId source = 0; source < V; ++source) {
        transport::phast_one_to_all(ch, inv_rank, source, dist);
        for (VertexId v = 0; v < V; ++v) {
            const Distance expected = dijkstra.query(source, v).distance_units;
            if (dist[v] != expected) {
                std::cerr << "phast_one_to_all[" << label << "] mismatch: source=" << source << " v=" << v
                          << " expected=" << expected << " got=" << dist[v] << "\n";
                return false;
            }
        }
    }

    return true;
}

// Directed graph where d(a,b) ≠ d(b,a) for many pairs.
// 0 --1--> 1 --1--> 2
//          ^         |
//          |   10    |
//          +----+----+  (2→1 costs 10, but going 1→2 costs 1)
// Also: 3 is isolated (no edges).
Graph make_directed_asymmetric_graph() {
    return make_graph(4, {
                             /*0*/ {Edge{.to = 1, .weight = 1}},
                             /*1*/ {Edge{.to = 2, .weight = 1}},
                             /*2*/ {Edge{.to = 1, .weight = 10}},
                             /*3*/ {},
                         });
}

// Simple symmetric graph: 0--1--2--3 (undirected ring).
Graph make_symmetric_graph() {
    return make_graph(4, {
                             /*0*/ {Edge{.to = 1, .weight = 2}, Edge{.to = 3, .weight = 5}},
                             /*1*/ {Edge{.to = 0, .weight = 2}, Edge{.to = 2, .weight = 3}},
                             /*2*/ {Edge{.to = 1, .weight = 3}, Edge{.to = 3, .weight = 1}},
                             /*3*/ {Edge{.to = 2, .weight = 1}, Edge{.to = 0, .weight = 5}},
                         });
}

} // namespace

int main() {
    bool ok = true;
    ok = check_phast(make_directed_asymmetric_graph(), "directed_asymmetric") && ok;
    ok = check_phast(make_symmetric_graph(), "symmetric") && ok;
    if (!ok) {
        std::cerr << "phast tests FAILED\n";
        return 1;
    }
    std::cout << "phast tests passed\n";
    return 0;
}
