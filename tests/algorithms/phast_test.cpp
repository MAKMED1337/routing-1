#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/phast.hpp"
#include "graph/graph.hpp"
#include "graph_fixtures.hpp"

#include <iostream>
#include <vector>

namespace {

using transport::ContractionHierarchy;
using transport::ContractionHierarchyAlgorithm;
using transport::DijkstraAlgorithm;
using transport::Distance;
using transport::Graph;
using transport::VertexId;

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

} // namespace

int main() {
    bool ok = true;
    ok &= check_phast(transport::test::make_directed_asymmetric_graph(), "directed_asymmetric");
    ok &= check_phast(transport::test::make_symmetric_graph(), "symmetric");
    if (!ok) {
        std::cerr << "phast tests FAILED\n";
        return 1;
    }
    std::cout << "phast tests passed\n";
    return 0;
}
