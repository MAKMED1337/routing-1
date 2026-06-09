#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/phast.hpp"
#include "graph/graph.hpp"
#include "graph_fixtures.hpp"

#include <iostream>
#include <numeric>
#include <vector>

namespace {

using transport::ContractionHierarchyAlgorithm;
using transport::DijkstraAlgorithm;
using transport::Distance;
using transport::Graph;
using transport::PhastAlgorithm;
using transport::VertexId;

// Check PhastAlgorithm single-target APIs against Dijkstra.
bool check_phast_context(const Graph &graph, const std::string &label) {
    ContractionHierarchyAlgorithm ch_algo(graph);
    ch_algo.preprocess();
    const PhastAlgorithm ctx(ch_algo.get_ch());

    const DijkstraAlgorithm dijkstra(graph);
    const VertexId V = graph.vertex_count();
    std::vector<Distance> dist;

    for (VertexId target = 0; target < V; ++target) {
        ctx.all_to_one(target, dist);
        for (VertexId v = 0; v < V; ++v) {
            const Distance expected = dijkstra.query(v, target).distance_units;
            if (dist[v] != expected) {
                std::cerr << "all_to_one[" << label << "] mismatch: v=" << v << " target=" << target
                          << " expected=" << expected << " got=" << dist[v] << "\n";
                return false;
            }
        }
    }

    for (VertexId source = 0; source < V; ++source) {
        ctx.one_to_all(source, dist);
        for (VertexId v = 0; v < V; ++v) {
            const Distance expected = dijkstra.query(source, v).distance_units;
            if (dist[v] != expected) {
                std::cerr << "one_to_all[" << label << "] mismatch: source=" << source << " v=" << v
                          << " expected=" << expected << " got=" << dist[v] << "\n";
                return false;
            }
        }
    }

    return true;
}

// Check all_to_one_batch against Dijkstra for several batch sizes and layouts.
bool check_phast_batch(const Graph &graph, const std::string &label) {
    ContractionHierarchyAlgorithm ch_algo(graph);
    ch_algo.preprocess();
    const PhastAlgorithm ctx(ch_algo.get_ch());

    const DijkstraAlgorithm dijkstra(graph);
    const VertexId V = graph.vertex_count();
    std::vector<Distance> dist;

    auto verify = [&](std::span<const VertexId> targets, const std::string &sub) -> bool {
        const size_t B = targets.size();
        ctx.all_to_one_batch(targets, dist);
        for (VertexId v = 0; v < V; ++v) {
            for (size_t i = 0; i < B; ++i) {
                const Distance expected = dijkstra.query(v, targets[i]).distance_units;
                if (dist[v * B + i] != expected) {
                    std::cerr << "all_to_one_batch[" << label << "/" << sub << "] mismatch: v=" << v << " targets[" << i
                              << "]=" << targets[i] << " expected=" << expected << " got=" << dist[v * B + i] << "\n";
                    return false;
                }
            }
        }
        return true;
    };

    bool ok = true;

    // B=0: empty span — dist must be cleared, no UB.
    {
        const std::vector<VertexId> empty;
        ctx.all_to_one_batch(empty, dist);
        if (!dist.empty()) {
            std::cerr << "all_to_one_batch[" << label << "/B=0] expected empty dist, got size=" << dist.size() << "\n";
            return false;
        }
    }

    // B=1: degenerate batch matches single-target.
    if (V > 0) {
        const std::vector<VertexId> t1 = {0};
        ok &= verify(t1, "B=1");
    }

    // B=2: two distinct targets.
    if (V > 1) {
        const std::vector<VertexId> t2 = {0, V - 1};
        ok &= verify(t2, "B=2");
    }

    // B=V: all vertices as targets.
    {
        std::vector<VertexId> all(V);
        std::iota(all.begin(), all.end(), VertexId{0});
        ok &= verify(all, "B=V");
    }

    // Repeated targets: lanes 0 and 1 must agree (lane independence).
    if (V > 1) {
        const std::vector<VertexId> rep = {0, 0, V - 1};
        ok &= verify(rep, "repeated");
    }

    return ok;
}

} // namespace

int main() {
    const std::vector<std::pair<std::string, Graph>> fixtures = {
        {"directed_asymmetric", transport::test::make_directed_asymmetric_graph()},
        {"symmetric", transport::test::make_symmetric_graph()},
        {"disconnected", transport::test::make_disconnected_graph()},
    };

    bool ok = true;
    for (const auto &[name, graph] : fixtures) {
        ok &= check_phast_context(graph, name);
        ok &= check_phast_batch(graph, name);
    }

    if (!ok) {
        std::cerr << "phast tests FAILED\n";
        return 1;
    }
    std::cout << "phast tests passed\n";
    return 0;
}
