#include "algorithms/dijkstra.hpp"

#include "algorithms/heap_node.hpp"

#include <string_view>

namespace transport {

DijkstraAlgorithm::DijkstraAlgorithm(const Graph &graph) : graph_(graph), dist_(graph.vertex_count(), kUnreachable) {}

std::string_view DijkstraAlgorithm::name() const { return "dijkstra"; }

PathResult DijkstraAlgorithm::query(VertexId source, VertexId target) const {
    dist_.reset();
    HeapQueue pq;
    dist_.set(source, 0);
    pq.push({0, source});

    QueryStats stats;
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != dist_.get(top.v)) {
            continue;
        }

        ++stats.settled;
        if (top.v == target) {
            break;
        }

        for (const Edge &e : graph_.adjacent_edges(top.v)) {
            ++stats.relaxed_arcs;
            const Distance nd = top.key + e.weight;
            if (nd < dist_.get(e.to)) {
                dist_.set(e.to, nd);
                pq.push({nd, e.to});
                ++stats.heap_pushes;
            }
        }
    }

    return PathResult{dist_.get(target), stats};
}

} // namespace transport
