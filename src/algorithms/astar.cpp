#include "algorithms/astar.hpp"

#include "algorithms/heap_node.hpp"

#include <string_view>

namespace transport {

AStarAlgorithm::AStarAlgorithm(const Graph &graph, Heuristic heuristic)
    : graph_(graph), heuristic_(std::move(heuristic)), g_(graph.vertex_count(), kUnreachable) {}

std::string_view AStarAlgorithm::name() const { return "astar"; }

PathResult AStarAlgorithm::query(VertexId source, VertexId target) const {
    g_.reset();
    HeapQueue pq;

    g_.set(source, 0);
    pq.push({heuristic_(source, target), source});

    QueryStats stats;
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();

        ++stats.heuristic_evals;
        const Distance current_f = g_.get(top.v) + heuristic_(top.v, target);
        if (top.key != current_f) {
            continue;
        }

        ++stats.settled;
        if (top.v == target) {
            break;
        }

        for (const Edge &e : graph_.adjacent_edges(top.v)) {
            ++stats.relaxed_arcs;
            const Distance ng = g_.get(top.v) + e.weight;
            if (ng < g_.get(e.to)) {
                g_.set(e.to, ng);
                ++stats.heuristic_evals;
                pq.push({ng + heuristic_(e.to, target), e.to});
                ++stats.heap_pushes;
            }
        }
    }

    return PathResult{g_.get(target), stats};
}

} // namespace transport
