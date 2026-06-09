#include "algorithms/astar.hpp"

#include "algorithms/heap_node.hpp"

#include <functional>
#include <queue>
#include <string_view>
#include <vector>

namespace transport {

AStarAlgorithm::AStarAlgorithm(const Graph &graph, Heuristic heuristic)
    : graph_(graph), heuristic_(std::move(heuristic)), g_(graph.vertex_count(), kUnreachable) {}

std::string_view AStarAlgorithm::name() const { return "astar"; }

PathResult AStarAlgorithm::query(VertexId source, VertexId target) const {
    g_.reset();
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;

    g_.set(source, 0);
    pq.push({heuristic_(source, target), source});

    uint32_t settled = 0;
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();

        const Distance current_f = g_.get(top.v) + heuristic_(top.v, target);
        if (top.key != current_f) {
            continue;
        }

        ++settled;
        if (top.v == target) {
            break;
        }

        for (const Edge &e : graph_.adjacent_edges(top.v)) {
            const Distance ng = g_.get(top.v) + e.weight;
            if (ng < g_.get(e.to)) {
                g_.set(e.to, ng);
                pq.push({ng + heuristic_(e.to, target), e.to});
            }
        }
    }

    return PathResult{g_.get(target), settled};
}

} // namespace transport
