#pragma once

#include "algorithms/heap_node.hpp"
#include "graph/graph.hpp"
#include "routing/routing.hpp"

#include <algorithm>
#include <concepts>
#include <functional>
#include <queue>
#include <ranges>
#include <vector>

namespace transport {

template <typename F>
concept AdjacencyFn = std::invocable<F, VertexId> && std::ranges::range<std::invoke_result_t<F, VertexId>>;

// Generic Dijkstra from source. dist is reset to kUnreachable, then Dijkstra runs from source.
// dist must be pre-sized to vertex_count.
template <AdjacencyFn F> void dijkstra_one_to_all(F adjacent_edges, VertexId source, std::vector<Distance> &dist) {
    std::fill(dist.begin(), dist.end(), kUnreachable);
    dist[source] = 0;
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
    pq.push({0, source});
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != dist[top.v]) {
            continue;
        }
        for (const Edge &e : adjacent_edges(top.v)) {
            const Distance nd = top.key + e.weight;
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                pq.push({nd, e.to});
            }
        }
    }
}

// Convenience overload for Graph. out is resized to graph.vertex_count() by this function.
void dijkstra_one_to_all(const Graph &graph, VertexId source, std::vector<Distance> &out);

} // namespace transport
