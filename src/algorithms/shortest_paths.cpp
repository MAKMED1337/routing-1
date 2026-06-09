#include "algorithms/shortest_paths.hpp"

#include "algorithms/heap_node.hpp"
#include "routing/routing.hpp"

#include <algorithm>
#include <functional>
#include <queue>
#include <span>
#include <vector>

namespace transport {

void dijkstra_one_to_all(std::span<const uint64_t> offsets, std::span<const Edge> edges, VertexId source,
                         std::vector<Distance> &dist) {
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
        const size_t begin = static_cast<size_t>(offsets[top.v]);
        const size_t end = static_cast<size_t>(offsets[top.v + 1]);
        for (size_t i = begin; i < end; ++i) {
            const Edge &e = edges[i];
            const Distance nd = top.key + e.weight;
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                pq.push({nd, e.to});
            }
        }
    }
}

void dijkstra_one_to_all(const Graph &graph, VertexId source, std::vector<Distance> &out) {
    out.resize(graph.vertex_count());
    if (source >= graph.vertex_count()) {
        std::fill(out.begin(), out.end(), kUnreachable);
        return;
    }
    dijkstra_one_to_all(graph.offsets, graph.edges, source, out);
}

} // namespace transport
