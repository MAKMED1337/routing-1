#include "algorithms/shortest_paths.hpp"

#include "algorithms/heap_node.hpp"
#include "routing/routing.hpp"

#include <algorithm>
#include <functional>
#include <queue>
#include <vector>

namespace transport {

void dijkstra_one_to_all(const Graph &graph, VertexId source, std::vector<Distance> &out) {
    std::fill(out.begin(), out.end(), kUnreachable);
    if (source >= graph.vertex_count()) {
        return;
    }

    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
    out[source] = 0;
    pq.push({0, source});
    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != out[top.v]) {
            continue;
        }

        for (const Edge &edge : graph.adjacent_edges(top.v)) {
            const Distance next_distance = top.key + edge.weight;
            if (next_distance < out[edge.to]) {
                out[edge.to] = next_distance;
                pq.push({next_distance, edge.to});
            }
        }
    }
}

} // namespace transport
