#include "algorithms/phast.hpp"

#include "algorithms/heap_node.hpp"
#include "routing/routing.hpp"

#include <algorithm>
#include <functional>
#include <queue>
#include <vector>

namespace transport {

namespace {

template <typename AdjacentEdges>
void upward_dijkstra(AdjacentEdges adjacent_edges, VertexId source, std::vector<Distance> &dist) {
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

} // namespace

std::vector<VertexId> build_inv_rank(const ContractionHierarchy &ch) {
    const VertexId V = ch.vertex_count();
    std::vector<VertexId> inv(V);
    for (VertexId v = 0; v < V; ++v) {
        inv[ch.rank[v]] = v;
    }
    return inv;
}

void phast_all_to_one(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId target,
                      std::vector<Distance> &dist) {
    const VertexId V = ch.vertex_count();
    std::fill(dist.begin(), dist.end(), kUnreachable);
    dist[target] = 0;

    // Phase 1: backward upward search from target gives d(v, target) for high-rank v.
    upward_dijkstra([&ch](VertexId v) { return ch.backward_adjacent_edges(v); }, target, dist);

    // Phase 2: downward sweep using forward arcs (descending rank order).
    // forward_adjacent_edges(v) → arcs v→u where rank[u] > rank[v].
    // d(v, target) ≤ w(v,u) + d(u, target) for each such arc.
    for (VertexId r = V; r-- > 0;) {
        const VertexId v = inv_rank[r];
        for (const Edge &e : ch.forward_adjacent_edges(v)) {
            if (dist[e.to] == kUnreachable) {
                continue;
            }
            dist[v] = std::min(dist[v], e.weight + dist[e.to]);
        }
    }
}

void phast_one_to_all(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId source,
                      std::vector<Distance> &dist) {
    const VertexId V = ch.vertex_count();
    std::fill(dist.begin(), dist.end(), kUnreachable);
    dist[source] = 0;

    // Phase 1: forward upward search from source gives d(source, u) for high-rank u.
    upward_dijkstra([&ch](VertexId v) { return ch.forward_adjacent_edges(v); }, source, dist);

    // Phase 2: downward sweep using backward arcs (descending rank order).
    // backward_adjacent_edges(v) stores arc v→u where rank[u] > rank[v], representing original arc u→v.
    // d(source, v) ≤ d(source, u) + w(u,v); rank[u] > rank[v] so u is settled before v.
    for (VertexId r = V; r-- > 0;) {
        const VertexId v = inv_rank[r];
        for (const Edge &e : ch.backward_adjacent_edges(v)) {
            if (dist[e.to] == kUnreachable) {
                continue;
            }
            dist[v] = std::min(dist[v], e.weight + dist[e.to]);
        }
    }
}

} // namespace transport
