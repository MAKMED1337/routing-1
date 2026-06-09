#include "algorithms/phast.hpp"

#include "algorithms/shortest_paths.hpp"

#include <algorithm>
#include <vector>

namespace transport {

namespace {

template <AdjacencyFn F>
void downward_sweep(F adjacent_edges, const std::vector<VertexId> &inv_rank, std::vector<Distance> &dist) {
    for (VertexId r = static_cast<VertexId>(inv_rank.size()); r-- > 0;) {
        const VertexId v = inv_rank[r];
        for (const Edge &e : adjacent_edges(v)) {
            if (dist[e.to] != kUnreachable) {
                dist[v] = std::min(dist[v], e.weight + dist[e.to]);
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
    // Phase 1: backward upward search from target gives d(v, target) for high-rank v.
    dijkstra_one_to_all([&ch](VertexId v) { return ch.backward_adjacent_edges(v); }, target, dist);
    // Phase 2: downward sweep propagates through forward arcs to lower-rank vertices.
    // forward_adjacent_edges(v) → arcs v→u where rank[u] > rank[v].
    // d(v, target) ≤ w(v,u) + d(u, target) for each such arc.
    downward_sweep([&ch](VertexId v) { return ch.forward_adjacent_edges(v); }, inv_rank, dist);
}

void phast_one_to_all(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId source,
                      std::vector<Distance> &dist) {
    // Phase 1: forward upward search from source gives d(source, u) for high-rank u.
    dijkstra_one_to_all([&ch](VertexId v) { return ch.forward_adjacent_edges(v); }, source, dist);
    // Phase 2: downward sweep propagates through backward arcs to lower-rank vertices.
    // backward_adjacent_edges(v) stores arc v→u where rank[u] > rank[v], representing original arc u→v.
    // d(source, v) ≤ d(source, u) + w(u,v); rank[u] > rank[v] so u is settled before v.
    downward_sweep([&ch](VertexId v) { return ch.backward_adjacent_edges(v); }, inv_rank, dist);
}

} // namespace transport
