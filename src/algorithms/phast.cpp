#include "algorithms/phast.hpp"

#include "algorithms/shortest_paths.hpp"
#include "routing/routing.hpp"

#include <algorithm>
#include <concepts>
#include <ranges>
#include <vector>

namespace transport {

namespace {

template <typename F>
concept AdjacencyFn = std::invocable<F, VertexId> && std::ranges::range<std::invoke_result_t<F, VertexId>>;

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
    dijkstra_one_to_all(ch.backward_offsets, ch.backward_edges, target, dist);
    // Phase 2: downward sweep using forward arcs propagates to lower-rank vertices.
    // forward_adjacent_edges(v) → arcs v→u where rank[u] > rank[v].
    // d(v, target) ≤ w(v,u) + d(u, target) for each such arc.
    downward_sweep([&ch](VertexId v) { return ch.forward_adjacent_edges(v); }, inv_rank, dist);
}

void phast_one_to_all(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId source,
                      std::vector<Distance> &dist) {
    // Phase 1: forward upward search from source gives d(source, u) for high-rank u.
    dijkstra_one_to_all(ch.forward_offsets, ch.forward_edges, source, dist);
    // Phase 2: downward sweep using backward arcs propagates to lower-rank vertices.
    // backward_adjacent_edges(v) stores arc v→u where rank[u] > rank[v], representing original arc u→v.
    // d(source, v) ≤ d(source, u) + w(u,v); rank[u] > rank[v] so u is settled before v.
    downward_sweep([&ch](VertexId v) { return ch.backward_adjacent_edges(v); }, inv_rank, dist);
}

} // namespace transport
