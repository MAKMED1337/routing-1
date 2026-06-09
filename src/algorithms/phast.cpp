#include "algorithms/phast.hpp"

#include "algorithms/shortest_paths.hpp"
#include "routing/routing.hpp"

#include <algorithm>
#include <functional>
#include <queue>
#include <vector>

namespace transport {

namespace {

// Legacy rank-space sweep: processes vertices in descending rank order, relaxing via adjacent_edges.
// dist is indexed by original vertex id; e.to is also an original vertex id.
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

// Shared helper for both single-target context APIs.
// Runs an upward Dijkstra from start_rank in rank-space using up_adj, then a downward sweep
// using sweep_adj, and scatters the result into dist (original vertex order, resized to V).
template <AdjacencyFn Up, AdjacencyFn Sweep>
void phast_single_target(const PhastContext &ctx, VertexId start_rank, Up up_adj, Sweep sweep_adj,
                         std::vector<Distance> &dist) {
    const VertexId V = ctx.vertex_count();
    std::vector<Distance> dist_rank(V);
    dijkstra_one_to_all(up_adj, start_rank, dist_rank);
    for (VertexId r = V; r-- > 0;) {
        for (const Edge &e : sweep_adj(r)) {
            if (dist_rank[e.to] != kUnreachable) {
                dist_rank[r] = std::min(dist_rank[r], e.weight + dist_rank[e.to]);
            }
        }
    }
    dist.resize(V);
    for (VertexId r = 0; r < V; ++r) {
        dist[ctx.rank_to_vertex[r]] = dist_rank[r];
    }
}

} // namespace

// ---------- Legacy APIs ----------

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
    // Phase 2: downward sweep using forward arcs propagates to lower-rank vertices.
    // forward_adjacent_edges(v) → arcs v→u where rank[u] > rank[v].
    // d(v, target) ≤ w(v,u) + d(u, target) for each such arc.
    downward_sweep([&ch](VertexId v) { return ch.forward_adjacent_edges(v); }, inv_rank, dist);
}

void phast_one_to_all(const ContractionHierarchy &ch, const std::vector<VertexId> &inv_rank, VertexId source,
                      std::vector<Distance> &dist) {
    // Phase 1: forward upward search from source gives d(source, u) for high-rank u.
    dijkstra_one_to_all([&ch](VertexId v) { return ch.forward_adjacent_edges(v); }, source, dist);
    // Phase 2: downward sweep using backward arcs propagates to lower-rank vertices.
    // backward_adjacent_edges(v) stores arc v→u where rank[u] > rank[v], representing original arc u→v.
    // d(source, v) ≤ d(source, u) + w(u,v); rank[u] > rank[v] so u is settled before v.
    downward_sweep([&ch](VertexId v) { return ch.backward_adjacent_edges(v); }, inv_rank, dist);
}

// ---------- PhastContext ----------

PhastContext::PhastContext(const ContractionHierarchy &ch) {
    const VertexId V = ch.vertex_count();
    rank_to_vertex.resize(V);
    vertex_to_rank.resize(V);

    for (VertexId v = 0; v < V; ++v) {
        const VertexId r = static_cast<VertexId>(ch.rank[v]);
        rank_to_vertex[r] = v;
        vertex_to_rank[v] = r;
    }

    // Build rank-indexed forward CSR (edges .to = destination rank).
    fwd_offsets.resize(V + 1, 0);
    for (VertexId r = 0; r < V; ++r) {
        fwd_offsets[r + 1] =
            fwd_offsets[r] + static_cast<uint64_t>(ch.forward_adjacent_edges(rank_to_vertex[r]).size());
    }
    fwd_edges.reserve(static_cast<size_t>(fwd_offsets[V]));
    for (VertexId r = 0; r < V; ++r) {
        for (const Edge &e : ch.forward_adjacent_edges(rank_to_vertex[r])) {
            fwd_edges.push_back({vertex_to_rank[e.to], e.weight});
        }
    }

    // Build rank-indexed backward CSR (edges .to = destination rank).
    bwd_offsets.resize(V + 1, 0);
    for (VertexId r = 0; r < V; ++r) {
        bwd_offsets[r + 1] =
            bwd_offsets[r] + static_cast<uint64_t>(ch.backward_adjacent_edges(rank_to_vertex[r]).size());
    }
    bwd_edges.reserve(static_cast<size_t>(bwd_offsets[V]));
    for (VertexId r = 0; r < V; ++r) {
        for (const Edge &e : ch.backward_adjacent_edges(rank_to_vertex[r])) {
            bwd_edges.push_back({vertex_to_rank[e.to], e.weight});
        }
    }
}

VertexId PhastContext::vertex_count() const { return static_cast<VertexId>(rank_to_vertex.size()); }

std::span<const Edge> PhastContext::fwd_adjacent_edges(VertexId rank) const {
    const size_t begin = static_cast<size_t>(fwd_offsets[rank]);
    const size_t end = static_cast<size_t>(fwd_offsets[rank + 1]);
    return {fwd_edges.data() + begin, end - begin};
}

std::span<const Edge> PhastContext::bwd_adjacent_edges(VertexId rank) const {
    const size_t begin = static_cast<size_t>(bwd_offsets[rank]);
    const size_t end = static_cast<size_t>(bwd_offsets[rank + 1]);
    return {bwd_edges.data() + begin, end - begin};
}

// ---------- Optimized context APIs ----------

void phast_all_to_one(const PhastContext &ctx, VertexId target, std::vector<Distance> &dist) {
    phast_single_target(
        ctx, ctx.vertex_to_rank[target], [&ctx](VertexId r) { return ctx.bwd_adjacent_edges(r); },
        [&ctx](VertexId r) { return ctx.fwd_adjacent_edges(r); }, dist);
}

void phast_one_to_all(const PhastContext &ctx, VertexId source, std::vector<Distance> &dist) {
    phast_single_target(
        ctx, ctx.vertex_to_rank[source], [&ctx](VertexId r) { return ctx.fwd_adjacent_edges(r); },
        [&ctx](VertexId r) { return ctx.bwd_adjacent_edges(r); }, dist);
}

void phast_all_to_one_batch(const PhastContext &ctx, std::span<const VertexId> targets, std::vector<Distance> &dist) {
    const VertexId V = ctx.vertex_count();
    const size_t B = targets.size();

    // dist_rank layout: rank-major, lane-minor — dist_rank[r * B + lane].
    std::vector<Distance> dist_rank(V * B, kUnreachable);

    // Phase 1: B independent backward upward Dijkstras, one per target lane.
    for (size_t lane = 0; lane < B; ++lane) {
        const VertexId tr = ctx.vertex_to_rank[targets[lane]];
        dist_rank[tr * B + lane] = 0;
        std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> pq;
        pq.push({0, tr});
        while (!pq.empty()) {
            const HeapNode top = pq.top();
            pq.pop();
            if (top.key != dist_rank[top.v * B + lane]) {
                continue;
            }
            for (const Edge &e : ctx.bwd_adjacent_edges(top.v)) {
                const Distance nd = top.key + e.weight;
                Distance &slot = dist_rank[e.to * B + lane];
                if (nd < slot) {
                    slot = nd;
                    pq.push({nd, e.to});
                }
            }
        }
    }

    // Phase 2: One shared forward downward sweep for all lanes simultaneously.
    for (VertexId r = V; r-- > 0;) {
        Distance *dv = &dist_rank[r * B];
        for (const Edge &e : ctx.fwd_adjacent_edges(r)) {
            const Distance *du = &dist_rank[e.to * B];
            for (size_t lane = 0; lane < B; ++lane) {
                if (du[lane] != kUnreachable) {
                    dv[lane] = std::min(dv[lane], e.weight + du[lane]);
                }
            }
        }
    }

    // Scatter to output in original vertex order (vertex-major): dist[v * B + lane].
    dist.resize(V * B);
    for (VertexId r = 0; r < V; ++r) {
        const VertexId v = ctx.rank_to_vertex[r];
        const Distance *src = &dist_rank[r * B];
        Distance *dst = &dist[v * B];
        for (size_t lane = 0; lane < B; ++lane) {
            dst[lane] = src[lane];
        }
    }
}

} // namespace transport
