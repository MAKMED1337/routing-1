#include "algorithms/phast.hpp"

#include "algorithms/shortest_paths.hpp"
#include "routing/routing.hpp"

#include <algorithm>
#include <vector>

namespace transport {

namespace {

// Shared helper for both single-target context methods.
// Runs an upward Dijkstra from start_rank using up_adj, then a rank-space downward sweep
// using sweep_adj, and scatters the result into dist (original vertex order, resized to V).
template <AdjacencyFn Up, AdjacencyFn Sweep>
void phast_single_target(const PhastAlgorithm &ctx, VertexId start_rank, Up up_adj, Sweep sweep_adj,
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

// ---------- PhastAlgorithm ----------

PhastAlgorithm::PhastAlgorithm(const ContractionHierarchy &ch) {
    const VertexId V = ch.vertex_count();
    rank_to_vertex.resize(V);
    vertex_to_rank.resize(V);

    for (VertexId v = 0; v < V; ++v) {
        const VertexId r = static_cast<VertexId>(ch.rank[v]);
        rank_to_vertex[r] = v;
        vertex_to_rank[v] = r;
    }

    auto build_csr = [&](auto adj_fn, std::vector<size_t> &offsets, std::vector<Edge> &edges) {
        offsets.resize(V + 1, 0);
        for (VertexId r = 0; r < V; ++r) {
            offsets[r + 1] = offsets[r] + adj_fn(rank_to_vertex[r]).size();
        }
        edges.reserve(offsets[V]);
        for (VertexId r = 0; r < V; ++r) {
            for (const Edge &e : adj_fn(rank_to_vertex[r])) {
                edges.push_back({vertex_to_rank[e.to], e.weight});
            }
        }
    };

    build_csr([&ch](VertexId v) { return ch.forward_adjacent_edges(v); }, fwd_offsets, fwd_edges);
    build_csr([&ch](VertexId v) { return ch.backward_adjacent_edges(v); }, bwd_offsets, bwd_edges);
}

VertexId PhastAlgorithm::vertex_count() const { return static_cast<VertexId>(rank_to_vertex.size()); }

std::span<const Edge> PhastAlgorithm::fwd_adjacent_edges(VertexId rank) const {
    return {fwd_edges.data() + fwd_offsets[rank], fwd_offsets[rank + 1] - fwd_offsets[rank]};
}

std::span<const Edge> PhastAlgorithm::bwd_adjacent_edges(VertexId rank) const {
    return {bwd_edges.data() + bwd_offsets[rank], bwd_offsets[rank + 1] - bwd_offsets[rank]};
}

void PhastAlgorithm::all_to_one(VertexId target, std::vector<Distance> &dist) const {
    phast_single_target(
        *this, vertex_to_rank[target], [this](VertexId r) { return bwd_adjacent_edges(r); },
        [this](VertexId r) { return fwd_adjacent_edges(r); }, dist);
}

void PhastAlgorithm::one_to_all(VertexId source, std::vector<Distance> &dist) const {
    phast_single_target(
        *this, vertex_to_rank[source], [this](VertexId r) { return fwd_adjacent_edges(r); },
        [this](VertexId r) { return bwd_adjacent_edges(r); }, dist);
}

void PhastAlgorithm::all_to_one_batch(std::span<const VertexId> targets, std::vector<Distance> &dist) const {
    const VertexId V = vertex_count();
    const size_t B = targets.size();
    if (B == 0) {
        dist.clear();
        return;
    }

    // dist_rank layout: rank-major, lane-minor — dist_rank[r * B + lane].
    // thread_local so the allocation is reused across batches on the same thread,
    // avoiding repeated 500+ MB malloc/free under multi-threaded preprocessing.
    thread_local std::vector<Distance> dist_rank;
    dist_rank.assign(V * B, kUnreachable);

    // Phase 1: B independent backward upward Dijkstras, one per target lane.
    for (size_t lane = 0; lane < B; ++lane) {
        const VertexId tr = vertex_to_rank[targets[lane]];
        dist_rank[tr * B + lane] = 0;
        HeapQueue pq;
        pq.push({0, tr});
        while (!pq.empty()) {
            const HeapNode top = pq.top();
            pq.pop();
            if (top.key != dist_rank[top.v * B + lane]) {
                continue;
            }
            for (const Edge &e : bwd_adjacent_edges(top.v)) {
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
        for (const Edge &e : fwd_adjacent_edges(r)) {
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
        const VertexId v = rank_to_vertex[r];
        const Distance *src = &dist_rank[r * B];
        Distance *dst = &dist[v * B];
        for (size_t lane = 0; lane < B; ++lane) {
            dst[lane] = src[lane];
        }
    }
}

} // namespace transport
