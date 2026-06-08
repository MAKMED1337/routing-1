#include "algorithms/bidirectional_astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace transport {
namespace {

struct SignedHeapNode {
    int64_t key = 0;
    VertexId v = 0;

    bool operator>(const SignedHeapNode &other) const { return key > other.key; }
};

[[nodiscard]] int64_t heuristic_units(const Graph &graph, VertexId from, VertexId to) {
    return static_cast<int64_t>(
        std::floor(haversine_meters(graph.coords[from], graph.coords[to]) * static_cast<double>(kDistanceScale)));
}

void update_best(Distance candidate, Distance opposite, Distance &best) {
    if (opposite < kUnreachable) {
        best = std::min(best, candidate + opposite);
    }
}

} // namespace

BidirectionalAStarAlgorithm::BidirectionalAStarAlgorithm(const Graph &graph)
    : graph_(graph), forward_dist_(graph.vertex_count(), kUnreachable),
      backward_dist_(graph.vertex_count(), kUnreachable) {}

std::string_view BidirectionalAStarAlgorithm::name() const { return "bidi_astar"; }

void BidirectionalAStarAlgorithm::preprocess() { reverse_ = build_reverse_graph(graph_); }

PathResult BidirectionalAStarAlgorithm::query(VertexId source, VertexId target) const {
    if (reverse_.offsets.empty()) {
        throw std::logic_error("BidirectionalAStarAlgorithm::preprocess() must be called before query()");
    }

    if (source == target) {
        return PathResult{0, 0};
    }

    forward_dist_.reset();
    backward_dist_.reset();

    auto forward_potential = [this, source, target](VertexId vertex) -> int64_t {
        const int64_t to_target = heuristic_units(graph_, vertex, target);
        const int64_t from_source = heuristic_units(graph_, source, vertex);
        return (to_target - from_source) / 2;
    };
    auto backward_potential = [&forward_potential](VertexId vertex) -> int64_t { return -forward_potential(vertex); };

    auto forward_key = [&forward_potential](VertexId vertex, Distance distance) -> int64_t {
        return static_cast<int64_t>(distance) + forward_potential(vertex);
    };
    auto backward_key = [&backward_potential](VertexId vertex, Distance distance) -> int64_t {
        return static_cast<int64_t>(distance) + backward_potential(vertex);
    };

    std::priority_queue<SignedHeapNode, std::vector<SignedHeapNode>, std::greater<>> forward_pq;
    std::priority_queue<SignedHeapNode, std::vector<SignedHeapNode>, std::greater<>> backward_pq;

    forward_dist_.set(source, 0);
    backward_dist_.set(target, 0);
    forward_pq.push({forward_key(source, 0), source});
    backward_pq.push({backward_key(target, 0), target});

    constexpr int64_t kEmptyQueueKey = std::numeric_limits<int64_t>::max() / 4;

    Distance best = kUnreachable;
    uint32_t settled = 0;

    while (!forward_pq.empty() && !backward_pq.empty()) {
        const int64_t forward_min = forward_pq.empty() ? kEmptyQueueKey : forward_pq.top().key;
        const int64_t backward_min = backward_pq.empty() ? kEmptyQueueKey : backward_pq.top().key;

        if (best < kUnreachable && forward_min + backward_min >= static_cast<int64_t>(best)) {
            break;
        }

        if (forward_min <= backward_min) {
            const SignedHeapNode top = forward_pq.top();
            forward_pq.pop();

            const Distance current = forward_dist_.get(top.v);
            if (top.key != forward_key(top.v, current)) {
                continue;
            }

            ++settled;
            update_best(current, backward_dist_.get(top.v), best);

            for (const Edge &edge : graph_.adjacent_edges(top.v)) {
                const Distance next = current + edge.weight;
                if (next < forward_dist_.get(edge.to)) {
                    forward_dist_.set(edge.to, next);
                    forward_pq.push({forward_key(edge.to, next), edge.to});
                    update_best(next, backward_dist_.get(edge.to), best);
                }
            }
        } else {
            const SignedHeapNode top = backward_pq.top();
            backward_pq.pop();

            const Distance current = backward_dist_.get(top.v);
            if (top.key != backward_key(top.v, current)) {
                continue;
            }

            ++settled;
            update_best(current, forward_dist_.get(top.v), best);

            for (const Edge &edge : reverse_.adjacent_edges(top.v)) {
                const Distance next = current + edge.weight;
                if (next < backward_dist_.get(edge.to)) {
                    backward_dist_.set(edge.to, next);
                    backward_pq.push({backward_key(edge.to, next), edge.to});
                    update_best(next, forward_dist_.get(edge.to), best);
                }
            }
        }
    }

    return PathResult{best, settled};
}

} // namespace transport
