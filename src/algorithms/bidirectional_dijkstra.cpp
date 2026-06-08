#include "algorithms/bidirectional_dijkstra.hpp"

#include "algorithms/heap_node.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace transport {
namespace {

Distance saturating_add(Distance lhs, Distance rhs) {
    if (lhs == kUnreachable || rhs == kUnreachable) {
        return kUnreachable;
    }
    if (kUnreachable - lhs < rhs) {
        return kUnreachable;
    }
    return lhs + rhs;
}

void update_best(Distance candidate, Distance opposite, Distance &best) {
    best = std::min(best, saturating_add(candidate, opposite));
}

} // namespace

BidirectionalDijkstraAlgorithm::BidirectionalDijkstraAlgorithm(const Graph &graph)
    : graph_(graph), forward_dist_(graph.vertex_count(), kUnreachable),
      backward_dist_(graph.vertex_count(), kUnreachable) {}

std::string_view BidirectionalDijkstraAlgorithm::name() const { return "bidijkstra"; }

void BidirectionalDijkstraAlgorithm::preprocess() { reverse_ = build_reverse_graph(graph_); }

PathResult BidirectionalDijkstraAlgorithm::query(VertexId source, VertexId target) const {
    if (reverse_.offsets.empty()) {
        throw std::logic_error("BidirectionalDijkstraAlgorithm::preprocess() must be called before query()");
    }

    forward_dist_.reset();
    backward_dist_.reset();

    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> forward_pq;
    std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<>> backward_pq;

    forward_dist_.set(source, 0);
    backward_dist_.set(target, 0);
    forward_pq.push({0, source});
    backward_pq.push({0, target});

    Distance best = source == target ? 0 : kUnreachable;
    uint32_t settled = 0;

    while (!forward_pq.empty() || !backward_pq.empty()) {
        const Distance forward_min = forward_pq.empty() ? kUnreachable : forward_pq.top().key;
        const Distance backward_min = backward_pq.empty() ? kUnreachable : backward_pq.top().key;
        if (saturating_add(forward_min, backward_min) >= best) {
            break;
        }

        if (forward_min <= backward_min) {
            const HeapNode top = forward_pq.top();
            forward_pq.pop();
            if (top.key != forward_dist_.get(top.v)) {
                continue;
            }

            ++settled;
            update_best(top.key, backward_dist_.get(top.v), best);

            const uint64_t begin = graph_.offsets[top.v];
            const uint64_t end = graph_.offsets[static_cast<size_t>(top.v) + 1];
            for (uint64_t i = begin; i < end; ++i) {
                const Edge &edge = graph_.edges[static_cast<size_t>(i)];
                const Distance next = top.key + edge.weight;
                if (next < forward_dist_.get(edge.to)) {
                    forward_dist_.set(edge.to, next);
                    forward_pq.push({next, edge.to});
                    update_best(next, backward_dist_.get(edge.to), best);
                }
            }
        } else {
            const HeapNode top = backward_pq.top();
            backward_pq.pop();
            if (top.key != backward_dist_.get(top.v)) {
                continue;
            }

            ++settled;
            update_best(top.key, forward_dist_.get(top.v), best);

            const uint64_t begin = reverse_.offsets[top.v];
            const uint64_t end = reverse_.offsets[static_cast<size_t>(top.v) + 1];
            for (uint64_t i = begin; i < end; ++i) {
                const Edge &edge = reverse_.edges[static_cast<size_t>(i)];
                const Distance next = top.key + edge.weight;
                if (next < backward_dist_.get(edge.to)) {
                    backward_dist_.set(edge.to, next);
                    backward_pq.push({next, edge.to});
                    update_best(next, forward_dist_.get(edge.to), best);
                }
            }
        }
    }

    return PathResult{best, settled};
}

} // namespace transport
