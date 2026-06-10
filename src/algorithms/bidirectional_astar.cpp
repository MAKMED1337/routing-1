#include "algorithms/bidirectional_astar.hpp"

#include "algorithms/heap_node.hpp"

#include <algorithm>
#include <stdexcept>
#include <string_view>

namespace transport {

BidirectionalAStarAlgorithm::BidirectionalAStarAlgorithm(const Graph &graph, Heuristic heuristic)
    : graph_(graph), heuristic_(std::move(heuristic)), forward_dist_(graph.vertex_count(), kUnreachable),
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

    auto forward_key = [this, target](VertexId vertex, Distance distance) -> Distance {
        return distance + heuristic_(vertex, target);
    };
    auto backward_key = [this, source](VertexId vertex, Distance distance) -> Distance {
        return distance + heuristic_(source, vertex);
    };

    HeapQueue forward_pq;
    HeapQueue backward_pq;

    forward_dist_.set(source, 0);
    backward_dist_.set(target, 0);
    forward_pq.push({forward_key(source, 0), source});
    backward_pq.push({backward_key(target, 0), target});

    Distance best = kUnreachable;
    uint32_t settled = 0;

    auto update_best = [&best](Distance candidate, Distance opposite) {
        if (opposite < kUnreachable) {
            best = std::min(best, candidate + opposite);
        }
    };

    auto settle_next = [&settled, &update_best](HeapQueue &pq, const Graph &search_graph, StampedVector<Distance> &dist,
                                                const StampedVector<Distance> &opposite_dist, const auto &key_for) {
        const HeapNode top = pq.top();
        pq.pop();

        const Distance current = dist.get(top.v);
        if (top.key != key_for(top.v, current)) {
            return;
        }

        ++settled;
        update_best(current, opposite_dist.get(top.v));

        for (const Edge &edge : search_graph.adjacent_edges(top.v)) {
            const Distance next = current + edge.weight;
            if (next < dist.get(edge.to)) {
                dist.set(edge.to, next);
                pq.push({key_for(edge.to, next), edge.to});
                update_best(next, opposite_dist.get(edge.to));
            }
        }
    };

    while (!forward_pq.empty() && !backward_pq.empty()) {
        const Distance forward_min = forward_pq.top().key;
        const Distance backward_min = backward_pq.top().key;
        if (best < kUnreachable && (forward_min >= best || backward_min >= best)) {
            break;
        }

        if (forward_min <= backward_min) {
            settle_next(forward_pq, graph_, forward_dist_, backward_dist_, forward_key);
        } else {
            settle_next(backward_pq, reverse_, backward_dist_, forward_dist_, backward_key);
        }
    }

    return PathResult{best, settled};
}

} // namespace transport
