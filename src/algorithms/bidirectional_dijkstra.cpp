#include "algorithms/bidirectional_dijkstra.hpp"

#include "algorithms/heap_node.hpp"

#include <algorithm>
#include <stdexcept>
#include <string_view>

namespace transport {

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

    HeapQueue forward_pq;
    HeapQueue backward_pq;

    forward_dist_.set(source, 0);
    backward_dist_.set(target, 0);
    forward_pq.push({0, source});
    backward_pq.push({0, target});

    Distance best = source == target ? 0 : kUnreachable;
    QueryStats stats;

    auto update_best = [&best](Distance candidate, Distance opposite) {
        if (opposite < kUnreachable) {
            best = std::min(best, candidate + opposite);
        }
    };

    auto settle_next = [&stats, &update_best](HeapQueue &pq, const Graph &search_graph, StampedVector<Distance> &dist,
                                              const StampedVector<Distance> &opposite_dist, bool is_forward) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != dist.get(top.v)) {
            return;
        }

        ++stats.settled;
        if (is_forward) {
            ++stats.settled_forward;
        } else {
            ++stats.settled_backward;
        }
        update_best(top.key, opposite_dist.get(top.v));

        for (const Edge &edge : search_graph.adjacent_edges(top.v)) {
            ++stats.relaxed_arcs;
            const Distance next = top.key + edge.weight;
            if (next < dist.get(edge.to)) {
                dist.set(edge.to, next);
                pq.push({next, edge.to});
                ++stats.heap_pushes;
                update_best(next, opposite_dist.get(edge.to));
            }
        }
    };

    while (!forward_pq.empty() && !backward_pq.empty()) {
        const Distance forward_min = forward_pq.top().key;
        const Distance backward_min = backward_pq.top().key;
        if (forward_min + backward_min >= best) {
            break;
        }

        if (forward_min <= backward_min) {
            settle_next(forward_pq, graph_, forward_dist_, backward_dist_, true);
        } else {
            settle_next(backward_pq, reverse_, backward_dist_, forward_dist_, false);
        }
    }

    return PathResult{best, stats};
}

} // namespace transport
