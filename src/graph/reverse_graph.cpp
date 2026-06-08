#include "graph/reverse_graph.hpp"

#include <cstddef>
#include <vector>

namespace transport {

Graph build_reverse_graph(const Graph &graph) {
    const VertexId vertices = graph.vertex_count();

    Graph reverse;
    reverse.coords = graph.coords;
    reverse.offsets.assign(static_cast<size_t>(vertices) + 1, 0);
    reverse.edges.resize(graph.edges.size());

    for (const Edge &edge : graph.edges) {
        ++reverse.offsets[static_cast<size_t>(edge.to) + 1];
    }
    for (size_t i = 1; i < reverse.offsets.size(); ++i) {
        reverse.offsets[i] += reverse.offsets[i - 1];
    }

    std::vector<uint64_t> next = reverse.offsets;
    for (VertexId from = 0; from < vertices; ++from) {
        for (const Edge &edge : graph.adjacent_edges(from)) {
            const uint64_t slot = next[edge.to]++;
            reverse.edges[static_cast<size_t>(slot)] = Edge{.to = from, .weight = edge.weight};
        }
    }

    return reverse;
}

} // namespace transport
