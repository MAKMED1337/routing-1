#include "graph/reverse_graph.hpp"

#include <cstddef>
#include <vector>

namespace transport {

ReverseGraph build_reverse_graph(const Graph &graph) {
    const VertexId vertices = graph.vertex_count();

    ReverseGraph reverse;
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
        const uint64_t begin = graph.offsets[from];
        const uint64_t end = graph.offsets[static_cast<size_t>(from) + 1];
        for (uint64_t i = begin; i < end; ++i) {
            const Edge &edge = graph.edges[static_cast<size_t>(i)];
            const uint64_t slot = next[edge.to]++;
            reverse.edges[static_cast<size_t>(slot)] = Edge{.to = from, .weight = edge.weight};
        }
    }

    return reverse;
}

} // namespace transport
