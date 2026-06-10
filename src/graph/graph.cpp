#include "graph/graph.hpp"

namespace transport {

VertexId Graph::vertex_count() const { return vertex_count_; }

uint64_t Graph::edge_count() const { return static_cast<uint64_t>(edges.size()); }

std::span<const Edge> Graph::adjacent_edges(VertexId vertex) const {
    const size_t begin = offsets[vertex];
    const size_t end = offsets[static_cast<size_t>(vertex) + 1];
    return std::span<const Edge>(edges.data() + begin, end - begin);
}

} // namespace transport
