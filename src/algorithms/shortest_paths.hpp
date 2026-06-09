#pragma once

#include "graph/graph.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace transport {

// Dijkstra from source over a CSR adjacency. dist is reset to kUnreachable, then Dijkstra runs.
// dist must be pre-sized to vertex_count.
void dijkstra_one_to_all(std::span<const uint64_t> offsets, std::span<const Edge> edges, VertexId source,
                         std::vector<Distance> &dist);

// Convenience overload for Graph. out is resized to graph.vertex_count() by this function.
void dijkstra_one_to_all(const Graph &graph, VertexId source, std::vector<Distance> &out);

} // namespace transport
