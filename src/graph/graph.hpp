#pragma once

#include "graph/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace transport {

constexpr uint32_t kDistanceScale = 1000;

struct NodeCoord {
    double lat = 0.0;
    double lon = 0.0;
};

struct Edge {
    VertexId to = 0;
    Weight weight = 0;
};

class Graph {
public:
    VertexId vertex_count_ = 0;
    std::vector<size_t> offsets;
    std::vector<Edge> edges;

    [[nodiscard]] VertexId vertex_count() const;
    [[nodiscard]] uint64_t edge_count() const;
    [[nodiscard]] std::span<const Edge> adjacent_edges(VertexId vertex) const;
};

[[nodiscard]] bool save_graph_binary(const Graph &graph, const std::string &path);
[[nodiscard]] Graph load_graph_binary(const std::string &path);

// Coordinate sidecar: lat/lon for each vertex, stored separately from the graph topology.
// Coordinates are written in graph-id order and are only meaningful alongside the matching
// graph binary (same vertex count, same id assignment).
[[nodiscard]] bool save_coords_binary(std::span<const NodeCoord> coords, const std::string &path);
[[nodiscard]] std::vector<NodeCoord> load_coords_binary(const std::string &path);

double haversine_meters(const NodeCoord &a, const NodeCoord &b);

// Throws std::invalid_argument with a message naming `context` if coords.size() != vertex_count.
void require_matching_coords(std::span<const NodeCoord> coords, VertexId vertex_count, std::string_view context);

} // namespace transport
