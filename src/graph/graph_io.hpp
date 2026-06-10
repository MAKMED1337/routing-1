#pragma once

#include "graph/geometry.hpp"
#include "graph/graph.hpp"

#include <span>
#include <string>
#include <vector>

namespace transport {

[[nodiscard]] bool save_graph_binary(const Graph &graph, const std::string &path);
[[nodiscard]] Graph load_graph_binary(const std::string &path);

// Coordinate sidecar: lat/lon for each vertex, stored separately from the graph topology.
// Coordinates are written in graph-id order and are only meaningful alongside the matching
// graph binary (same vertex count, same id assignment).
[[nodiscard]] bool save_coords_binary(std::span<const NodeCoord> coords, const std::string &path);
[[nodiscard]] std::vector<NodeCoord> load_coords_binary(const std::string &path);

} // namespace transport
