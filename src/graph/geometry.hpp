#pragma once

#include "graph/types.hpp"

#include <span>
#include <string_view>

namespace transport {

struct NodeCoord {
    double lat = 0.0;
    double lon = 0.0;
};

double haversine_meters(const NodeCoord &a, const NodeCoord &b);

// Throws std::invalid_argument with a message naming `context` if coords.size() != vertex_count.
void require_matching_coords(std::span<const NodeCoord> coords, VertexId vertex_count, std::string_view context);

} // namespace transport
