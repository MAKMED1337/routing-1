#include "graph/geometry.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace transport {

double haversine_meters(const NodeCoord &a, const NodeCoord &b) {
    constexpr double kEarthRadiusM = 6371008.8;
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    const double p1 = a.lat * kDegToRad;
    const double p2 = b.lat * kDegToRad;
    const double dlat = (b.lat - a.lat) * kDegToRad;
    const double dlon = (b.lon - a.lon) * kDegToRad;

    const double haversine = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                             std::cos(p1) * std::cos(p2) * std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
    const double c = 2.0 * std::atan2(std::sqrt(haversine), std::sqrt(1.0 - haversine));
    return kEarthRadiusM * c;
}

void require_matching_coords(std::span<const NodeCoord> coords, VertexId vertex_count, std::string_view context) {
    if (coords.size() != static_cast<size_t>(vertex_count)) {
        throw std::invalid_argument(std::string(context) + " requires coordinates matching the graph vertices");
    }
}

} // namespace transport
