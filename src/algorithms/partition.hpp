#pragma once

#include "graph/graph.hpp"

#include <cstdint>
#include <string_view>
#include <vector>

namespace transport {

enum class PartitionMethod { Grid, Inertial, KaMinPar };

[[nodiscard]] PartitionMethod parse_partition_method(std::string_view method);
[[nodiscard]] std::string_view partition_method_name(PartitionMethod method);

// Assigns a region id in [0, regions) to each vertex using the requested method.
//   "grid"     — divide lat/lon bounding box into a sqrt(regions) × sqrt(regions) uniform grid
//   "inertial" — binary kd-split alternating lon/lat, log2(regions) levels (regions must be a power of 2)
//   "kaminpar" — k-way partitioning via KaMinPar (must be compiled with TRANSPORT_HAVE_KAMINPAR)
// Returns a vector of length vertex_count.
[[nodiscard]] std::vector<uint16_t> build_partition(const Graph &graph, uint16_t regions, PartitionMethod method);

} // namespace transport
