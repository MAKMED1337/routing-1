#pragma once

#include "graph/graph.hpp"

#include <cstdint>
#include <string_view>
#include <vector>

namespace transport {

// Throws std::invalid_argument if regions not in [1, 64].
void validate_region_count(uint32_t regions, std::string_view prefix);

// Throws std::invalid_argument if regions is not a power of two.
// Only applies when the inertial partition method is used.
void validate_inertial_regions(uint32_t regions, std::string_view prefix);

// Bitmask for a single region id (r must be in [0, 63]).
inline constexpr uint64_t region_bit(uint32_t r) noexcept { return 1ULL << r; }

// Parses method string and returns one region-id per vertex.
// Wraps parse_partition_method + build_partition so callers need not see the PartitionMethod enum.
[[nodiscard]] std::vector<uint16_t> make_region_assignment(const Graph &graph, uint32_t regions,
                                                           std::string_view method);

} // namespace transport
