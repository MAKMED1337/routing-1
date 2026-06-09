#include "algorithms/region_flags.hpp"

#include "algorithms/partition.hpp"

#include <bit>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace transport {

void validate_region_count(uint32_t regions, std::string_view prefix) {
    if (regions == 0 || regions > 64) {
        throw std::invalid_argument(std::string(prefix) + ": regions must be in [1, 64]");
    }
}

void validate_inertial_regions(uint32_t regions, std::string_view prefix) {
    if (regions == 0 || !std::has_single_bit(regions)) {
        throw std::invalid_argument(std::string(prefix) + ": inertial partition requires regions to be a power of two");
    }
}

std::vector<uint16_t> make_region_assignment(const Graph &graph, uint32_t regions, std::string_view method) {
    return build_partition(graph, static_cast<uint16_t>(regions), parse_partition_method(method));
}

} // namespace transport
