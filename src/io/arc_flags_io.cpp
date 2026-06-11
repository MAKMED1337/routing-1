#include "io/arc_flags_io.hpp"

#include "algorithms/partition.hpp"
#include "io/binary_io.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace transport::arcflags {
namespace {

constexpr uint32_t kArcFlagsMagic = 0x31464154U; // TAF1, little-endian
constexpr uint32_t kArcFlagsVersion = 1;

uint32_t stored_partition_method(PartitionMethod method) {
    switch (method) {
    case PartitionMethod::Grid:
        return 0;
    case PartitionMethod::Inertial:
        return 1;
    case PartitionMethod::KaMinPar:
        return 2;
    }
    throw std::invalid_argument("arcflags_io: unsupported partition method");
}

PartitionMethod loaded_partition_method(uint32_t method) {
    switch (method) {
    case 0:
        return PartitionMethod::Grid;
    case 1:
        return PartitionMethod::Inertial;
    case 2:
        return PartitionMethod::KaMinPar;
    default:
        throw std::runtime_error("arcflags_io: unsupported partition method");
    }
}

size_t memory_size(uint64_t value) {
    if constexpr (std::numeric_limits<uint64_t>::max() > std::numeric_limits<size_t>::max()) {
        if (value > std::numeric_limits<size_t>::max()) {
            throw std::runtime_error("arcflags_io: count exceeds addressable size");
        }
    }
    return static_cast<size_t>(value);
}

void validate_data(const ArcFlagsPreprocessedData &data) {
    if (data.regions == 0 || data.regions > 64) {
        throw std::runtime_error("arcflags_io: regions must be in [1, 64]");
    }
    if (data.region_of.empty()) {
        throw std::runtime_error("arcflags_io: region array must not be empty");
    }
    for (const uint16_t region : data.region_of) {
        if (region >= data.regions) {
            throw std::runtime_error("arcflags_io: region id out of range");
        }
    }
}

} // namespace

bool save_arc_flags(const ArcFlagsPreprocessedData &data, const std::filesystem::path &path) {
    validate_data(data);

    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }

    io::write_one(out, kArcFlagsMagic);
    io::write_one(out, kArcFlagsVersion);
    io::write_one(out, static_cast<uint64_t>(data.region_of.size()));
    io::write_one(out, static_cast<uint64_t>(data.forward_flags.size()));
    io::write_one(out, static_cast<uint32_t>(data.regions));
    io::write_one(out, stored_partition_method(data.partition_method));
    io::write_span(out, std::span<const uint16_t>(data.region_of));
    io::write_span(out, std::span<const uint64_t>(data.forward_flags));
    return static_cast<bool>(out);
}

ArcFlagsPreprocessedData load_arc_flags(const std::filesystem::path &path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("arcflags_io: failed to open file: " + path.string());
    }

    uint32_t magic = 0;
    uint32_t version = 0;
    uint64_t vertices = 0;
    uint64_t edges = 0;
    uint32_t regions = 0;
    uint32_t partition_method = 0;
    if (!io::read_one(in, magic) || !io::read_one(in, version) || !io::read_one(in, vertices) ||
        !io::read_one(in, edges) || !io::read_one(in, regions) || !io::read_one(in, partition_method)) {
        throw std::runtime_error("arcflags_io: truncated header");
    }
    if (magic != kArcFlagsMagic) {
        throw std::runtime_error("arcflags_io: invalid magic");
    }
    if (version != kArcFlagsVersion) {
        throw std::runtime_error("arcflags_io: unsupported version");
    }

    ArcFlagsPreprocessedData data;
    data.regions = static_cast<uint16_t>(regions);
    data.partition_method = loaded_partition_method(partition_method);
    data.region_of.resize(memory_size(vertices));
    data.forward_flags.resize(memory_size(edges));
    io::read_span(in, std::span<uint16_t>(data.region_of), "arcflags_io: truncated region array");
    io::read_span(in, std::span<uint64_t>(data.forward_flags), "arcflags_io: truncated flag array");

    validate_data(data);
    return data;
}

} // namespace transport::arcflags
