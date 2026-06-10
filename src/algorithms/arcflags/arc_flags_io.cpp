#include "algorithms/arcflags/arc_flags_io.hpp"

#include "algorithms/partition.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace transport::arcflags {
namespace {

constexpr uint32_t kArcFlagsMagic = 0x31464154U; // TAF1, little-endian
constexpr uint32_t kArcFlagsVersion = 1;

template <typename T> void write_one(std::ofstream &out, const T &value) {
    static_assert(std::is_trivially_copyable_v<T>);
    const auto bytes = std::bit_cast<std::array<char, sizeof(T)>>(value);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

template <typename T> bool read_one(std::ifstream &in, T &value) {
    static_assert(std::is_trivially_copyable_v<T>);
    std::array<char, sizeof(T)> bytes{};
    in.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    value = std::bit_cast<T>(bytes);
    return static_cast<bool>(in);
}

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

bool save_arc_flags(const ArcFlagsPreprocessedData &data, const std::string &path) {
    validate_data(data);

    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }

    write_one(out, kArcFlagsMagic);
    write_one(out, kArcFlagsVersion);
    write_one(out, static_cast<uint64_t>(data.region_of.size()));
    write_one(out, static_cast<uint64_t>(data.forward_flags.size()));
    write_one(out, static_cast<uint32_t>(data.regions));
    write_one(out, stored_partition_method(data.partition_method));
    out.write(reinterpret_cast<const char *>(data.region_of.data()),
              static_cast<std::streamsize>(data.region_of.size() * sizeof(uint16_t)));
    out.write(reinterpret_cast<const char *>(data.forward_flags.data()),
              static_cast<std::streamsize>(data.forward_flags.size() * sizeof(uint64_t)));
    return static_cast<bool>(out);
}

ArcFlagsPreprocessedData load_arc_flags(const std::string &path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("arcflags_io: failed to open file: " + path);
    }

    uint32_t magic = 0;
    uint32_t version = 0;
    uint64_t vertices = 0;
    uint64_t edges = 0;
    uint32_t regions = 0;
    uint32_t partition_method = 0;
    if (!read_one(in, magic) || !read_one(in, version) || !read_one(in, vertices) || !read_one(in, edges) ||
        !read_one(in, regions) || !read_one(in, partition_method)) {
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
    in.read(reinterpret_cast<char *>(data.region_of.data()),
            static_cast<std::streamsize>(data.region_of.size() * sizeof(uint16_t)));
    if (!in) {
        throw std::runtime_error("arcflags_io: truncated region array");
    }
    in.read(reinterpret_cast<char *>(data.forward_flags.data()),
            static_cast<std::streamsize>(data.forward_flags.size() * sizeof(uint64_t)));
    if (!in) {
        throw std::runtime_error("arcflags_io: truncated flag array");
    }

    validate_data(data);
    return data;
}

} // namespace transport::arcflags
