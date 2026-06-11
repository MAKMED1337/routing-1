#include "io/ch_io.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "io/binary_io.hpp"

namespace transport::ch {
namespace {

constexpr uint32_t kChMagic = 0x48435254U; // TRCH, little-endian
constexpr uint32_t kChVersion = 1;

uint64_t stored_vertex(VertexId vertex) {
    if constexpr (std::numeric_limits<VertexId>::max() > std::numeric_limits<uint64_t>::max()) {
        if (vertex > std::numeric_limits<uint64_t>::max()) {
            throw std::runtime_error("ch_io: vertex id is too large for binary format");
        }
    }
    return static_cast<uint64_t>(vertex);
}

VertexId memory_vertex(uint64_t vertex) {
    if constexpr (std::numeric_limits<uint64_t>::max() > std::numeric_limits<VertexId>::max()) {
        if (vertex > std::numeric_limits<VertexId>::max()) {
            throw std::runtime_error("ch_io: vertex id exceeds addressable size");
        }
    }
    return static_cast<VertexId>(vertex);
}

size_t memory_size(uint64_t value) {
    if constexpr (std::numeric_limits<uint64_t>::max() > std::numeric_limits<size_t>::max()) {
        if (value > std::numeric_limits<size_t>::max()) {
            throw std::runtime_error("ch_io: count exceeds addressable size");
        }
    }
    return static_cast<size_t>(value);
}

void write_offsets(std::ofstream &out, const std::vector<uint64_t> &offsets) {
    io::write_span(out, std::span<const uint64_t>(offsets));
}

void read_offsets(std::ifstream &in, std::vector<uint64_t> &offsets) {
    io::read_span(in, std::span<uint64_t>(offsets), "ch_io: truncated offsets");
}

void write_edges(std::ofstream &out, const std::vector<Edge> &edges) {
    for (const Edge &edge : edges) {
        io::write_one(out, stored_vertex(edge.to));
        io::write_one(out, edge.weight);
    }
}

void read_edges(std::ifstream &in, std::vector<Edge> &edges) {
    for (Edge &edge : edges) {
        uint64_t to = 0;
        if (!io::read_one(in, to) || !io::read_one(in, edge.weight)) {
            throw std::runtime_error("ch_io: truncated edges");
        }
        edge.to = memory_vertex(to);
    }
}

void validate_offsets(const std::vector<uint64_t> &offsets, uint64_t edge_count) {
    if (offsets.empty() || offsets.front() != 0 || offsets.back() != edge_count || !std::ranges::is_sorted(offsets)) {
        throw std::runtime_error("ch_io: invalid offsets");
    }
}

void validate_rank(const std::vector<uint32_t> &rank) {
    std::vector<uint8_t> seen(rank.size(), 0);
    for (const uint32_t r : rank) {
        if (r >= rank.size() || seen[r] != 0) {
            throw std::runtime_error("ch_io: rank is not a permutation");
        }
        seen[r] = 1;
    }
}

void validate_edges(const std::vector<Edge> &edges, VertexId vertices) {
    for (const Edge &edge : edges) {
        if (edge.to >= vertices) {
            throw std::runtime_error("ch_io: edge target out of range");
        }
    }
}

void validate_ch(const ContractionHierarchy &ch) {
    const VertexId vertices = ch.vertex_count();
    if (vertices > std::numeric_limits<uint32_t>::max()) {
        throw std::runtime_error("ch_io: hierarchy has too many vertices for binary format");
    }
    if (ch.forward_offsets.size() != vertices + 1 || ch.backward_offsets.size() != vertices + 1) {
        throw std::runtime_error("ch_io: invalid offset count");
    }
    validate_rank(ch.rank);
    validate_offsets(ch.forward_offsets, static_cast<uint64_t>(ch.forward_edges.size()));
    validate_offsets(ch.backward_offsets, static_cast<uint64_t>(ch.backward_edges.size()));
    validate_edges(ch.forward_edges, vertices);
    validate_edges(ch.backward_edges, vertices);
}

} // namespace

bool save_ch(const ContractionHierarchy &ch, const std::filesystem::path &path) {
    validate_ch(ch);

    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }

    io::write_one(out, kChMagic);
    io::write_one(out, kChVersion);
    io::write_one(out, static_cast<uint32_t>(ch.vertex_count()));
    io::write_one(out, static_cast<uint64_t>(ch.forward_edges.size()));
    io::write_one(out, static_cast<uint64_t>(ch.backward_edges.size()));
    io::write_span(out, std::span<const uint32_t>(ch.rank));
    write_offsets(out, ch.forward_offsets);
    write_edges(out, ch.forward_edges);
    write_offsets(out, ch.backward_offsets);
    write_edges(out, ch.backward_edges);
    return static_cast<bool>(out);
}

ContractionHierarchy load_ch(const std::filesystem::path &path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("ch_io: failed to open file: " + path.string());
    }

    uint32_t magic = 0;
    uint32_t version = 0;
    uint32_t vertices = 0;
    uint64_t forward_edges = 0;
    uint64_t backward_edges = 0;
    if (!io::read_one(in, magic) || !io::read_one(in, version) || !io::read_one(in, vertices) ||
        !io::read_one(in, forward_edges) || !io::read_one(in, backward_edges)) {
        throw std::runtime_error("ch_io: truncated header");
    }
    if (magic != kChMagic) {
        throw std::runtime_error("ch_io: invalid magic");
    }
    if (version != kChVersion) {
        throw std::runtime_error("ch_io: unsupported version");
    }

    ContractionHierarchy ch;
    ch.rank.resize(vertices);
    ch.forward_offsets.resize(static_cast<size_t>(vertices) + 1);
    ch.forward_edges.resize(memory_size(forward_edges));
    ch.backward_offsets.resize(static_cast<size_t>(vertices) + 1);
    ch.backward_edges.resize(memory_size(backward_edges));

    io::read_span(in, std::span<uint32_t>(ch.rank), "ch_io: truncated rank array");
    read_offsets(in, ch.forward_offsets);
    read_edges(in, ch.forward_edges);
    read_offsets(in, ch.backward_offsets);
    read_edges(in, ch.backward_edges);

    validate_ch(ch);
    return ch;
}

} // namespace transport::ch
