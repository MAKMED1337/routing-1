#include "io/graph_io.hpp"

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>

#include "io/binary_io.hpp"

namespace transport {

namespace {

constexpr uint32_t kMagicIntWeights = 0x54524733U; // TRG3
constexpr uint32_t kMagicCoords = 0x43524431U;     // CRD1

uint64_t stored_offset(size_t offset) {
    if constexpr (std::numeric_limits<size_t>::max() > std::numeric_limits<uint64_t>::max()) {
        if (offset > std::numeric_limits<uint64_t>::max()) {
            throw std::runtime_error("graph offset is too large for TRG3 binary format");
        }
    }
    return static_cast<uint64_t>(offset);
}

size_t memory_offset(uint64_t offset) {
    if constexpr (std::numeric_limits<uint64_t>::max() > std::numeric_limits<size_t>::max()) {
        if (offset > std::numeric_limits<size_t>::max()) {
            throw std::runtime_error("corrupted graph file: offset exceeds addressable size");
        }
    }
    return static_cast<size_t>(offset);
}

void validate_graph(const Graph &graph, uint64_t expected_edges) {
    if (graph.offsets.size() != static_cast<size_t>(graph.vertex_count()) + 1) {
        throw std::runtime_error("corrupted graph file: invalid offset count");
    }
    if (graph.edge_count() != expected_edges) {
        throw std::runtime_error("corrupted graph file: invalid edge count");
    }
    if (!graph.offsets.empty() && graph.offsets.front() != 0) {
        throw std::runtime_error("corrupted graph file: first offset must be zero");
    }

    size_t previous = 0;
    for (const size_t offset : graph.offsets) {
        if (offset < previous) {
            throw std::runtime_error("corrupted graph file: offsets must be monotonic");
        }
        if (stored_offset(offset) > expected_edges) {
            throw std::runtime_error("corrupted graph file: offset exceeds edge count");
        }
        previous = offset;
    }
    if (!graph.offsets.empty() && stored_offset(graph.offsets.back()) != expected_edges) {
        throw std::runtime_error("corrupted graph file: final offset must match edge count");
    }

    const VertexId vertices = graph.vertex_count();
    for (const Edge &edge : graph.edges) {
        if (edge.to >= vertices) {
            throw std::runtime_error("corrupted graph file: edge destination out of range");
        }
    }
}

} // namespace

bool save_graph_binary(const Graph &graph, const std::filesystem::path &path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }

    io::write_one(out, kMagicIntWeights);
    // TRG3 stores the vertex count as a fixed-width header field.
    const VertexId vertex_count = graph.vertex_count();
    if (vertex_count > std::numeric_limits<uint32_t>::max()) {
        throw std::runtime_error("graph has too many vertices for TRG3 binary format");
    }
    const uint32_t vertices = static_cast<uint32_t>(vertex_count);
    const uint64_t edges = graph.edge_count();
    io::write_one(out, vertices);
    io::write_one(out, edges);

    for (const size_t offset : graph.offsets) {
        io::write_one(out, stored_offset(offset));
    }
    io::write_span(out, std::span<const Edge>(graph.edges));
    return static_cast<bool>(out);
}

Graph load_graph_binary(const std::filesystem::path &path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("failed to open graph file: " + path.string());
    }

    uint32_t magic = 0;
    if (!io::read_one(in, magic)) {
        throw std::runtime_error("corrupted graph file: failed to read magic");
    }
    if (magic != kMagicIntWeights) {
        throw std::runtime_error("invalid graph file magic");
    }

    uint32_t vertices = 0;
    uint64_t edges = 0;
    if (!io::read_one(in, vertices)) {
        throw std::runtime_error("corrupted graph file: failed to read vertex count");
    }
    if (!io::read_one(in, edges)) {
        throw std::runtime_error("corrupted graph file: failed to read edge count");
    }

    Graph graph;
    graph.vertex_count_ = vertices;

    graph.offsets.resize(static_cast<size_t>(vertices) + 1);
    graph.edges.resize(static_cast<size_t>(edges));

    for (size_t &offset : graph.offsets) {
        uint64_t stored_offset = 0;
        io::read_one(in, stored_offset);
        if (!in) {
            throw std::runtime_error("corrupted graph file");
        }
        offset = memory_offset(stored_offset);
    }

    io::read_span(in, std::span<Edge>(graph.edges), "corrupted graph file");

    validate_graph(graph, edges);
    return graph;
}

bool save_coords_binary(std::span<const NodeCoord> coords, const std::filesystem::path &path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }

    io::write_one(out, kMagicCoords);
    if (coords.size() > std::numeric_limits<uint32_t>::max()) {
        throw std::runtime_error("too many coordinates for CRD1 binary format");
    }
    const auto vertices = static_cast<uint32_t>(coords.size());
    io::write_one(out, vertices);
    for (const NodeCoord &node : coords) {
        io::write_one(out, node.lat);
        io::write_one(out, node.lon);
    }
    return static_cast<bool>(out);
}

std::vector<NodeCoord> load_coords_binary(const std::filesystem::path &path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("failed to open coords file: " + path.string());
    }

    uint32_t magic = 0;
    if (!io::read_one(in, magic)) {
        throw std::runtime_error("corrupted coords file: failed to read magic");
    }
    if (magic != kMagicCoords) {
        throw std::runtime_error("invalid coords file magic");
    }

    uint32_t vertices = 0;
    if (!io::read_one(in, vertices)) {
        throw std::runtime_error("corrupted coords file: failed to read vertex count");
    }

    std::vector<NodeCoord> coords(vertices);
    for (NodeCoord &node : coords) {
        io::read_one(in, node.lat);
        io::read_one(in, node.lon);
        if (!in) {
            throw std::runtime_error("corrupted coords file");
        }
    }
    return coords;
}

} // namespace transport
