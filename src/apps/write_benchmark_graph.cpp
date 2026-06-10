#include "graph/graph_io.hpp"

#include <CLI/CLI.hpp>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {

void add_edge(std::vector<std::vector<transport::Edge>> &rows, const std::vector<transport::NodeCoord> &coords,
              uint32_t from, uint32_t to) {
    const transport::NodeCoord &a = coords[from];
    const transport::NodeCoord &b = coords[to];
    const uint32_t weight = static_cast<uint32_t>(
        std::ceil(transport::haversine_meters(a, b) * static_cast<double>(transport::kDistanceScale)));
    rows[from].push_back(transport::Edge{to, weight});
}

struct GridGraph {
    transport::Graph graph;
    std::vector<transport::NodeCoord> coords;
};

GridGraph make_grid_graph(uint32_t width, uint32_t height) {
    const uint32_t vertices = width * height;
    std::vector<transport::NodeCoord> coords(vertices);
    auto id = [width](uint32_t x, uint32_t y) { return y * width + x; };
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            coords[id(x, y)] =
                transport::NodeCoord{52.0 + static_cast<double>(y) * 0.01, 21.0 + static_cast<double>(x) * 0.01};
        }
    }

    std::vector<std::vector<transport::Edge>> rows(vertices);
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            const uint32_t v = id(x, y);
            if (x + 1 < width) {
                add_edge(rows, coords, v, id(x + 1, y));
                add_edge(rows, coords, id(x + 1, y), v);
            }
            if (y + 1 < height) {
                add_edge(rows, coords, v, id(x, y + 1));
                add_edge(rows, coords, id(x, y + 1), v);
            }
            if (x + 1 < width && y + 1 < height) {
                add_edge(rows, coords, v, id(x + 1, y + 1));
            }
        }
    }

    transport::Graph graph;
    graph.vertex_count_ = vertices;
    graph.offsets.assign(static_cast<size_t>(vertices) + 1, 0);
    for (uint32_t v = 0; v < vertices; ++v) {
        graph.offsets[v + 1] = graph.offsets[v] + rows[v].size();
    }
    for (const std::vector<transport::Edge> &row : rows) {
        graph.edges.insert(graph.edges.end(), row.begin(), row.end());
    }
    return GridGraph{.graph = std::move(graph), .coords = std::move(coords)};
}

} // namespace

int main(int argc, char **argv) {
    std::string graph_output;
    std::string coords_output;

    CLI::App app{"Write a synthetic benchmark grid graph"};
    app.add_option("output_graph", graph_output, "Output graph binary")->required();
    app.add_option("coords_output", coords_output, "Optional output coordinates binary");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &err) {
        return app.exit(err);
    }

    const GridGraph grid = make_grid_graph(30, 30);
    if (!transport::save_graph_binary(grid.graph, graph_output)) {
        std::cerr << "failed to save graph\n";
        return 1;
    }
    if (!coords_output.empty() && !transport::save_coords_binary(grid.coords, coords_output)) {
        std::cerr << "failed to save coordinates\n";
        return 1;
    }

    std::cout << "vertices=" << grid.graph.vertex_count() << "\n";
    std::cout << "directed_edges=" << grid.graph.edge_count() << "\n";
    return 0;
}
