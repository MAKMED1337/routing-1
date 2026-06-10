#include "algorithms/routing_instance.hpp"
#include "graph/graph_io.hpp"
#include "routing/routing.hpp"

#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

void print_usage() {
    std::cout << "usage: transport_query --graph <graph.bin> --source <id> --target <id> --algorithm "
                 "dijkstra|astar|alt|bidijkstra|bidi_astar|ch|arcflags|chase|hl [--coords <coords.bin>]\n";
}

std::string require_value(int argc, char **argv, int &i, std::string_view key) {
    if (i + 1 >= argc) {
        throw std::invalid_argument("missing value for " + std::string(key));
    }
    ++i;
    return argv[i];
}

transport::VertexId parse_vertex_id(std::string_view text, std::string_view key) {
    size_t consumed = 0;
    const std::string value(text);
    if (value.empty() || value.front() == '-') {
        throw std::invalid_argument("invalid integer for " + std::string(key) + ": " + value);
    }
    const unsigned long long parsed = std::stoull(value, &consumed);
    if (consumed != value.size()) {
        throw std::invalid_argument("invalid integer for " + std::string(key) + ": " + value);
    }
    if (parsed > std::numeric_limits<transport::VertexId>::max()) {
        throw std::invalid_argument("value too large for " + std::string(key) + ": " + value);
    }
    return static_cast<transport::VertexId>(parsed);
}

} // namespace

int main(int argc, char **argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        print_usage();
        return 0;
    }

    std::string graph_path;
    std::string coords_path;
    transport::VertexId source = 0;
    transport::VertexId target = 0;
    std::string algo = "dijkstra";

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            if (key == "--graph") {
                graph_path = require_value(argc, argv, i, key);
            } else if (key == "--coords") {
                coords_path = require_value(argc, argv, i, key);
            } else if (key == "--source") {
                source = parse_vertex_id(require_value(argc, argv, i, key), key);
            } else if (key == "--target") {
                target = parse_vertex_id(require_value(argc, argv, i, key), key);
            } else if (key == "--algorithm") {
                algo = require_value(argc, argv, i, key);
            } else {
                throw std::invalid_argument("unknown argument: " + key);
            }
        }
    } catch (const std::exception &err) {
        std::cerr << err.what() << "\n";
        print_usage();
        return 1;
    }

    if (graph_path.empty()) {
        std::cerr << "missing --graph\n";
        return 1;
    }

    const transport::Graph graph = transport::load_graph_binary(graph_path);
    if (source >= graph.vertex_count() || target >= graph.vertex_count()) {
        std::cerr << "source/target out of range\n";
        return 1;
    }

    std::vector<transport::NodeCoord> coords;
    if (!coords_path.empty()) {
        coords = transport::load_coords_binary(coords_path);
    }

    std::optional<transport::RoutingInstance> instance;
    try {
        instance.emplace(transport::make_routing_instance(algo, graph, coords));
        instance->preprocess();
    } catch (const std::exception &err) {
        std::cerr << err.what() << "\n";
        return 1;
    }

    const transport::RoutingAlgorithm &algorithm = instance->algorithm();
    const transport::PathResult result = algorithm.query(source, target);
    std::cout << "algorithm=" << algorithm.name() << "\n";
    std::cout << "distance_units=" << result.distance_units << "\n";
    std::cout << "distance_scale=" << transport::kDistanceScale << "\n";
    std::cout << "distance_m="
              << static_cast<double>(result.distance_units) / static_cast<double>(transport::kDistanceScale) << "\n";
    std::cout << "settled=" << result.settled << "\n";
    return 0;
}
