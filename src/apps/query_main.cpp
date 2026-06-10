#include "algorithms/routing_instance.hpp"
#include "graph/graph_io.hpp"
#include "routing/routing.hpp"

#include <CLI/CLI.hpp>

#include <iostream>
#include <optional>
#include <string>
#include <vector>

int main(int argc, char **argv) {
    std::string graph_path;
    std::string coords_path;
    transport::VertexId source = 0;
    transport::VertexId target = 0;
    std::string algo = "dijkstra";

    CLI::App app{"Run a shortest-path query against a binary graph"};
    app.add_option("--graph", graph_path, "Path to graph binary")->required()->check(CLI::ExistingFile);
    app.add_option("--coords", coords_path, "Path to coordinates binary")->check(CLI::ExistingFile);
    app.add_option("--source", source, "Source vertex id")->required();
    app.add_option("--target", target, "Target vertex id")->required();
    app.add_option("--algorithm", algo, "Routing algorithm name")->default_val("dijkstra");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &err) {
        return app.exit(err);
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
    } catch (const std::exception &err) {
        std::cerr << err.what() << "\n";
        return 1;
    }

    const transport::RoutingAlgorithm &algorithm = *instance->algorithm;
    const transport::PathResult result = algorithm.query(source, target);
    std::cout << "algorithm=" << algorithm.name() << "\n";
    std::cout << "distance_units=" << result.distance_units << "\n";
    std::cout << "distance_scale=" << transport::kDistanceScale << "\n";
    std::cout << "distance_m="
              << static_cast<double>(result.distance_units) / static_cast<double>(transport::kDistanceScale) << "\n";
    std::cout << "settled=" << result.settled << "\n";
    return 0;
}
