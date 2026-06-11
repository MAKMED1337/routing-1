#include "algorithms/routing_instance.hpp"
#include "io/graph_io.hpp"
#include "routing/routing.hpp"

#include <CLI/CLI.hpp>

#include <filesystem>
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
    transport::RoutingPreprocessingContext preprocessing_context;

    CLI::App app{"Run a shortest-path query against a binary graph"};
    app.add_option("--graph", graph_path, "Path to graph binary")->required()->check(CLI::ExistingFile);
    app.add_option("--coords", coords_path, "Path to coordinates binary")->check(CLI::ExistingFile);
    app.add_option("--source", source, "Source vertex id")->required();
    app.add_option("--target", target, "Target vertex id")->required();
    app.add_option("--algorithm", algo, "Routing algorithm name")->default_val("dijkstra");
    app.add_option("--ch-load", preprocessing_context.ch_load_path, "Load a precomputed CH artifact")
        ->check(CLI::ExistingFile);
    app.add_option("--ch-save", preprocessing_context.ch_save_path, "Save a computed CH artifact");
    app.add_option("--arcflags-load", preprocessing_context.arcflags_load_path, "Load a precomputed ArcFlags artifact")
        ->check(CLI::ExistingFile);
    app.add_option("--arcflags-save", preprocessing_context.arcflags_save_path, "Save a computed ArcFlags artifact");

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
    for (const auto &path : {preprocessing_context.ch_save_path, preprocessing_context.arcflags_save_path}) {
        if (path && std::filesystem::exists(*path)) {
            std::cerr << "artifact output file already exists: " << *path << "\n";
            return 1;
        }
        if (path) {
            const std::filesystem::path artifact_path(*path);
            if (artifact_path.has_parent_path()) {
                std::filesystem::create_directories(artifact_path.parent_path());
            }
        }
    }

    std::vector<transport::NodeCoord> coords;
    if (!coords_path.empty()) {
        coords = transport::load_coords_binary(coords_path);
    }

    std::optional<transport::RoutingInstance> instance;
    try {
        instance.emplace(transport::make_routing_instance(algo, graph, coords, preprocessing_context));
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
    std::cout << "settled=" << result.stats.settled << "\n";
    std::cout << "relaxed_arcs=" << result.stats.relaxed_arcs << "\n";
    std::cout << "heap_pushes=" << result.stats.heap_pushes << "\n";
    std::cout << "heuristic_evals=" << result.stats.heuristic_evals << "\n";
    std::cout << "settled_forward=" << result.stats.settled_forward << "\n";
    std::cout << "settled_backward=" << result.stats.settled_backward << "\n";
    std::cout << "pruned_by_flag=" << result.stats.pruned_by_flag << "\n";
    return 0;
}
