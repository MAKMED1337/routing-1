#include "algorithms/dijkstra.hpp"
#include "apps/bench_utils.hpp"
#include "io/graph_io.hpp"
#include "routing/routing.hpp"

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using transport::DijkstraAlgorithm;
using transport::Graph;
using transport::kUnreachable;
using transport::Stopwatch;
using transport::to_seconds;
using transport::VertexId;

int main(int argc, char **argv) {
    std::string graph_path;
    std::string coords_path; // unused by Dijkstra but accepted for caller convenience
    std::string source_path;
    std::string out_path;
    uint32_t queries = 1000;
    uint32_t min_settled = 100000;
    uint32_t max_settled = 1000000;
    uint32_t seed = 1;

    CLI::App app{"Generate Dijkstra query pairs for replay benchmarking.\n"
                 "Runs Dijkstra only (one process, no candidate algorithm), filters pairs by settled-vertex\n"
                 "count, and writes (source, target, distance, settled) records to a JSON file.\n"
                 "The output can be replayed by transport_bench_replay against any algorithm."};
    app.add_option("--graph", graph_path, "Path to graph binary")->required()->check(CLI::ExistingFile);
    app.add_option("--coords", coords_path, "Coordinates binary (accepted but unused; for scripting consistency)")
        ->check(CLI::ExistingFile);
    app.add_option("--source", source_path, "Original source file path (e.g. OSM PBF) for output metadata");
    app.add_option("--out", out_path, "Output JSON path for query pairs")->required();
    app.add_option("--queries", queries, "Number of accepted query pairs to collect")
        ->default_val(1000)
        ->check(CLI::PositiveNumber);
    app.add_option("--min-settled", min_settled, "Minimum settled vertices for an accepted query")->default_val(100000);
    app.add_option("--max-settled", max_settled, "Maximum settled vertices for an accepted query")
        ->default_val(1000000);
    app.add_option("--seed", seed, "Random seed for pair sampling")->default_val(1);

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &err) {
        return app.exit(err);
    }

    if (min_settled > max_settled) {
        std::cerr << "--min-settled must be <= --max-settled\n";
        return 1;
    }
    const fs::path output_path(out_path);
    if (fs::exists(output_path)) {
        std::cerr << "output file already exists: " << out_path << "\n";
        return 1;
    }
    if (output_path.has_parent_path()) {
        fs::create_directories(output_path.parent_path());
    }

    const Stopwatch load_sw;
    const Graph graph = transport::load_graph_binary(graph_path);
    const std::chrono::nanoseconds load_wall = load_sw.wall_elapsed();
    const std::chrono::nanoseconds load_cpu = load_sw.cpu_elapsed();
    if (graph.vertex_count() == 0) {
        std::cerr << "graph must contain at least one vertex\n";
        return 1;
    }

    DijkstraAlgorithm dijkstra(graph);

    std::mt19937 rng(seed);
    std::uniform_int_distribution<VertexId> pick(0, graph.vertex_count() - 1);

    using bench::Json;
    std::vector<Json> pairs;
    pairs.reserve(queries);
    uint64_t attempted = 0;
    const uint64_t max_attempts = static_cast<uint64_t>(queries) * 1000;

    while (pairs.size() < queries && attempted < max_attempts) {
        ++attempted;
        const VertexId source = pick(rng);
        const VertexId target = pick(rng);
        if (source == target) {
            continue;
        }

        const Stopwatch q_sw;
        const transport::PathResult result = dijkstra.query(source, target);
        const std::chrono::nanoseconds q_wall = q_sw.wall_elapsed();
        const std::chrono::nanoseconds q_cpu = q_sw.cpu_elapsed();

        if (result.distance_units == kUnreachable) {
            continue;
        }
        if (result.stats.settled < min_settled || result.stats.settled > max_settled) {
            continue;
        }

        Json pair;
        pair["source"] = static_cast<uint64_t>(source);
        pair["target"] = static_cast<uint64_t>(target);
        pair["distance"] = result.distance_units;
        pair["settled"] = result.stats.settled;
        pair["dijkstra_wall_us"] = bench::to_microseconds(q_wall);
        pair["dijkstra_cpu_us"] = bench::to_microseconds(q_cpu);
        pairs.push_back(std::move(pair));
    }

    Json graph_obj;
    graph_obj["path"] = graph_path;
    if (!source_path.empty()) {
        graph_obj["source"] = source_path;
    }
    graph_obj["vertices"] = graph.vertex_count();
    graph_obj["directed_edges"] = graph.edge_count();

    Json j;
    j["date"] = bench::current_datetime_iso();
    j["graph"] = std::move(graph_obj);
    j["load_wall_s"] = to_seconds(load_wall);
    j["load_cpu_s"] = to_seconds(load_cpu);
    j["seed"] = seed;
    j["filters"] = Json{{"min_settled", min_settled}, {"max_settled", max_settled}};
    j["requested"] = queries;
    j["accepted"] = static_cast<uint64_t>(pairs.size());
    j["attempted"] = attempted;
    j["distance_scale"] = transport::kDistanceScale;
    j["pairs"] = std::move(pairs);

    std::ofstream out(out_path);
    if (!out) {
        std::cerr << "failed to open output file: " << out_path << "\n";
        return 1;
    }
    out << j.dump(2) << "\n";

    const bool full = static_cast<uint32_t>(j["accepted"].get<uint64_t>()) == queries;
    std::cout << "graph=" << graph_path << "\n";
    std::cout << "accepted=" << j["accepted"].get<uint64_t>() << "/" << queries << "\n";
    std::cout << "attempted=" << attempted << "\n";
    std::cout << "output=" << out_path << "\n";
    return full ? 0 : 3;
}
