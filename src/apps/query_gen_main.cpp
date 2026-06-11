#include "algorithms/heap_node.hpp"
#include "algorithms/stamped_vector.hpp"
#include "apps/bench_utils.hpp"
#include "graph/graph.hpp"
#include "io/graph_io.hpp"
#include "routing/routing.hpp"

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using transport::Distance;
using transport::Graph;
using transport::HeapNode;
using transport::HeapQueue;
using transport::kUnreachable;
using transport::StampedVector;
using transport::Stopwatch;
using transport::to_seconds;
using transport::VertexId;

namespace {

enum class SelectionMode { RangedDijkstra, Random };

SelectionMode parse_selection_mode(const std::string &s) {
    if (s == "ranged-dijkstra") {
        return SelectionMode::RangedDijkstra;
    }
    if (s == "random") {
        return SelectionMode::Random;
    }
    throw std::invalid_argument("unknown selection mode '" + s + "'; expected ranged-dijkstra or random");
}

// Runs Dijkstra from source, settling at most max_settled vertices.
// Returns (vertex, distance) pairs in settling order.
std::vector<std::pair<VertexId, Distance>> ranged_dijkstra(const Graph &graph, StampedVector<Distance> &dist,
                                                           VertexId source, uint32_t max_settled) {
    dist.reset();
    HeapQueue pq;
    dist.set(source, 0);
    pq.push({0, source});

    std::vector<std::pair<VertexId, Distance>> settled;
    settled.reserve(max_settled);

    while (!pq.empty() && settled.size() < max_settled) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != dist.get(top.v)) {
            continue;
        }
        settled.push_back({top.v, top.key});
        for (const auto &e : graph.adjacent_edges(top.v)) {
            const Distance nd = top.key + e.weight;
            if (nd < dist.get(e.to)) {
                dist.set(e.to, nd);
                pq.push({nd, e.to});
            }
        }
    }
    return settled;
}

} // namespace

int main(int argc, char **argv) {
    fs::path graph_path;
    fs::path coords_path;
    fs::path source_path;
    fs::path out_path;
    std::string selection_str = "ranged-dijkstra";
    uint32_t queries = 1000;
    uint32_t min_settled = 100000;
    uint32_t max_settled = 1000000;
    uint32_t seed = 1;

    CLI::App app{"Generate query pairs for replay benchmarking.\n"
                 "\n"
                 "Pair selection modes (--selection):\n"
                 "  ranged-dijkstra: runs Dijkstra from a random source until max_settled vertices are\n"
                 "    settled, then picks a target from the settled range [min_settled, max_settled].\n"
                 "    Stores (source, target, distance) with the exact Dijkstra distance.\n"
                 "  random: picks random (source, target) pairs without running Dijkstra.\n"
                 "    Faster but stores no reference distance; bench_replay skips distance validation."};
    app.add_option("--graph", graph_path, "Path to graph binary")->required()->check(CLI::ExistingFile);
    app.add_option("--coords", coords_path, "Coordinates binary (accepted but unused; for scripting consistency)")
        ->check(CLI::ExistingFile);
    app.add_option("--source", source_path, "Original source file path (e.g. OSM PBF) for output metadata");
    app.add_option("--out", out_path, "Output JSON path for query pairs")->required();
    app.add_option("--selection", selection_str, "Pair selection mode: ranged-dijkstra or random")
        ->default_val("ranged-dijkstra");
    app.add_option("--queries", queries, "Number of query pairs to generate")
        ->default_val(1000)
        ->check(CLI::PositiveNumber);
    app.add_option("--min-settled", min_settled,
                   "Minimum settled vertices for ranged-dijkstra; ignored for random mode")
        ->default_val(100000);
    app.add_option("--max-settled", max_settled,
                   "Maximum settled vertices for ranged-dijkstra; ignored for random mode")
        ->default_val(1000000);
    app.add_option("--seed", seed, "Random seed for pair sampling")->default_val(1);

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &err) {
        return app.exit(err);
    }

    SelectionMode mode;
    try {
        mode = parse_selection_mode(selection_str);
    } catch (const std::exception &err) {
        std::cerr << err.what() << "\n";
        return 1;
    }
    if (mode == SelectionMode::RangedDijkstra && min_settled > max_settled) {
        std::cerr << "--min-settled must be <= --max-settled\n";
        return 1;
    }
    if (fs::exists(out_path)) {
        std::cerr << "output file already exists: " << out_path << "\n";
        return 1;
    }
    if (out_path.has_parent_path()) {
        fs::create_directories(out_path.parent_path());
    }

    const Stopwatch load_sw;
    const Graph graph = transport::load_graph_binary(graph_path);
    const std::chrono::nanoseconds load_wall = load_sw.wall_elapsed();
    const std::chrono::nanoseconds load_cpu = load_sw.cpu_elapsed();
    if (graph.vertex_count() == 0) {
        std::cerr << "graph must contain at least one vertex\n";
        return 1;
    }

    std::mt19937 rng(seed);
    std::uniform_int_distribution<VertexId> pick(0, graph.vertex_count() - 1);

    using bench::Json;
    std::vector<Json> pairs;
    pairs.reserve(queries);
    uint64_t attempted = 0;

    if (mode == SelectionMode::RangedDijkstra) {
        StampedVector<Distance> dist(graph.vertex_count(), kUnreachable);
        const uint64_t max_attempts = static_cast<uint64_t>(queries) * 100;

        while (pairs.size() < queries && attempted < max_attempts) {
            ++attempted;
            const VertexId source = pick(rng);

            const auto settled = ranged_dijkstra(graph, dist, source, max_settled);
            if (settled.size() < min_settled) {
                continue;
            }

            std::uniform_int_distribution<size_t> pick_pos(min_settled - 1, settled.size() - 1);
            const size_t pos = pick_pos(rng);
            const auto [target, distance] = settled[pos];

            Json pair;
            pair["source"] = static_cast<uint64_t>(source);
            pair["target"] = static_cast<uint64_t>(target);
            pair["distance"] = distance;
            pair["settled"] = static_cast<uint32_t>(pos + 1);
            pairs.push_back(std::move(pair));
        }
    } else {
        // Random mode: no Dijkstra, no distance stored.
        while (pairs.size() < queries) {
            ++attempted;
            const VertexId source = pick(rng);
            const VertexId target = pick(rng);
            if (source == target) {
                continue;
            }
            Json pair;
            pair["source"] = static_cast<uint64_t>(source);
            pair["target"] = static_cast<uint64_t>(target);
            pairs.push_back(std::move(pair));
        }
    }

    Json graph_obj;
    graph_obj["path"] = graph_path.string();
    if (!source_path.empty()) {
        graph_obj["source"] = source_path.string();
    }
    graph_obj["vertices"] = graph.vertex_count();
    graph_obj["directed_edges"] = graph.edge_count();

    Json j;
    j["date"] = bench::current_datetime_iso();
    j["graph"] = std::move(graph_obj);
    j["load_wall_s"] = to_seconds(load_wall);
    j["load_cpu_s"] = to_seconds(load_cpu);
    j["seed"] = seed;
    j["selection"] = selection_str;
    if (mode == SelectionMode::RangedDijkstra) {
        j["filters"] = Json{{"min_settled", min_settled}, {"max_settled", max_settled}};
    }
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

    const uint64_t accepted = j["accepted"].get<uint64_t>();
    std::cout << "graph=" << graph_path.string() << "\n";
    std::cout << "selection=" << selection_str << "\n";
    std::cout << "accepted=" << accepted << "/" << queries << "\n";
    std::cout << "attempted=" << attempted << "\n";
    std::cout << "output=" << out_path.string() << "\n";
    return accepted == queries ? 0 : 3;
}
