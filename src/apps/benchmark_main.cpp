#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/routing_instance.hpp"
#include "graph/graph_io.hpp"
#include "routing/routing.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace fs = std::filesystem;
using transport::ContractionHierarchyAlgorithm;
using transport::Graph;
using transport::PathResult;
using transport::RoutingAlgorithm;
using transport::RoutingInstance;
using transport::VertexId;

namespace {

struct TimedResult {
    PathResult path;
    uint64_t query_us = 0;
};

struct TimedAlgorithmResult {
    std::string_view name;
    const TimedResult &timed;
};

void print_usage() {
    std::cerr << "usage: transport_benchmark --graph <graph.bin> "
                 "[--algorithm-a dijkstra|astar|alt|bidijkstra|bidi_astar|ch|arcflags|chase|hl] "
                 "[--algorithm-b dijkstra|astar|alt|bidijkstra|bidi_astar|ch|arcflags|chase|hl] [--queries N] "
                 "[--min-settled A] "
                 "[--max-settled B] "
                 "[--seed S] [--out file] [--coords <coords.bin>]\n";
}

std::string require_value(int argc, char **argv, int &i, std::string_view key) {
    if (i + 1 >= argc) {
        throw std::invalid_argument("missing value for " + std::string(key));
    }
    ++i;
    return argv[i];
}

uint32_t parse_u32(std::string_view text, std::string_view key) {
    size_t consumed = 0;
    const std::string value(text);
    if (value.empty() || value.front() == '-') {
        throw std::invalid_argument("invalid integer for " + std::string(key) + ": " + value);
    }
    const unsigned long long parsed = std::stoull(value, &consumed);
    if (consumed != value.size()) {
        throw std::invalid_argument("invalid integer for " + std::string(key) + ": " + value);
    }
    if (parsed > std::numeric_limits<uint32_t>::max()) {
        throw std::invalid_argument("value too large for " + std::string(key) + ": " + value);
    }
    return static_cast<uint32_t>(parsed);
}

TimedResult query_timed(const RoutingAlgorithm &algorithm, VertexId source, VertexId target) {
    const auto t0 = std::chrono::steady_clock::now();
    const PathResult result = algorithm.query(source, target);
    const auto t1 = std::chrono::steady_clock::now();
    return TimedResult{result,
                       static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count())};
}

bool same_distance(uint64_t a, uint64_t b) { return a == b; }

void print_preprocessing_metrics(std::string_view prefix, const RoutingInstance &instance) {
    const RoutingInstance::PreprocessTiming &timing = instance.timing();
    std::cout << prefix << "_dependency_preprocess_wall_s=" << timing.dependency_wall_s << "\n";
    std::cout << prefix << "_dependency_preprocess_cpu_s=" << timing.dependency_cpu_s << "\n";
    std::cout << prefix << "_algorithm_preprocess_wall_s=" << timing.algorithm_wall_s << "\n";
    std::cout << prefix << "_algorithm_preprocess_cpu_s=" << timing.algorithm_cpu_s << "\n";
    std::cout << prefix << "_after_dependency_preprocess_peak_rss_mb=" << timing.after_dependency_peak_rss_mb << "\n";
    std::cout << prefix << "_after_algorithm_preprocess_peak_rss_mb=" << timing.after_algorithm_peak_rss_mb << "\n";
    if (const auto *ch = dynamic_cast<const ContractionHierarchyAlgorithm *>(&instance.algorithm())) {
        std::cout << prefix << "_auxiliary_edges=" << ch->auxiliary_edge_count() << "\n";
    }
}

void write_benchmark_row(std::ofstream &out, uint32_t query, VertexId source, VertexId target,
                         const std::array<TimedAlgorithmResult, 2> &results) {
    out << query << "," << source << "," << target;
    for (const TimedAlgorithmResult &result : results) {
        out << "," << result.name;
    }
    out << "," << transport::kDistanceScale;
    for (const TimedAlgorithmResult &result : results) {
        out << "," << result.timed.path.distance_units;
    }
    for (const TimedAlgorithmResult &result : results) {
        out << "," << result.timed.path.settled;
    }
    for (const TimedAlgorithmResult &result : results) {
        out << "," << result.timed.query_us;
    }
    out << "\n";
}

} // namespace

int main(int argc, char **argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        print_usage();
        return 0;
    }

    std::string graph_path;
    std::string coords_path;
    std::string out_path = "reports/benchmarks/results.csv";
    std::string algorithm_a = "dijkstra";
    std::string algorithm_b = "ch";
    uint32_t queries = 10'000;
    uint32_t min_settled = 100'000;
    uint32_t max_settled = 1'000'000;
    uint32_t seed = 1;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            if (key == "--graph") {
                graph_path = require_value(argc, argv, i, key);
            } else if (key == "--coords") {
                coords_path = require_value(argc, argv, i, key);
            } else if (key == "--out") {
                out_path = require_value(argc, argv, i, key);
            } else if (key == "--queries") {
                queries = parse_u32(require_value(argc, argv, i, key), key);
            } else if (key == "--min-settled") {
                min_settled = parse_u32(require_value(argc, argv, i, key), key);
            } else if (key == "--max-settled") {
                max_settled = parse_u32(require_value(argc, argv, i, key), key);
            } else if (key == "--seed") {
                seed = parse_u32(require_value(argc, argv, i, key), key);
            } else if (key == "--algorithm-a") {
                algorithm_a = require_value(argc, argv, i, key);
            } else if (key == "--algorithm-b") {
                algorithm_b = require_value(argc, argv, i, key);
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
        print_usage();
        return 1;
    }
    if (queries == 0) {
        std::cerr << "--queries must be > 0\n";
        return 1;
    }
    if (min_settled > max_settled) {
        std::cerr << "--min-settled must be <= --max-settled\n";
        return 1;
    }

    const Graph graph = transport::load_graph_binary(graph_path);
    if (graph.vertex_count() == 0) {
        std::cerr << "graph must contain at least one vertex\n";
        return 1;
    }
    std::vector<transport::NodeCoord> coords;
    if (!coords_path.empty()) {
        coords = transport::load_coords_binary(coords_path);
    }

    std::optional<RoutingInstance> instance_a;
    std::optional<RoutingInstance> instance_b;
    try {
        instance_a.emplace(transport::make_routing_instance(algorithm_a, graph, coords));
        instance_b.emplace(transport::make_routing_instance(algorithm_b, graph, coords));
        instance_a->preprocess();
        instance_b->preprocess();
    } catch (const std::exception &err) {
        std::cerr << err.what() << "\n";
        return 1;
    }
    RoutingAlgorithm &runner_a_algo = instance_a->algorithm();
    RoutingAlgorithm &runner_b_algo = instance_b->algorithm();
    print_preprocessing_metrics(runner_a_algo.name(), *instance_a);
    print_preprocessing_metrics(runner_b_algo.name(), *instance_b);

    std::mt19937 rng(seed);
    std::uniform_int_distribution<VertexId> pick(0, graph.vertex_count() - 1);

    const fs::path output_path(out_path);
    if (fs::exists(output_path)) {
        std::cerr << "output file already exists: " << out_path << "\n";
        return 1;
    }
    if (output_path.has_parent_path()) {
        fs::create_directories(output_path.parent_path());
    }
    std::ofstream out(out_path);
    if (!out) {
        std::cerr << "failed to open output file\n";
        return 1;
    }
    out << "query,source,target,algorithm_a,algorithm_b,distance_scale,"
        << "algorithm_a_units,algorithm_b_units,algorithm_a_settled,algorithm_b_settled,algorithm_a_us,algorithm_b_"
           "us\n";

    uint32_t accepted = 0;
    uint64_t attempts = 0;
    const uint64_t max_attempts = static_cast<uint64_t>(queries) * 100;
    while (accepted < queries && attempts < max_attempts) {
        ++attempts;
        const VertexId source = pick(rng);
        const VertexId target = pick(rng);
        if (source == target) {
            continue;
        }

        const TimedResult a = query_timed(runner_a_algo, source, target);
        const TimedResult b = query_timed(runner_b_algo, source, target);

        if (a.path.distance_units == transport::kUnreachable || a.path.settled < min_settled ||
            a.path.settled > max_settled) {
            continue;
        }
        if (!same_distance(a.path.distance_units, b.path.distance_units)) {
            std::cerr << "distance mismatch for query source=" << source << " target=" << target
                      << " algorithm_a=" << runner_a_algo.name() << " distance=" << a.path.distance_units
                      << " algorithm_b=" << runner_b_algo.name() << " distance=" << b.path.distance_units << "\n";
            return 2;
        }

        write_benchmark_row(
            out, accepted, source, target,
            {TimedAlgorithmResult{runner_a_algo.name(), a}, TimedAlgorithmResult{runner_b_algo.name(), b}});
        ++accepted;
    }

    std::cout << "algorithm_a=" << runner_a_algo.name() << "\n";
    std::cout << "algorithm_b=" << runner_b_algo.name() << "\n";
    std::cout << "accepted_queries=" << accepted << "\n";
    std::cout << "attempted_queries=" << attempts << "\n";
    std::cout << "output_csv=" << out_path << "\n";
    return accepted == queries ? 0 : 3;
}
