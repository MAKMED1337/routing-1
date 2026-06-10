#pragma once

#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stopwatch.hpp"
#include "graph/graph_io.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <functional>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <numeric>
#include <ostream>
#include <random>
#include <sstream>
#include <string>
#include <sys/resource.h>
#include <vector>

namespace bench {

using Json = nlohmann::ordered_json;
using transport::Graph;
using transport::RoutingAlgorithm;
using transport::Stopwatch;
using transport::to_seconds;
using transport::VertexId;

struct BenchmarkArgs {
    std::string graph_path;
    std::string source_path; // original data source (e.g. OSM PBF) — written to "graph.source" in JSON
    uint32_t query_count = 10'000;
    uint32_t seed = 1;
};

// Graph together with its load timing and post-load RSS, ready to hand to a RoutingAlgorithm constructor.
struct LoadedGraph {
    Graph graph;
    std::chrono::nanoseconds wall{};
    std::chrono::nanoseconds cpu{};
    uint64_t peak_rss_mb = 0;
};

inline double to_microseconds(std::chrono::nanoseconds d) {
    return std::chrono::duration<double, std::micro>(d).count();
}

inline uint64_t peak_rss_mb() {
    struct rusage ru{};
    getrusage(RUSAGE_SELF, &ru);
    return static_cast<uint64_t>(ru.ru_maxrss) / 1024;
}

inline std::string current_datetime_iso() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
    return oss.str();
}

inline std::chrono::nanoseconds percentile(const std::vector<std::chrono::nanoseconds> &sorted, double pct) {
    assert(!sorted.empty());
    const size_t idx =
        std::min(static_cast<size_t>(static_cast<double>(sorted.size()) * pct / 100.0), sorted.size() - 1);
    return sorted[idx];
}

// --- main entry points ---

inline LoadedGraph load_graph(const BenchmarkArgs &args) {
    const Stopwatch sw;
    Graph graph = transport::load_graph_binary(args.graph_path);
    const std::chrono::nanoseconds wall = sw.wall_elapsed();
    const std::chrono::nanoseconds cpu = sw.cpu_elapsed();
    return LoadedGraph{std::move(graph), wall, cpu, peak_rss_mb()};
}

// Preprocesses `algo`, runs the query loop, and writes a complete JSON object to `out`.
// `extra_fields` is called after preprocessing and must return a Json object with no keys
// overlapping the standard fields; its entries are merged before the "queries" block. Use it
// for algorithm-specific metadata (e.g. witness calls, auxiliary edge counts) that are only
// available post-preprocess. Pass {} to omit algorithm-specific fields.
inline void run_benchmark(const BenchmarkArgs &args, const LoadedGraph &loaded, std::string_view variant, RoutingAlgorithm &algo, std::function<Json()> extra_fields = {},
                          std::ostream &out = std::cout) {
    if (args.query_count == 0) {
        throw std::invalid_argument("benchmark query count must be > 0");
    }
    if (loaded.graph.vertex_count() == 0) {
        throw std::invalid_argument("benchmark graph must contain at least one vertex");
    }

    const Stopwatch pp_sw;
    algo.preprocess();
    const std::chrono::nanoseconds pp_wall = pp_sw.wall_elapsed();
    const std::chrono::nanoseconds pp_cpu = pp_sw.cpu_elapsed();
    const uint64_t after_preprocess_rss = peak_rss_mb();

    std::mt19937 rng(args.seed);
    std::uniform_int_distribution<VertexId> pick(0, loaded.graph.vertex_count() - 1);

    std::vector<std::chrono::nanoseconds> times;
    times.reserve(args.query_count);
    for (uint32_t i = 0; i < args.query_count; ++i) {
        const VertexId src = pick(rng);
        const VertexId dst = pick(rng);
        const Stopwatch t;
        (void)algo.query(src, dst);
        times.push_back(t.wall_elapsed());
    }
    std::sort(times.begin(), times.end());
    const std::chrono::nanoseconds total = std::accumulate(times.begin(), times.end(), std::chrono::nanoseconds{0});
    const double mean_us = to_microseconds(total) / static_cast<double>(times.size());

    Json graph_obj;
    graph_obj["path"] = args.graph_path;
    if (!args.source_path.empty()) {
        graph_obj["source"] = args.source_path;
    }
    graph_obj["vertices"] = loaded.graph.vertex_count();
    graph_obj["directed_edges"] = loaded.graph.edge_count();

    Json j;
    j["algorithm"] = algo.name();
    j["variant"] = variant;
    j["date"] = current_datetime_iso();
    j["graph"] = std::move(graph_obj);
    j["load_wall_s"] = to_seconds(loaded.wall);
    j["load_cpu_s"] = to_seconds(loaded.cpu);
    j["after_load_peak_rss_mb"] = loaded.peak_rss_mb;
    j["preprocess_wall_s"] = to_seconds(pp_wall);
    j["preprocess_cpu_s"] = to_seconds(pp_cpu);
    j["after_preprocess_peak_rss_mb"] = after_preprocess_rss;
    if (extra_fields) {
        const Json extra = extra_fields();
        for (const auto &[key, val] : extra.items()) {
            assert(!j.contains(key) && "extra_fields key conflicts with a standard benchmark field");
            assert(key != "queries" && "extra_fields must not use the reserved key \"queries\"");
            (void)val;
        }
        j.update(extra);
    }
    j["queries"] = Json{
        {"count", args.query_count},
        {"seed", args.seed},
        {"mean_us", mean_us},
        {"p50_us", to_microseconds(percentile(times, 50))},
        {"p95_us", to_microseconds(percentile(times, 95))},
        {"p99_us", to_microseconds(percentile(times, 99))},
        {"max_us", to_microseconds(times.back())},
    };

    out << j.dump(2) << "\n";
}

} // namespace bench
