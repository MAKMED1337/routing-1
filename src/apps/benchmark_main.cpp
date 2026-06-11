#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/routing_instance.hpp"
#include "apps/bench_utils.hpp"
#include "io/graph_io.hpp"
#include "routing/routing.hpp"

#include <CLI/CLI.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace fs = std::filesystem;
using transport::ContractionHierarchyAlgorithm;
using transport::Graph;
using transport::PathResult;
using transport::PreprocessReport;
using transport::RoutingAlgorithm;
using transport::RoutingInstance;
using transport::Stopwatch;
using transport::to_seconds;
using transport::VertexId;

namespace {

struct TimedResult {
    PathResult path;
    std::chrono::nanoseconds query_wall{};
    std::chrono::nanoseconds query_cpu{};
};

TimedResult query_timed(const RoutingAlgorithm &algorithm, VertexId source, VertexId target) {
    const Stopwatch sw;
    const PathResult result = algorithm.query(source, target);
    return TimedResult{result, sw.wall_elapsed(), sw.cpu_elapsed()};
}

bench::Json preprocess_to_json(const PreprocessReport &report, const RoutingAlgorithm &algorithm) {
    bench::Json j;
    j["dependency_wall_s"] = to_seconds(report.dependency.wall);
    j["dependency_cpu_s"] = to_seconds(report.dependency.cpu);
    j["after_dependency_peak_rss_mb"] = report.dependency.process_peak_rss_mb;
    if (report.dependency.ch) {
        j["dependency_ch_ordering_init_s"] = to_seconds(report.dependency.ch->ordering_init);
        j["dependency_ch_witness_calls"] = report.dependency.ch->witness_calls;
    }
    j["algorithm_wall_s"] = to_seconds(report.algorithm.wall);
    j["algorithm_cpu_s"] = to_seconds(report.algorithm.cpu);
    j["after_algorithm_peak_rss_mb"] = report.algorithm.process_peak_rss_mb;
    if (const auto *ch = dynamic_cast<const ContractionHierarchyAlgorithm *>(&algorithm)) {
        j["auxiliary_edges"] = ch->auxiliary_edge_count();
    }
    if (report.ch_loaded_from) {
        j["ch_loaded_from"] = *report.ch_loaded_from;
    }
    if (report.ch_saved_to) {
        j["ch_saved_to"] = *report.ch_saved_to;
    }
    if (report.arcflags_loaded_from) {
        j["arcflags_loaded_from"] = *report.arcflags_loaded_from;
    }
    if (report.arcflags_saved_to) {
        j["arcflags_saved_to"] = *report.arcflags_saved_to;
    }
    return j;
}

} // namespace

int main(int argc, char **argv) {
    std::string graph_path;
    std::string coords_path;
    std::string source_path;
    std::string out_path = "reports/benchmarks/results.json";
    std::string algorithm_a = "dijkstra";
    std::string algorithm_b = "ch";
    uint32_t queries = 10'000;
    uint32_t min_settled = 100'000;
    uint32_t max_settled = 1'000'000;
    uint32_t seed = 1;
    transport::RoutingPreprocessingContext context_a;
    transport::RoutingPreprocessingContext context_b;

    CLI::App app{"Compare two routing algorithms on sampled graph queries"};
    app.add_option("--graph", graph_path, "Path to graph binary")->required()->check(CLI::ExistingFile);
    app.add_option("--coords", coords_path, "Path to coordinates binary")->check(CLI::ExistingFile);
    app.add_option("--source", source_path, "Original source file path (e.g. OSM PBF) for benchmark metadata");
    app.add_option("--out", out_path, "Output JSON path")->default_val("reports/benchmarks/results.json");
    app.add_option("--queries", queries, "Accepted query count")->default_val(10'000)->check(CLI::PositiveNumber);
    app.add_option("--min-settled", min_settled, "Minimum settled vertices for accepted baseline queries")
        ->default_val(100'000);
    app.add_option("--max-settled", max_settled, "Maximum settled vertices for accepted baseline queries")
        ->default_val(1'000'000);
    app.add_option("--seed", seed, "Random seed")->default_val(1);
    app.add_option("--algorithm-a", algorithm_a, "First routing algorithm")->default_val("dijkstra");
    app.add_option("--algorithm-b", algorithm_b, "Second routing algorithm")->default_val("ch");
    app.add_option("--algorithm-a-ch-load", context_a.ch_load_path, "Load CH artifact for algorithm A")
        ->check(CLI::ExistingFile);
    app.add_option("--algorithm-a-ch-save", context_a.ch_save_path, "Save CH artifact for algorithm A");
    app.add_option("--algorithm-a-arcflags-load", context_a.arcflags_load_path,
                   "Load ArcFlags artifact for algorithm A")
        ->check(CLI::ExistingFile);
    app.add_option("--algorithm-a-arcflags-save", context_a.arcflags_save_path,
                   "Save ArcFlags artifact for algorithm A");
    app.add_option("--algorithm-b-ch-load", context_b.ch_load_path, "Load CH artifact for algorithm B")
        ->check(CLI::ExistingFile);
    app.add_option("--algorithm-b-ch-save", context_b.ch_save_path, "Save CH artifact for algorithm B");
    app.add_option("--algorithm-b-arcflags-load", context_b.arcflags_load_path,
                   "Load ArcFlags artifact for algorithm B")
        ->check(CLI::ExistingFile);
    app.add_option("--algorithm-b-arcflags-save", context_b.arcflags_save_path,
                   "Save ArcFlags artifact for algorithm B");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &err) {
        return app.exit(err);
    }

    if (queries == 0) {
        std::cerr << "--queries must be > 0\n";
        return 1;
    }
    if (min_settled > max_settled) {
        std::cerr << "--min-settled must be <= --max-settled\n";
        return 1;
    }
    std::set<std::string> artifact_save_paths;
    for (const auto &path :
         {context_a.ch_save_path, context_a.arcflags_save_path, context_b.ch_save_path, context_b.arcflags_save_path}) {
        if (path && fs::exists(*path)) {
            std::cerr << "artifact output file already exists: " << *path << "\n";
            return 1;
        }
        if (path) {
            if (!artifact_save_paths.insert(*path).second) {
                std::cerr << "duplicate artifact output path: " << *path << "\n";
                return 1;
            }
            const fs::path artifact_path(*path);
            if (artifact_path.has_parent_path()) {
                fs::create_directories(artifact_path.parent_path());
            }
        }
    }

    const Graph graph = transport::load_graph_binary(graph_path);
    if (graph.vertex_count() == 0) {
        std::cerr << "graph must contain at least one vertex\n";
        return 1;
    }
    const double after_load_rss = transport::peak_rss_mb();

    std::vector<transport::NodeCoord> coords;
    if (!coords_path.empty()) {
        coords = transport::load_coords_binary(coords_path);
    }

    std::optional<RoutingInstance> instance_a;
    std::optional<RoutingInstance> instance_b;
    try {
        instance_a.emplace(transport::make_routing_instance(algorithm_a, graph, coords, context_a));
        instance_b.emplace(transport::make_routing_instance(algorithm_b, graph, coords, context_b));
    } catch (const std::exception &err) {
        std::cerr << err.what() << "\n";
        return 1;
    }
    const RoutingAlgorithm &runner_a_algo = *instance_a->algorithm;
    const RoutingAlgorithm &runner_b_algo = *instance_b->algorithm;

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

    bench::QueryAggregateInput agg_a;
    bench::QueryAggregateInput agg_b;
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

        if (a.path.distance_units == transport::kUnreachable || a.path.stats.settled < min_settled ||
            a.path.stats.settled > max_settled) {
            continue;
        }
        if (a.path.distance_units != b.path.distance_units) {
            std::cerr << "distance mismatch for query source=" << source << " target=" << target
                      << " algorithm_a=" << runner_a_algo.name() << " distance=" << a.path.distance_units
                      << " algorithm_b=" << runner_b_algo.name() << " distance=" << b.path.distance_units << "\n";
            return 2;
        }

        agg_a.query_wall.push_back(a.query_wall);
        agg_a.query_cpu.push_back(a.query_cpu);
        agg_a.settled.push_back(a.path.stats.settled);
        agg_a.relaxed_arcs.push_back(a.path.stats.relaxed_arcs);
        agg_a.heap_pushes.push_back(a.path.stats.heap_pushes);
        agg_a.heuristic_evals.push_back(a.path.stats.heuristic_evals);
        agg_a.pruned_by_flag.push_back(a.path.stats.pruned_by_flag);

        agg_b.query_wall.push_back(b.query_wall);
        agg_b.query_cpu.push_back(b.query_cpu);
        agg_b.settled.push_back(b.path.stats.settled);
        agg_b.relaxed_arcs.push_back(b.path.stats.relaxed_arcs);
        agg_b.heap_pushes.push_back(b.path.stats.heap_pushes);
        agg_b.heuristic_evals.push_back(b.path.stats.heuristic_evals);
        agg_b.pruned_by_flag.push_back(b.path.stats.pruned_by_flag);

        ++accepted;
    }

    bench::Json graph_obj;
    graph_obj["path"] = graph_path;
    if (!source_path.empty()) {
        graph_obj["source"] = source_path;
    }
    graph_obj["vertices"] = graph.vertex_count();
    graph_obj["directed_edges"] = graph.edge_count();

    bench::Json j;
    j["algorithm_a"] = runner_a_algo.name();
    j["algorithm_b"] = runner_b_algo.name();
    j["date"] = bench::current_datetime_iso();
    j["graph"] = std::move(graph_obj);
    j["distance_scale"] = transport::kDistanceScale;
    j["after_load_peak_rss_mb"] = after_load_rss;
    j["filters"] = bench::Json{{"min_settled", min_settled}, {"max_settled", max_settled}};
    j["preprocessing"] = bench::Json{
        {"algorithm_a", preprocess_to_json(instance_a->stats, runner_a_algo)},
        {"algorithm_b", preprocess_to_json(instance_b->stats, runner_b_algo)},
        {"rss_note", "after_*_peak_rss_mb values are process-wide high-water marks; algorithm_b RSS includes "
                     "algorithm_a's retained memory"},
    };
    j["queries"] = bench::Json{
        {"requested", queries},
        {"accepted", accepted},
        {"attempted", attempts},
        {"seed", seed},
        {"algorithm_a", bench::aggregate_to_json(agg_a)},
        {"algorithm_b", bench::aggregate_to_json(agg_b)},
    };

    std::ofstream out(out_path);
    if (!out) {
        std::cerr << "failed to open output file\n";
        return 1;
    }
    out << j.dump(2) << "\n";

    std::cout << "algorithm_a=" << runner_a_algo.name() << "\n";
    std::cout << "algorithm_b=" << runner_b_algo.name() << "\n";
    std::cout << "accepted_queries=" << accepted << "\n";
    std::cout << "attempted_queries=" << attempts << "\n";
    std::cout << "output_json=" << out_path << "\n";
    return accepted == queries ? 0 : 3;
}
