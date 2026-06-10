#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/routing_instance.hpp"
#include "apps/bench_utils.hpp"
#include "graph/graph_io.hpp"
#include "routing/routing.hpp"

#include <CLI/CLI.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <optional>
#include <random>
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
using transport::to_seconds;
using transport::VertexId;

namespace {

struct TimedResult {
    PathResult path;
    uint64_t query_us = 0;
};

TimedResult query_timed(const RoutingAlgorithm &algorithm, VertexId source, VertexId target) {
    const auto t0 = std::chrono::steady_clock::now();
    const PathResult result = algorithm.query(source, target);
    const auto t1 = std::chrono::steady_clock::now();
    return TimedResult{result,
                       static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count())};
}

void print_preprocessing_metrics(std::string_view prefix, const RoutingInstance &instance) {
    const PreprocessReport &report = instance.stats;
    std::cout << prefix << "_dependency_preprocess_wall_s=" << report.dependency.wall_s << "\n";
    std::cout << prefix << "_dependency_preprocess_cpu_s=" << report.dependency.cpu_s << "\n";
    std::cout << prefix << "_algorithm_preprocess_wall_s=" << report.algorithm.wall_s << "\n";
    std::cout << prefix << "_algorithm_preprocess_cpu_s=" << report.algorithm.cpu_s << "\n";
    std::cout << prefix << "_after_dependency_preprocess_process_peak_rss_mb=" << report.dependency.process_peak_rss_mb
              << "\n";
    std::cout << prefix << "_after_algorithm_preprocess_process_peak_rss_mb=" << report.algorithm.process_peak_rss_mb
              << "\n";
    if (report.dependency.ch) {
        std::cout << prefix << "_dependency_ch_witness_calls=" << report.dependency.ch->witness_calls << "\n";
        std::cout << prefix << "_dependency_ch_ordering_init_s=" << to_seconds(report.dependency.ch->ordering_init_ns)
                  << "\n";
    }
    if (const auto *ch = dynamic_cast<const ContractionHierarchyAlgorithm *>(instance.algorithm.get())) {
        std::cout << prefix << "_auxiliary_edges=" << ch->auxiliary_edge_count() << "\n";
    }
}

struct QueryAggregateInput {
    std::vector<uint64_t> query_us;
    std::vector<uint32_t> settled;
    std::vector<uint64_t> relaxed_arcs;
    std::vector<uint64_t> heap_pushes;
    std::vector<uint64_t> heuristic_evals;
    std::vector<uint64_t> pruned_by_flag;
};

template <typename T> double mean_of(const std::vector<T> &v) {
    if (v.empty()) {
        return 0.0;
    }
    return static_cast<double>(std::accumulate(v.begin(), v.end(), T{0})) / static_cast<double>(v.size());
}

template <typename T> double percentile_of(std::vector<T> v, double pct) {
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    const size_t idx = std::min(static_cast<size_t>(static_cast<double>(v.size()) * pct / 100.0), v.size() - 1);
    return static_cast<double>(v[idx]);
}

bench::Json aggregate_to_json(QueryAggregateInput &agg) {
    bench::Json j;
    j["mean_us"] = mean_of(agg.query_us);
    j["p50_us"] = percentile_of(agg.query_us, 50.0);
    j["p95_us"] = percentile_of(agg.query_us, 95.0);
    j["p99_us"] = percentile_of(agg.query_us, 99.0);
    j["max_us"] =
        agg.query_us.empty() ? 0.0 : static_cast<double>(*std::max_element(agg.query_us.begin(), agg.query_us.end()));
    j["mean_settled"] = mean_of(agg.settled);
    j["p50_settled"] = percentile_of(agg.settled, 50.0);
    j["p95_settled"] = percentile_of(agg.settled, 95.0);
    j["mean_relaxed_arcs"] = mean_of(agg.relaxed_arcs);
    j["mean_heap_pushes"] = mean_of(agg.heap_pushes);
    j["mean_heuristic_evals"] = mean_of(agg.heuristic_evals);
    j["mean_pruned_by_flag"] = mean_of(agg.pruned_by_flag);
    return j;
}

} // namespace

int main(int argc, char **argv) {
    std::string graph_path;
    std::string coords_path;
    std::string out_path = "reports/benchmarks/results.json";
    std::string algorithm_a = "dijkstra";
    std::string algorithm_b = "ch";
    uint32_t queries = 10'000;
    uint32_t min_settled = 100'000;
    uint32_t max_settled = 1'000'000;
    uint32_t seed = 1;

    CLI::App app{"Compare two routing algorithms on sampled graph queries"};
    app.add_option("--graph", graph_path, "Path to graph binary")->required()->check(CLI::ExistingFile);
    app.add_option("--coords", coords_path, "Path to coordinates binary")->check(CLI::ExistingFile);
    app.add_option("--out", out_path, "Output JSON path")->default_val("reports/benchmarks/results.json");
    app.add_option("--queries", queries, "Accepted query count")->default_val(10'000)->check(CLI::PositiveNumber);
    app.add_option("--min-settled", min_settled, "Minimum settled vertices for accepted baseline queries")
        ->default_val(100'000);
    app.add_option("--max-settled", max_settled, "Maximum settled vertices for accepted baseline queries")
        ->default_val(1'000'000);
    app.add_option("--seed", seed, "Random seed")->default_val(1);
    app.add_option("--algorithm-a", algorithm_a, "First routing algorithm")->default_val("dijkstra");
    app.add_option("--algorithm-b", algorithm_b, "Second routing algorithm")->default_val("ch");

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
    } catch (const std::exception &err) {
        std::cerr << err.what() << "\n";
        return 1;
    }
    const RoutingAlgorithm &runner_a_algo = *instance_a->algorithm;
    const RoutingAlgorithm &runner_b_algo = *instance_b->algorithm;
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

    QueryAggregateInput agg_a;
    QueryAggregateInput agg_b;
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

        agg_a.query_us.push_back(a.query_us);
        agg_a.settled.push_back(a.path.stats.settled);
        agg_a.relaxed_arcs.push_back(a.path.stats.relaxed_arcs);
        agg_a.heap_pushes.push_back(a.path.stats.heap_pushes);
        agg_a.heuristic_evals.push_back(a.path.stats.heuristic_evals);
        agg_a.pruned_by_flag.push_back(a.path.stats.pruned_by_flag);

        agg_b.query_us.push_back(b.query_us);
        agg_b.settled.push_back(b.path.stats.settled);
        agg_b.relaxed_arcs.push_back(b.path.stats.relaxed_arcs);
        agg_b.heap_pushes.push_back(b.path.stats.heap_pushes);
        agg_b.heuristic_evals.push_back(b.path.stats.heuristic_evals);
        agg_b.pruned_by_flag.push_back(b.path.stats.pruned_by_flag);

        ++accepted;
    }

    bench::Json j;
    j["algorithm_a"] = runner_a_algo.name();
    j["algorithm_b"] = runner_b_algo.name();
    j["date"] = bench::current_datetime_iso();
    j["graph"] =
        bench::Json{{"path", graph_path}, {"vertices", graph.vertex_count()}, {"directed_edges", graph.edge_count()}};
    j["distance_scale"] = transport::kDistanceScale;
    j["filters"] = bench::Json{{"min_settled", min_settled}, {"max_settled", max_settled}};
    j["preprocessing"] = bench::Json{
        {"algorithm_a_wall_s", instance_a->stats.dependency.wall_s + instance_a->stats.algorithm.wall_s},
        {"algorithm_b_wall_s", instance_b->stats.dependency.wall_s + instance_b->stats.algorithm.wall_s},
    };
    j["queries"] = bench::Json{
        {"requested", queries},
        {"accepted", accepted},
        {"attempted", attempts},
        {"seed", seed},
        {"algorithm_a", aggregate_to_json(agg_a)},
        {"algorithm_b", aggregate_to_json(agg_b)},
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
