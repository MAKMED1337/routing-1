#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/partition.hpp"
#include "algorithms/routing_instance.hpp"
#include "apps/bench_utils.hpp"
#include "io/graph_io.hpp"
#include "routing/routing.hpp"

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace fs = std::filesystem;
using transport::ContractionHierarchyAlgorithm;
using transport::Distance;
using transport::Graph;
using transport::PreprocessReport;
using transport::RoutingAlgorithm;
using transport::RoutingInstance;
using transport::RoutingPreprocessingContext;
using transport::Stopwatch;
using transport::to_seconds;
using transport::VertexId;

namespace {

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

struct QueryPair {
    VertexId source;
    VertexId target;
    std::optional<Distance> expected_distance;
};

struct PairsFile {
    std::string graph_path;
    std::string graph_source;
    uint64_t graph_vertices = 0;
    uint64_t graph_directed_edges = 0;
    uint32_t seed = 0;
    std::optional<uint32_t> min_settled;
    std::optional<uint32_t> max_settled;
    std::vector<QueryPair> pairs;
};

PairsFile load_pairs(const fs::path &path) {
    std::ifstream f(path);
    if (!f) {
        throw std::runtime_error("cannot open query pairs file: " + path.string());
    }
    const bench::Json j = bench::Json::parse(f);

    PairsFile pf;
    pf.graph_path = j.at("graph").at("path").get<std::string>();
    if (j.at("graph").contains("source")) {
        pf.graph_source = j.at("graph").at("source").get<std::string>();
    }
    pf.graph_vertices = j.at("graph").at("vertices").get<uint64_t>();
    pf.graph_directed_edges = j.at("graph").at("directed_edges").get<uint64_t>();
    pf.seed = j.at("seed").get<uint32_t>();
    if (j.contains("filters")) {
        pf.min_settled = j.at("filters").at("min_settled").get<uint32_t>();
        pf.max_settled = j.at("filters").at("max_settled").get<uint32_t>();
    }

    for (const auto &p : j.at("pairs")) {
        QueryPair pair;
        pair.source = p.at("source").get<VertexId>();
        pair.target = p.at("target").get<VertexId>();
        if (p.contains("distance")) {
            pair.expected_distance = p.at("distance").get<Distance>();
        }
        pf.pairs.push_back(std::move(pair));
    }
    return pf;
}

} // namespace

int main(int argc, char **argv) {
    fs::path graph_path;
    fs::path coords_path;
    fs::path source_path;
    fs::path pairs_path;
    fs::path out_path;
    std::string algorithm_name;
    std::string variant;
    RoutingPreprocessingContext context;

    CLI::App app{"Replay saved query pairs against one routing algorithm.\n"
                 "Loads a single algorithm per process for clean RSS accounting.\n"
                 "When pairs carry reference distances (ranged-dijkstra selection), validates every\n"
                 "candidate distance. Random-selection pairs have no reference distances; replay\n"
                 "measures query times only."};
    app.add_option("--graph", graph_path, "Path to graph binary")->required()->check(CLI::ExistingFile);
    app.add_option("--coords", coords_path, "Path to coordinates binary")->check(CLI::ExistingFile);
    app.add_option("--source", source_path, "Original source file path (e.g. OSM PBF) for output metadata");
    app.add_option("--queries", pairs_path, "Path to query pairs JSON produced by transport_query_gen")
        ->required()
        ->check(CLI::ExistingFile);
    app.add_option("--algorithm", algorithm_name, "Routing algorithm to benchmark")->required();
    app.add_option("--variant", variant, "Human-readable variant description for the output JSON")
        ->default_val("default");
    app.add_option("--out", out_path, "Output JSON path for benchmark results")->required();
    app.add_option("--ch-load", context.ch_load_path, "Load CH artifact from this path")->check(CLI::ExistingFile);
    app.add_option("--ch-save", context.ch_save_path, "Save CH artifact to this path");
    app.add_option("--arcflags-load", context.arcflags_load_path, "Load ArcFlags artifact from this path")
        ->check(CLI::ExistingFile);
    app.add_option("--arcflags-save", context.arcflags_save_path, "Save ArcFlags artifact to this path");
    app.add_option("--hl-label-fraction", context.hl_label_fraction,
                   "Hub Labels: fraction of top-ranked CH vertices to label (0 < f <= 1; default 0.25)")
        ->check(CLI::Range(0.0, 1.0));
    app.add_option("--hl-memory-budget-gb", context.hl_memory_budget_gb,
                   "Hub Labels: memory budget in GiB before throwing (default 18)")
        ->check(CLI::PositiveNumber);
    std::string arcflags_partition_str;
    app.add_option("--arcflags-regions", context.arcflags_regions, "Arc Flags: number of regions [1,64] (default 32)")
        ->check(CLI::Range(1u, 64u));
    app.add_option("--arcflags-partition", arcflags_partition_str,
                   "Arc Flags: partition method: grid|inertial|kaminpar (default inertial)")
        ->check(CLI::IsMember({"grid", "inertial", "kaminpar"}));
    app.add_option("--arcflags-threads", context.arcflags_threads, "Arc Flags: preprocessing thread count (default 1)")
        ->check(CLI::PositiveNumber);

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &err) {
        return app.exit(err);
    }

    if (!arcflags_partition_str.empty()) {
        try {
            context.arcflags_partition = transport::parse_partition_method(arcflags_partition_str);
        } catch (const std::exception &err) {
            std::cerr << err.what() << "\n";
            return 1;
        }
    }

    for (const auto &save_path : {context.ch_save_path, context.arcflags_save_path}) {
        if (save_path && fs::exists(*save_path)) {
            std::cerr << "artifact output file already exists: " << save_path->string() << "\n";
            return 1;
        }
    }
    {
        std::set<std::string> seen;
        for (const auto &sp : {context.ch_save_path, context.arcflags_save_path}) {
            if (sp && !seen.insert(sp->string()).second) {
                std::cerr << "duplicate artifact output path: " << sp->string() << "\n";
                return 1;
            }
        }
    }
    if (fs::exists(out_path)) {
        std::cerr << "output file already exists: " << out_path << "\n";
        return 1;
    }
    if (out_path.has_parent_path()) {
        fs::create_directories(out_path.parent_path());
    }

    PairsFile pf;
    try {
        pf = load_pairs(pairs_path);
    } catch (const std::exception &err) {
        std::cerr << "failed to load query pairs: " << err.what() << "\n";
        return 1;
    }
    if (pf.pairs.empty()) {
        std::cerr << "query pairs file contains no pairs\n";
        return 1;
    }

    const Stopwatch load_sw;
    const Graph graph = transport::load_graph_binary(graph_path);
    const std::chrono::nanoseconds load_wall = load_sw.wall_elapsed();
    const std::chrono::nanoseconds load_cpu = load_sw.cpu_elapsed();
    const double after_load_rss = transport::peak_rss_mb();

    if (graph.vertex_count() == 0) {
        std::cerr << "graph must contain at least one vertex\n";
        return 1;
    }
    if (graph.vertex_count() != static_cast<VertexId>(pf.graph_vertices) ||
        graph.edge_count() != pf.graph_directed_edges) {
        std::cerr << "graph mismatch: pairs file recorded " << pf.graph_vertices << " vertices / "
                  << pf.graph_directed_edges << " edges, loaded graph has " << graph.vertex_count() << " vertices / "
                  << graph.edge_count() << " edges\n";
        return 1;
    }

    std::vector<transport::NodeCoord> coords;
    if (!coords_path.empty()) {
        coords = transport::load_coords_binary(coords_path);
    }

    std::unique_ptr<RoutingInstance> instance;
    try {
        instance =
            std::make_unique<RoutingInstance>(transport::make_routing_instance(algorithm_name, graph, coords, context));
    } catch (const std::exception &err) {
        std::cerr << err.what() << "\n";
        return 1;
    }
    const RoutingAlgorithm &algo = *instance->algorithm;

    bench::QueryAggregateInput agg;
    agg.query_wall.reserve(pf.pairs.size());
    agg.query_cpu.reserve(pf.pairs.size());
    agg.settled.reserve(pf.pairs.size());
    agg.relaxed_arcs.reserve(pf.pairs.size());
    agg.heap_pushes.reserve(pf.pairs.size());
    agg.heuristic_evals.reserve(pf.pairs.size());
    agg.pruned_by_flag.reserve(pf.pairs.size());

    uint64_t mismatches = 0;
    bool distance_validated = false;
    for (const auto &pair : pf.pairs) {
        const Stopwatch q_sw;
        const transport::PathResult result = algo.query(pair.source, pair.target);
        const std::chrono::nanoseconds q_wall = q_sw.wall_elapsed();
        const std::chrono::nanoseconds q_cpu = q_sw.cpu_elapsed();

        if (pair.expected_distance) {
            distance_validated = true;
            if (result.distance_units != *pair.expected_distance) {
                std::cerr << "distance mismatch: source=" << pair.source << " target=" << pair.target
                          << " expected=" << *pair.expected_distance << " got=" << result.distance_units << "\n";
                ++mismatches;
            }
        }

        agg.query_wall.push_back(q_wall);
        agg.query_cpu.push_back(q_cpu);
        agg.settled.push_back(result.stats.settled);
        agg.relaxed_arcs.push_back(result.stats.relaxed_arcs);
        agg.heap_pushes.push_back(result.stats.heap_pushes);
        agg.heuristic_evals.push_back(result.stats.heuristic_evals);
        agg.pruned_by_flag.push_back(result.stats.pruned_by_flag);
    }

    if (distance_validated && mismatches > 0) {
        std::cerr << mismatches << " distance mismatch(es) detected\n";
    }

    bench::Json graph_obj;
    graph_obj["path"] = graph_path.string();
    if (!source_path.empty()) {
        graph_obj["source"] = source_path.string();
    } else if (!pf.graph_source.empty()) {
        graph_obj["source"] = pf.graph_source;
    }
    graph_obj["vertices"] = graph.vertex_count();
    graph_obj["directed_edges"] = graph.edge_count();

    bench::Json j;
    j["algorithm"] = std::string(algo.name());
    j["variant"] = variant;
    j["date"] = bench::current_datetime_iso();
    j["graph"] = std::move(graph_obj);
    j["query_set"] = pairs_path.string();
    j["distance_scale"] = transport::kDistanceScale;
    j["load_wall_s"] = to_seconds(load_wall);
    j["load_cpu_s"] = to_seconds(load_cpu);
    j["after_load_peak_rss_mb"] = after_load_rss;
    j["preprocessing"] = preprocess_to_json(instance->stats, algo);
    if (pf.min_settled) {
        j["query_set_filters"] = bench::Json{{"min_settled", *pf.min_settled}, {"max_settled", *pf.max_settled}};
    }
    {
        bench::Json queries_j = bench::aggregate_to_json(agg);
        queries_j["count"] = pf.pairs.size();
        queries_j["seed"] = pf.seed;
        queries_j["distance_validated"] = distance_validated;
        if (distance_validated) {
            queries_j["distance_mismatches"] = mismatches;
        }
        j["queries"] = std::move(queries_j);
    }

    std::ofstream out(out_path);
    if (!out) {
        std::cerr << "failed to open output file: " << out_path << "\n";
        return 1;
    }
    out << j.dump(2) << "\n";

    std::cout << "algorithm=" << algo.name() << "\n";
    std::cout << "queries=" << pf.pairs.size() << "\n";
    std::cout << "distance_validated=" << (distance_validated ? "true" : "false") << "\n";
    if (distance_validated) {
        std::cout << "mismatches=" << mismatches << "\n";
    }
    std::cout << "output=" << out_path.string() << "\n";
    return (distance_validated && mismatches > 0) ? 2 : 0;
}
