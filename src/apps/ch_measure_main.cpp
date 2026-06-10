#include "algorithms/ch/contraction_hierarchy.hpp"
#include "apps/bench_utils.hpp"

#include <CLI/CLI.hpp>

#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
    bench::BenchmarkArgs args;

    CLI::App app{"Measure Contraction Hierarchy preprocessing and query performance"};
    app.add_option("graph", args.graph_path, "Input graph binary")->required()->check(CLI::ExistingFile);
    app.add_option("queries", args.query_count, "Query count")->default_val(10'000)->check(CLI::PositiveNumber);
    app.add_option("seed", args.seed, "Random seed")->default_val(1);
    app.add_option("source_file", args.source_path, "Original source file path for benchmark metadata");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError &err) {
        return app.exit(err);
    }

    try {
        auto loaded = bench::load_graph(args);
        transport::ContractionHierarchyAlgorithm ch(loaded.graph);

        bench::run_benchmark(
            args, loaded, "lazy edge-difference ordering, witness hop_limit=5", ch,
            [&ch]() -> bench::Json {
                const transport::PreprocessStats stats = ch.preprocess_stats();
                return {
                    {"ordering_init_wall_s", bench::to_seconds(stats.ordering_init)},
                    {"ordering_note", "initial PQ scoring pass only; lazy re-scores and find_shortcuts are interleaved "
                                      "throughout the full contraction loop"},
                    {"witness_calls", stats.witness_calls},
                    {"witness_hop_limit", 5},
                    {"witness_note", "counts all WitnessSearch::run() calls from both edge_difference (ordering) and "
                                     "find_shortcuts (contraction); no per-call timing to avoid measurement overhead"},
                    {"auxiliary_edges", ch.auxiliary_edge_count()},
                };
            });
    } catch (const std::exception &err) {
        std::cerr << err.what() << "\n";
        return 1;
    }

    return 0;
}
