#include "algorithms/ch/contraction_hierarchy.hpp"
#include "apps/bench_utils.hpp"

#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

uint32_t parse_u32(std::string_view text, std::string_view name) {
    size_t consumed = 0;
    const std::string value(text);
    if (value.empty() || value.front() == '-') {
        throw std::invalid_argument("invalid integer for " + std::string(name) + ": " + value);
    }
    const unsigned long long parsed = std::stoull(value, &consumed);
    if (consumed != value.size()) {
        throw std::invalid_argument("invalid integer for " + std::string(name) + ": " + value);
    }
    if (parsed > std::numeric_limits<uint32_t>::max()) {
        throw std::invalid_argument("value too large for " + std::string(name) + ": " + value);
    }
    return static_cast<uint32_t>(parsed);
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 2 || argc > 5) {
        std::cerr << "usage: ch_measure <graph.graph> [queries=10000] [seed=1] [source_file]\n";
        return 1;
    }

    bench::BenchmarkArgs args;
    args.graph_path = argv[1];

    try {
        if (argc >= 3) {
            args.query_count = parse_u32(argv[2], "queries");
        }
        if (argc >= 4) {
            args.seed = parse_u32(argv[3], "seed");
        }
        if (argc >= 5) {
            args.source_path = argv[4];
        }

        auto loaded = bench::load_graph(args);
        transport::ContractionHierarchyAlgorithm ch(loaded.graph);

        bench::run_benchmark(
            args, loaded, "lazy edge-difference ordering, witness hop_limit=5", ch,
            [&ch]() -> bench::Json {
                const transport::PreprocessStats stats = ch.preprocess_stats();
                return {
                    {"ordering_init_wall_s", bench::to_seconds(stats.ordering_init_ns)},
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
