#include "algorithms/ch/ch_io.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "graph_fixtures.hpp"
#include "routing_test_utils.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {

bool same_edges(const std::vector<transport::Edge> &a, const std::vector<transport::Edge> &b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].to != b[i].to || a[i].weight != b[i].weight) {
            return false;
        }
    }
    return true;
}

bool expect_ch_load_failure(const std::filesystem::path &path) {
    try {
        (void)transport::ch::load_ch(path.string());
    } catch (const std::runtime_error &) {
        return true;
    }
    std::cerr << "ch_io: expected load failure for " << path << "\n";
    return false;
}

bool check_ch_round_trip() {
    const auto [graph, coords] = transport::test::make_grid_graph(3, 3);
    const transport::ContractionHierarchyBuildResult built = transport::build_contraction_hierarchy(graph);

    const std::filesystem::path path = std::filesystem::temp_directory_path() / "transport_ch_round_trip.ch";
    if (!transport::ch::save_ch(built.hierarchy, path.string())) {
        std::cerr << "ch_io: failed to save hierarchy\n";
        return false;
    }
    const transport::ContractionHierarchy loaded = transport::ch::load_ch(path.string());
    std::filesystem::remove(path);

    if (loaded.rank != built.hierarchy.rank || loaded.forward_offsets != built.hierarchy.forward_offsets ||
        loaded.backward_offsets != built.hierarchy.backward_offsets ||
        !same_edges(loaded.forward_edges, built.hierarchy.forward_edges) ||
        !same_edges(loaded.backward_edges, built.hierarchy.backward_edges)) {
        std::cerr << "ch_io: hierarchy changed after round-trip\n";
        return false;
    }

    transport::ContractionHierarchyAlgorithm ch_algo(graph, transport::ContractionHierarchy(loaded));
    return transport::test::check_all_pairs(graph, ch_algo, "loaded ch");
}

bool check_ch_truncation_fails() {
    const transport::Graph graph = transport::test::make_line_graph();
    const transport::ContractionHierarchyBuildResult built = transport::build_contraction_hierarchy(graph);
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "transport_ch_truncated.ch";
    if (!transport::ch::save_ch(built.hierarchy, path.string())) {
        std::cerr << "ch_io: failed to save truncation fixture\n";
        return false;
    }
    std::filesystem::resize_file(path, 8);
    const bool ok = expect_ch_load_failure(path);
    std::filesystem::remove(path);
    return ok;
}

bool check_bad_rank_fails() {
    const transport::Graph graph = transport::test::make_line_graph();
    const transport::ContractionHierarchyBuildResult built = transport::build_contraction_hierarchy(graph);
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "transport_ch_bad_rank.ch";
    if (!transport::ch::save_ch(built.hierarchy, path.string())) {
        std::cerr << "ch_io: failed to save bad-rank fixture\n";
        return false;
    }

    std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
    const uint32_t duplicate_rank = 0;
    file.seekp(28);
    file.write(reinterpret_cast<const char *>(&duplicate_rank), sizeof(duplicate_rank));
    file.seekp(32);
    file.write(reinterpret_cast<const char *>(&duplicate_rank), sizeof(duplicate_rank));
    file.close();

    const bool ok = expect_ch_load_failure(path);
    std::filesystem::remove(path);
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok &= check_ch_round_trip();
    ok &= check_ch_truncation_fails();
    ok &= check_bad_rank_fails();
    if (!ok) {
        std::cerr << "ch io tests FAILED\n";
        return 1;
    }
    return 0;
}
