#include "algorithms/arcflags/arc_flags.hpp"
#include "algorithms/arcflags/arc_flags_io.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/phast.hpp"
#include "graph_fixtures.hpp"
#include "routing_test_utils.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {

using transport::test::expect_throws;

bool check_arcflags_validation() {
    const transport::Graph graph = transport::test::make_line_graph();
    const transport::ContractionHierarchyBuildResult built = transport::build_contraction_hierarchy(graph);

    bool ok = true;
    ok &= expect_throws(
        [&] {
            (void)transport::ArcFlagsAlgorithm(graph, transport::PhastAlgorithm(built.hierarchy), 0,
                                               transport::PartitionMethod::Grid, 1);
        },
        "arcflags: expected std::invalid_argument for regions=0");
    ok &= expect_throws(
        [&] {
            (void)transport::ArcFlagsAlgorithm(graph, transport::PhastAlgorithm(built.hierarchy), 65,
                                               transport::PartitionMethod::Grid, 1);
        },
        "arcflags: expected std::invalid_argument for regions=65");
    ok &= expect_throws(
        [&] {
            (void)transport::ArcFlagsAlgorithm(graph, transport::PhastAlgorithm(built.hierarchy), 4,
                                               transport::PartitionMethod::Grid, 0);
        },
        "arcflags: expected std::invalid_argument for threads=0");
    ok &= expect_throws(
        [&] {
            (void)transport::ArcFlagsAlgorithm(graph, transport::PhastAlgorithm(built.hierarchy), 3,
                                               transport::PartitionMethod::Inertial, 1);
        },
        "arcflags: expected std::invalid_argument for non-power-of-two regions with inertial partition");
    return ok;
}

bool check_arcflags_query_before_preprocess_throws() {
    const transport::Graph graph = transport::test::make_line_graph();
    const transport::ContractionHierarchyBuildResult built = transport::build_contraction_hierarchy(graph);
    transport::ArcFlagsAlgorithm arcflags(graph, transport::PhastAlgorithm(built.hierarchy), 4,
                                          transport::PartitionMethod::Grid, 1);
    try {
        (void)arcflags.query(0, 1);
    } catch (const std::runtime_error &) {
        return true;
    }
    std::cerr << "arcflags: expected std::runtime_error when querying before preprocess()\n";
    return false;
}

bool check_arcflags_artifact_round_trip() {
    const auto [graph, coords] = transport::test::make_grid_graph(3, 3);
    const transport::ContractionHierarchyBuildResult built = transport::build_contraction_hierarchy(graph);
    transport::ArcFlagsAlgorithm arcflags(graph, transport::PhastAlgorithm(built.hierarchy), 4,
                                          transport::PartitionMethod::Grid, 1, coords);
    arcflags.preprocess();

    const std::filesystem::path path = std::filesystem::temp_directory_path() / "transport_arcflags_round_trip.af";
    if (!transport::arcflags::save_arc_flags(arcflags.export_preprocessed(), path.string())) {
        std::cerr << "arcflags: failed to save artifact\n";
        return false;
    }
    transport::ArcFlagsAlgorithm loaded(graph, transport::arcflags::load_arc_flags(path.string()));
    std::filesystem::remove(path);
    return transport::test::check_all_pairs(graph, loaded, "loaded arcflags");
}

bool check_arcflags_artifact_graph_mismatch_throws() {
    const auto [graph, coords] = transport::test::make_grid_graph(3, 3);
    const transport::Graph other_graph = transport::test::make_line_graph();
    const transport::ContractionHierarchyBuildResult built = transport::build_contraction_hierarchy(graph);
    transport::ArcFlagsAlgorithm arcflags(graph, transport::PhastAlgorithm(built.hierarchy), 4,
                                          transport::PartitionMethod::Grid, 1, coords);
    arcflags.preprocess();

    const std::filesystem::path path = std::filesystem::temp_directory_path() / "transport_arcflags_mismatch.af";
    if (!transport::arcflags::save_arc_flags(arcflags.export_preprocessed(), path.string())) {
        std::cerr << "arcflags: failed to save mismatch fixture\n";
        return false;
    }
    const transport::ArcFlagsPreprocessedData data = transport::arcflags::load_arc_flags(path.string());
    std::filesystem::remove(path);
    return expect_throws(
        [&] { (void)transport::ArcFlagsAlgorithm(other_graph, transport::ArcFlagsPreprocessedData(data)); },
        "arcflags: expected graph mismatch to throw");
}

} // namespace

int main() {
    bool ok = true;
    ok &= check_arcflags_validation();
    ok &= check_arcflags_query_before_preprocess_throws();
    ok &= check_arcflags_artifact_round_trip();
    ok &= check_arcflags_artifact_graph_mismatch_throws();
    if (!ok) {
        std::cerr << "arcflags tests FAILED\n";
        return 1;
    }
    return 0;
}
