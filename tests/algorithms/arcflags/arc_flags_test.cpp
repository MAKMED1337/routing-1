#include "algorithms/arcflags/arc_flags.hpp"
#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/phast.hpp"
#include "graph_fixtures.hpp"
#include "routing_test_utils.hpp"

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

} // namespace

int main() {
    bool ok = true;
    ok &= check_arcflags_validation();
    ok &= check_arcflags_query_before_preprocess_throws();
    if (!ok) {
        std::cerr << "arcflags tests FAILED\n";
        return 1;
    }
    return 0;
}
