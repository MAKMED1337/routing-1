#include "algorithms/routing_preprocessing_context.hpp"
#include "graph_fixtures.hpp"

#include <iostream>
#include <stdexcept>

namespace {

bool check_accessors_throw_before_build() {
    const transport::Graph graph = transport::test::make_line_graph();
    transport::RoutingPreprocessingContext context(graph);

    bool ok = true;
    try {
        (void)context.ch();
        std::cerr << "context: expected throw from ch() before build_ch()\n";
        ok = false;
    } catch (const std::logic_error &) {
    }
    try {
        (void)context.phast();
        std::cerr << "context: expected throw from phast() before build_phast()\n";
        ok = false;
    } catch (const std::logic_error &) {
    }
    return ok;
}

bool check_build_ch_is_idempotent_and_reusable() {
    const transport::Graph graph = transport::test::make_witness_graph();
    transport::RoutingPreprocessingContext context(graph);

    context.build_ch();
    const transport::ContractionHierarchy *first = &context.ch();
    context.build_ch();
    const transport::ContractionHierarchy *second = &context.ch();
    if (first != second) {
        std::cerr << "context: expected build_ch() to be a no-op once CH is built\n";
        return false;
    }
    return true;
}

bool check_build_phast_builds_ch_as_needed() {
    const transport::Graph graph = transport::test::make_witness_graph();
    transport::RoutingPreprocessingContext context(graph);

    if (context.has_ch() || context.has_phast()) {
        std::cerr << "context: expected no artifacts built initially\n";
        return false;
    }
    context.build_phast();
    if (!context.has_ch() || !context.has_phast()) {
        std::cerr << "context: expected build_phast() to also build CH\n";
        return false;
    }
    return true;
}

} // namespace

int main() {
    bool ok = true;
    ok &= check_accessors_throw_before_build();
    ok &= check_build_ch_is_idempotent_and_reusable();
    ok &= check_build_phast_builds_ch_as_needed();
    if (!ok) {
        std::cerr << "routing preprocessing context tests FAILED\n";
        return 1;
    }
    return 0;
}
