#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/phast.hpp"
#include "graph/graph.hpp"

#include <optional>

namespace transport {

// Owns expensive preprocessing artifacts (CH, PHAST) shared across CH-dependent algorithms
// (arc flags, CHASE, hub labels) so they are built once and injected rather than rebuilt by
// each algorithm. build_ch()/build_phast() are explicit and idempotent; accessors throw if the
// corresponding artifact has not been built.
class RoutingPreprocessingContext {
public:
    explicit RoutingPreprocessingContext(const Graph &graph);

    void build_ch();
    // Builds CH first if it has not been built yet.
    void build_phast();

    [[nodiscard]] bool has_ch() const { return ch_.has_value(); }
    [[nodiscard]] bool has_phast() const { return phast_.has_value(); }

    [[nodiscard]] const ContractionHierarchy &ch() const;
    [[nodiscard]] const PreprocessStats &ch_stats() const;
    [[nodiscard]] const PhastAlgorithm &phast() const;

private:
    const Graph &graph_;
    std::optional<ContractionHierarchy> ch_;
    std::optional<PreprocessStats> ch_stats_;
    std::optional<PhastAlgorithm> phast_;
};

} // namespace transport
