#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/phast.hpp"
#include "graph/graph.hpp"

#include <optional>

namespace transport {

// Without this context, each CH-dependent algorithm (arc flags, CHASE, hub labels) built its
// own ContractionHierarchy/PhastAlgorithm internally, so running several of them against the
// same graph paid for CH/PHAST construction once per algorithm and hid that cost inside each
// algorithm's preprocess() timing. RoutingPreprocessingContext builds CH/PHAST once, owns them
// for the lifetime of a RoutingInstance, and lets callers inject const references into
// algorithm constructors instead.
//
// CH and PHAST are the only two artifacts here because they are the only ones any current
// algorithm needs to share. If a future algorithm needs a different shared artifact, add a
// matching std::optional<T> member plus a build_t()/t() pair following the same idempotent
// build + throwing-accessor pattern, rather than growing CH/PHAST to serve unrelated needs.
//
// build_ch()/build_phast() are explicit and idempotent (calling either twice does not rebuild);
// accessors throw if the corresponding artifact has not been built.
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
