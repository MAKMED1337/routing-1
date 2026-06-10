#include "algorithms/routing_preprocessing_context.hpp"

#include <stdexcept>

namespace transport {

RoutingPreprocessingContext::RoutingPreprocessingContext(const Graph &graph) : graph_(graph) {}

void RoutingPreprocessingContext::build_ch() {
    if (ch_.has_value()) {
        return;
    }
    ContractionHierarchyBuildResult result = build_contraction_hierarchy(graph_);
    ch_ = std::move(result.hierarchy);
    ch_stats_ = result.stats;
}

void RoutingPreprocessingContext::build_phast() {
    if (phast_.has_value()) {
        return;
    }
    build_ch();
    phast_.emplace(*ch_);
}

const ContractionHierarchy &RoutingPreprocessingContext::ch() const {
    if (!ch_.has_value()) {
        throw std::logic_error("RoutingPreprocessingContext::build_ch() must be called before ch()");
    }
    return *ch_;
}

const PreprocessStats &RoutingPreprocessingContext::ch_stats() const {
    if (!ch_stats_.has_value()) {
        throw std::logic_error("RoutingPreprocessingContext::build_ch() must be called before ch_stats()");
    }
    return *ch_stats_;
}

const PhastAlgorithm &RoutingPreprocessingContext::phast() const {
    if (!phast_.has_value()) {
        throw std::logic_error("RoutingPreprocessingContext::build_phast() must be called before phast()");
    }
    return *phast_;
}

} // namespace transport
