#include "algorithms/alt/alt.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace transport {
namespace {

Distance lower_bound_term(Distance larger, Distance smaller) {
    // Missing or non-positive landmark inequalities contribute no usable lower bound.
    if (larger == kUnreachable || smaller == kUnreachable || larger <= smaller) {
        return 0;
    }
    return larger - smaller;
}

Distance landmark_lower_bound(const alt::LandmarkSet &landmarks, size_t landmark_idx, VertexId from, VertexId to) {
    const size_t vertices = landmarks.vertex_count;
    const size_t offset = landmark_idx * vertices;
    const Distance landmark_to_from = landmarks.dist_from[offset + from];
    const Distance landmark_to_to = landmarks.dist_from[offset + to];
    const Distance from_to_landmark = landmarks.dist_to[offset + from];
    const Distance to_to_landmark = landmarks.dist_to[offset + to];

    return std::max(lower_bound_term(landmark_to_to, landmark_to_from),
                    lower_bound_term(from_to_landmark, to_to_landmark));
}

Distance compute_potential(const alt::LandmarkSet &landmarks, const std::vector<size_t> &active_idx, VertexId vertex,
                           VertexId target) {
    Distance best = 0;
    for (const size_t landmark_idx : active_idx) {
        best = std::max(best, landmark_lower_bound(landmarks, landmark_idx, vertex, target));
    }
    return best;
}

} // namespace

AltAlgorithm::AltAlgorithm(const Graph &graph, alt::LandmarkSet landmarks, uint32_t active_landmarks)
    : graph_(graph), active_landmarks_(active_landmarks), landmarks_(std::move(landmarks)),
      astar_(graph, [this](VertexId vertex, VertexId target) { return potential(vertex, target); }) {}

std::string_view AltAlgorithm::name() const { return "alt"; }

void AltAlgorithm::preprocess() {}

uint64_t AltAlgorithm::landmark_table_bytes() const {
    return static_cast<uint64_t>((landmarks_.dist_from.size() + landmarks_.dist_to.size()) * sizeof(Distance));
}

Distance AltAlgorithm::potential(VertexId vertex, VertexId target) const {
    return compute_potential(landmarks_, active_idx_, vertex, target);
}

void AltAlgorithm::select_active(VertexId source, VertexId target) const {
    const size_t landmark_count = landmarks_.landmarks.size();
    const size_t selected_count = std::min(static_cast<size_t>(active_landmarks_), landmark_count);
    active_idx_.resize(selected_count);

    std::vector<std::pair<Distance, size_t>> scores;
    scores.reserve(landmark_count);
    for (size_t i = 0; i < landmark_count; ++i) {
        scores.emplace_back(landmark_lower_bound(landmarks_, i, source, target), i);
    }

    std::partial_sort(scores.begin(), scores.begin() + static_cast<std::ptrdiff_t>(selected_count), scores.end(),
                      [](const auto &lhs, const auto &rhs) { return lhs.first > rhs.first; });
    for (size_t i = 0; i < selected_count; ++i) {
        active_idx_[i] = scores[i].second;
    }
}

PathResult AltAlgorithm::query(VertexId source, VertexId target) const {
    select_active(source, target);
    return astar_.query(source, target);
}

} // namespace transport
