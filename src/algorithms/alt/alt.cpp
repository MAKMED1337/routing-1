#include "algorithms/alt/alt.hpp"

#include "graph/reverse_graph.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <string_view>
#include <vector>

namespace transport {
namespace {

struct AltHeapNode {
    int64_t key = 0;
    VertexId vertex = 0;

    bool operator>(const AltHeapNode &other) const { return key > other.key; }
};

int64_t compute_potential(const alt::LandmarkSet &landmarks, const std::vector<size_t> &active_idx, VertexId vertex,
                          VertexId target) {
    const size_t vertices = landmarks.vertex_count;
    int64_t best = 0;
    for (const size_t landmark_idx : active_idx) {
        const size_t offset = landmark_idx * vertices;
        const Distance landmark_to_target = landmarks.dist_from[offset + target];
        const Distance landmark_to_vertex = landmarks.dist_from[offset + vertex];
        const Distance vertex_to_landmark = landmarks.dist_to[offset + vertex];
        const Distance target_to_landmark = landmarks.dist_to[offset + target];

        if (landmark_to_target != alt::kLandmarkInf && landmark_to_vertex != alt::kLandmarkInf) {
            best = std::max(best, static_cast<int64_t>(landmark_to_target) - static_cast<int64_t>(landmark_to_vertex));
        }
        if (vertex_to_landmark != alt::kLandmarkInf && target_to_landmark != alt::kLandmarkInf) {
            best = std::max(best, static_cast<int64_t>(vertex_to_landmark) - static_cast<int64_t>(target_to_landmark));
        }
    }
    return best;
}

int64_t landmark_score(const alt::LandmarkSet &landmarks, size_t landmark_idx, VertexId source, VertexId target) {
    const size_t vertices = landmarks.vertex_count;
    const size_t offset = landmark_idx * vertices;
    const Distance landmark_to_target = landmarks.dist_from[offset + target];
    const Distance landmark_to_source = landmarks.dist_from[offset + source];
    const Distance source_to_landmark = landmarks.dist_to[offset + source];
    const Distance target_to_landmark = landmarks.dist_to[offset + target];

    int64_t score = 0;
    if (landmark_to_target != alt::kLandmarkInf && landmark_to_source != alt::kLandmarkInf) {
        score = std::max(score, static_cast<int64_t>(landmark_to_target) - static_cast<int64_t>(landmark_to_source));
    }
    if (source_to_landmark != alt::kLandmarkInf && target_to_landmark != alt::kLandmarkInf) {
        score = std::max(score, static_cast<int64_t>(source_to_landmark) - static_cast<int64_t>(target_to_landmark));
    }
    return score;
}

} // namespace

AltAlgorithm::AltAlgorithm(const Graph &graph) : AltAlgorithm(graph, 16, alt::LandmarkStrategy::Farthest, 4, 42) {}

AltAlgorithm::AltAlgorithm(const Graph &graph, uint32_t landmark_count, alt::LandmarkStrategy strategy,
                           uint32_t active_landmarks, uint32_t seed)
    : graph_(graph), landmark_count_(landmark_count), strategy_(strategy), active_landmarks_(active_landmarks),
      seed_(seed), dist_(graph.vertex_count(), kUnreachable) {}

std::string_view AltAlgorithm::name() const { return "alt"; }

void AltAlgorithm::preprocess() {
    reverse_ = build_reverse_graph(graph_);
    landmarks_ = alt::build_landmarks(graph_, reverse_, landmark_count_, strategy_, seed_);
}

uint64_t AltAlgorithm::landmark_table_bytes() const {
    return static_cast<uint64_t>((landmarks_.dist_from.size() + landmarks_.dist_to.size()) * sizeof(Distance));
}

int64_t AltAlgorithm::potential(VertexId vertex, VertexId target) const {
    return compute_potential(landmarks_, active_idx_, vertex, target);
}

void AltAlgorithm::select_active(VertexId source, VertexId target) const {
    const size_t landmark_count = landmarks_.landmarks.size();
    const size_t selected_count = std::min(static_cast<size_t>(active_landmarks_), landmark_count);
    active_idx_.resize(selected_count);

    std::vector<std::pair<int64_t, size_t>> scores;
    scores.reserve(landmark_count);
    for (size_t i = 0; i < landmark_count; ++i) {
        scores.emplace_back(landmark_score(landmarks_, i, source, target), i);
    }

    std::partial_sort(scores.begin(), scores.begin() + static_cast<std::ptrdiff_t>(selected_count), scores.end(),
                      [](const auto &lhs, const auto &rhs) { return lhs.first > rhs.first; });
    for (size_t i = 0; i < selected_count; ++i) {
        active_idx_[i] = scores[i].second;
    }
}

PathResult AltAlgorithm::query(VertexId source, VertexId target) const {
    if (source == target) {
        return PathResult{0, 0};
    }

    select_active(source, target);
    dist_.reset();

    std::priority_queue<AltHeapNode, std::vector<AltHeapNode>, std::greater<>> pq;
    dist_.set(source, 0);
    pq.push({potential(source, target), source});

    uint32_t settled = 0;
    while (!pq.empty()) {
        const AltHeapNode top = pq.top();
        pq.pop();

        const Distance current_distance = dist_.get(top.vertex);
        if (top.key != static_cast<int64_t>(current_distance) + potential(top.vertex, target)) {
            continue;
        }

        ++settled;
        if (top.vertex == target) {
            break;
        }

        for (const Edge &edge : graph_.adjacent_edges(top.vertex)) {
            const Distance next_distance = current_distance + edge.weight;
            if (next_distance < dist_.get(edge.to)) {
                dist_.set(edge.to, next_distance);
                pq.push({static_cast<int64_t>(next_distance) + potential(edge.to, target), edge.to});
            }
        }
    }

    return PathResult{dist_.get(target), settled};
}

} // namespace transport
