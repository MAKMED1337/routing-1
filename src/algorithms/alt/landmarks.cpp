#include "algorithms/alt/landmarks.hpp"

#include "algorithms/shortest_paths.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace transport::alt {
namespace {

std::vector<VertexId> select_random(const Graph &graph, uint32_t landmark_count, std::mt19937 &rng) {
    const size_t vertices = graph.vertex_count();
    std::vector<VertexId> candidates;
    candidates.reserve(vertices);
    for (size_t vertex = 0; vertex < vertices; ++vertex) {
        if (graph.offsets[vertex + 1] > graph.offsets[vertex]) {
            candidates.push_back(static_cast<VertexId>(vertex));
        }
    }

    std::shuffle(candidates.begin(), candidates.end(), rng);
    landmark_count = std::min(landmark_count, static_cast<uint32_t>(candidates.size()));
    return std::vector<VertexId>(candidates.begin(), candidates.begin() + landmark_count);
}

std::vector<VertexId> select_farthest(const Graph &graph, uint32_t landmark_count, std::mt19937 &rng) {
    const size_t vertices = graph.vertex_count();
    if (vertices == 0 || landmark_count == 0) {
        return {};
    }

    landmark_count = std::min(landmark_count, static_cast<uint32_t>(vertices));
    std::uniform_int_distribution<size_t> pick(0, vertices - 1);

    std::vector<VertexId> chosen;
    chosen.reserve(landmark_count);
    std::vector<bool> selected(vertices, false);
    std::vector<Distance> min_dist(vertices, kUnreachable);
    std::vector<Distance> tmp(vertices);

    const VertexId seed = static_cast<VertexId>(pick(rng));
    dijkstra_one_to_all(graph, seed, tmp);

    std::pair<Distance, VertexId> best = {0, seed};
    for (size_t vertex = 0; vertex < vertices; ++vertex) {
        if (tmp[vertex] != kUnreachable) {
            best = std::max(best, std::pair<Distance, VertexId>{tmp[vertex], static_cast<VertexId>(vertex)});
        }
    }

    chosen.push_back(best.second);
    selected[best.second] = true;
    dijkstra_one_to_all(graph, best.second, tmp);
    for (size_t vertex = 0; vertex < vertices; ++vertex) {
        min_dist[vertex] = std::min(min_dist[vertex], tmp[vertex]);
    }

    for (uint32_t i = 1; i < landmark_count; ++i) {
        best = {0, 0};
        for (size_t vertex = 0; vertex < vertices; ++vertex) {
            if (!selected[vertex]) {
                best = std::max(best, std::pair<Distance, VertexId>{min_dist[vertex], static_cast<VertexId>(vertex)});
            }
        }

        chosen.push_back(best.second);
        selected[best.second] = true;
        dijkstra_one_to_all(graph, best.second, tmp);
        for (size_t vertex = 0; vertex < vertices; ++vertex) {
            min_dist[vertex] = std::min(min_dist[vertex], tmp[vertex]);
        }
    }

    return chosen;
}

std::vector<VertexId> select_planar(const Graph &graph, uint32_t landmark_count) {
    const size_t vertices = graph.vertex_count();
    if (vertices == 0 || landmark_count == 0) {
        return {};
    }

    landmark_count = std::min(landmark_count, static_cast<uint32_t>(vertices));

    double sum_lat = 0.0;
    double sum_lon = 0.0;
    for (const NodeCoord &coord : graph.coords) {
        sum_lat += coord.lat;
        sum_lon += coord.lon;
    }

    const double center_lat = sum_lat / static_cast<double>(vertices);
    const double center_lon = sum_lon / static_cast<double>(vertices);

    std::vector<std::pair<double, VertexId>> best_by_sector(landmark_count, {-1.0, 0});
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    for (size_t vertex = 0; vertex < vertices; ++vertex) {
        const NodeCoord &coord = graph.coords[vertex];
        const double angle = std::atan2(coord.lat - center_lat, coord.lon - center_lon);
        const double normalized = (angle + std::numbers::pi) / kTwoPi;
        const size_t sector = static_cast<size_t>(normalized * static_cast<double>(landmark_count)) % landmark_count;
        const double dlat = coord.lat - center_lat;
        const double dlon = coord.lon - center_lon;
        const double squared_distance = dlat * dlat + dlon * dlon;
        best_by_sector[sector] = std::max(best_by_sector[sector],
                                          std::pair<double, VertexId>{squared_distance, static_cast<VertexId>(vertex)});
    }

    std::vector<VertexId> chosen;
    chosen.reserve(landmark_count);
    for (const auto &[distance, vertex] : best_by_sector) {
        if (distance >= 0.0) {
            chosen.push_back(vertex);
        }
    }

    for (size_t vertex = 0; chosen.size() < landmark_count && vertex < vertices; ++vertex) {
        const VertexId candidate = static_cast<VertexId>(vertex);
        if (std::find(chosen.begin(), chosen.end(), candidate) == chosen.end()) {
            chosen.push_back(candidate);
        }
    }

    return chosen;
}

} // namespace

LandmarkSet build_landmarks(const Graph &graph, const Graph &reverse, uint32_t landmark_count,
                            LandmarkStrategy strategy, std::mt19937 &rng) {
    const size_t vertices = graph.vertex_count();
    landmark_count = std::min(landmark_count, static_cast<uint32_t>(vertices));

    std::vector<VertexId> chosen;
    std::string strategy_name;
    switch (strategy) {
    case LandmarkStrategy::Random:
        chosen = select_random(graph, landmark_count, rng);
        strategy_name = "random";
        break;
    case LandmarkStrategy::Farthest:
        chosen = select_farthest(graph, landmark_count, rng);
        strategy_name = "farthest";
        break;
    case LandmarkStrategy::Planar:
        chosen = select_planar(graph, landmark_count);
        strategy_name = "planar";
        break;
    default:
        throw std::invalid_argument("unknown ALT landmark strategy");
    }

    const size_t chosen_count = chosen.size();
    LandmarkSet landmarks{
        .landmarks = std::move(chosen),
        .dist_from = std::vector<Distance>(chosen_count * vertices, kUnreachable),
        .dist_to = std::vector<Distance>(chosen_count * vertices, kUnreachable),
        .vertex_count = static_cast<uint32_t>(vertices),
        .strategy_name = std::move(strategy_name),
    };

    std::vector<Distance> tmp(vertices);
    for (size_t i = 0; i < landmarks.landmarks.size(); ++i) {
        const size_t offset = i * vertices;
        dijkstra_one_to_all(graph, landmarks.landmarks[i], tmp);
        std::copy(tmp.begin(), tmp.end(), landmarks.dist_from.begin() + static_cast<std::ptrdiff_t>(offset));

        dijkstra_one_to_all(reverse, landmarks.landmarks[i], tmp);
        std::copy(tmp.begin(), tmp.end(), landmarks.dist_to.begin() + static_cast<std::ptrdiff_t>(offset));
    }

    return landmarks;
}

} // namespace transport::alt
