#include "algorithms/hl/hub_labels.hpp"

#include "algorithms/heap_node.hpp"
#include "algorithms/stopwatch.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <span>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace transport {

namespace {

using Pq = std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<HeapNode>>;

} // namespace

HubLabelsAlgorithm::HubLabelsAlgorithm(const Graph &graph, double label_fraction, uint64_t memory_budget_bytes,
                                       uint32_t threads)
    : graph_(graph), label_fraction_(label_fraction), memory_budget_bytes_(memory_budget_bytes), threads_(threads) {
    if (label_fraction_ <= 0.0 || label_fraction_ > 1.0 || !std::isfinite(label_fraction_)) {
        throw std::invalid_argument("hl: label_fraction must be in (0, 1]");
    }
    if (memory_budget_bytes_ == 0) {
        throw std::invalid_argument("hl: memory_budget_bytes must be > 0");
    }
}

std::string_view HubLabelsAlgorithm::name() const { return "hl"; }

void HubLabelsAlgorithm::inject_ch(ContractionHierarchy ch) {
    ch_ = std::move(ch);
    ch_provided_ = true;
}

// ----- label intersection -----

Distance HubLabelsAlgorithm::intersect_labels(std::span<const HlEntry> f, std::span<const HlEntry> b) {
    Distance best = kUnreachable;
    size_t i = 0;
    size_t j = 0;
    while (i < f.size() && j < b.size()) {
        if (f[i].hub == b[j].hub) {
            const Distance d = f[i].dist <= kUnreachable - b[j].dist ? f[i].dist + b[j].dist : kUnreachable;
            best = std::min(best, d);
            ++i;
            ++j;
        } else if (f[i].hub < b[j].hub) {
            ++i;
        } else {
            ++j;
        }
    }
    return best;
}

// ----- preprocess -----

void HubLabelsAlgorithm::preprocess() {
    if (preprocessed_) {
        return;
    }
    Stopwatch total_sw;

    if (!ch_provided_) {
        ContractionHierarchyAlgorithm ch_algo(graph_);
        ch_algo.preprocess();
        ch_ = ch_algo.get_ch();
        stats_.ch_was_injected = false;
    } else {
        stats_.ch_was_injected = true;
    }

    const VertexId V = ch_.vertex_count();
    labeled_count_ = static_cast<VertexId>(static_cast<double>(V) * label_fraction_);
    if (labeled_count_ == 0) {
        labeled_count_ = 1; // at least the apex
    }
    if (labeled_count_ > V) {
        labeled_count_ = V;
    }
    label_threshold_ = V - labeled_count_;

    fwd_scratch_ = StampedVector<Distance>(V, kUnreachable);
    bwd_scratch_ = StampedVector<Distance>(V, kUnreachable);

    build_labels();

    stats_.label_fraction = label_fraction_;
    stats_.labeled_vertices = static_cast<uint32_t>(labeled_count_);
    stats_.label_build_wall_s = to_seconds(total_sw.wall_elapsed());
    stats_.label_build_cpu_s = to_seconds(total_sw.cpu_elapsed());
    preprocessed_ = true;
}

void HubLabelsAlgorithm::build_labels() {
    Stopwatch sw;
    const VertexId V = ch_.vertex_count();

    // inv_rank[r] = vertex with rank r.
    std::vector<VertexId> inv_rank(V);
    for (VertexId v = 0; v < V; ++v) {
        inv_rank[ch_.rank[v]] = v;
    }

    // Temporary per-vertex label storage indexed by vertex_id.
    // Unlabeled vertices keep empty vectors throughout.
    std::vector<std::vector<HlEntry>> temp_fwd(V);
    std::vector<std::vector<HlEntry>> temp_bwd(V);

    uint64_t prune_drops = 0;
    uint64_t total_fwd_entries = 0;
    uint64_t total_bwd_entries = 0;
    size_t max_fwd = 0;
    size_t max_bwd = 0;

    // Scratch for candidate gathering (reused across vertices).
    std::unordered_map<VertexId, Distance> cand_map;
    cand_map.reserve(512);
    std::vector<HlEntry> sorted_cands;
    sorted_cands.reserve(512);

    // Shared helper: builds one half-label for `vertex` from `offsets`/`edges`, propagating
    // distances through already-built `own_temp` labels and pruning against `other_temp`.
    auto build_half = [&](VertexId vertex, const std::vector<uint64_t> &offsets, const std::vector<Edge> &edges,
                          std::vector<std::vector<HlEntry>> &own_temp,
                          const std::vector<std::vector<HlEntry>> &other_temp) -> std::vector<HlEntry> & {
        cand_map.clear();
        cand_map[vertex] = Distance{0};
        for (const Edge &e : std::span(edges).subspan(static_cast<size_t>(offsets[vertex]),
                                                      static_cast<size_t>(offsets[vertex + 1] - offsets[vertex]))) {
            for (const HlEntry &he : own_temp[e.to]) {
                const Distance nd = he.dist <= kUnreachable - e.weight ? he.dist + e.weight : kUnreachable;
                auto [it, inserted] = cand_map.emplace(he.hub, nd);
                if (!inserted && nd < it->second) {
                    it->second = nd;
                }
            }
        }
        sorted_cands.clear();
        for (const auto &[h, d] : cand_map) {
            sorted_cands.push_back({h, d});
        }
        std::sort(sorted_cands.begin(), sorted_cands.end());

        std::vector<HlEntry> &label = own_temp[vertex];
        label.clear();
        for (const HlEntry &cand : sorted_cands) {
            if (intersect_labels(std::span(label), std::span<const HlEntry>(other_temp[cand.hub])) < cand.dist) {
                ++prune_drops;
            } else {
                label.push_back(cand);
            }
        }
        return label;
    };

    // Process labeled vertices in descending rank order.
    for (VertexId rank = V; rank-- > label_threshold_;) {
        const VertexId v = inv_rank[rank];

        const std::vector<HlEntry> &lf = build_half(v, ch_.forward_offsets, ch_.forward_edges, temp_fwd, temp_bwd);
        const std::vector<HlEntry> &lb = build_half(v, ch_.backward_offsets, ch_.backward_edges, temp_bwd, temp_fwd);

        total_fwd_entries += lf.size();
        total_bwd_entries += lb.size();
        max_fwd = std::max(max_fwd, lf.size());
        max_bwd = std::max(max_bwd, lb.size());

        // Incremental memory budget check every 64k vertices.
        // Peak memory ≈ 2 × label_data (temp vectors + CSR copy coexist during assembly)
        //             + fixed overhead (CH arcs, graph edges, per-vertex scratch).
        const VertexId processed = V - rank;
        if ((processed & 0xFFFFu) == 0 && processed > 0) {
            const uint64_t bytes_so_far = (total_fwd_entries + total_bwd_entries) * sizeof(HlEntry);
            const uint64_t projected_label_bytes =
                bytes_so_far * static_cast<uint64_t>(labeled_count_) / static_cast<uint64_t>(processed);
            // Fixed overhead: CH arcs + graph edges + per-vertex: temp vector headers (2×24),
            // inv_rank (8), fwd/bwd scratch (2×8), fwd/bwd offsets being built (2×8).
            const uint64_t fixed_bytes =
                (ch_.forward_edges.size() + ch_.backward_edges.size()) * sizeof(Edge) +
                (ch_.forward_offsets.size() + ch_.backward_offsets.size()) * sizeof(uint64_t) +
                ch_.rank.size() * sizeof(uint32_t) + graph_.edges.size() * sizeof(Edge) +
                graph_.offsets.size() * sizeof(size_t) +
                static_cast<uint64_t>(V) *
                    (2 * sizeof(std::vector<HlEntry>) + sizeof(VertexId) + 2 * sizeof(Distance) + 2 * sizeof(uint64_t));
            const uint64_t projected_peak = 2 * projected_label_bytes + fixed_bytes;
            if (projected_peak > memory_budget_bytes_) {
                throw std::runtime_error("hl: projected peak memory " +
                                         std::to_string(projected_peak / (1024ULL * 1024)) + " MB exceeds budget " +
                                         std::to_string(memory_budget_bytes_ / (1024ULL * 1024 * 1024)) +
                                         " GB after processing " + std::to_string(processed) + " of " +
                                         std::to_string(labeled_count_) + " labeled vertices");
            }
        }
    }

    // Assemble CSR from temp vectors.
    fwd_offsets_.resize(static_cast<size_t>(V) + 1);
    bwd_offsets_.resize(static_cast<size_t>(V) + 1);
    fwd_labels_.reserve(total_fwd_entries);
    bwd_labels_.reserve(total_bwd_entries);

    fwd_offsets_[0] = 0;
    bwd_offsets_[0] = 0;
    for (VertexId v = 0; v < V; ++v) {
        fwd_offsets_[v + 1] = fwd_offsets_[v] + temp_fwd[v].size();
        bwd_offsets_[v + 1] = bwd_offsets_[v] + temp_bwd[v].size();
        fwd_labels_.insert(fwd_labels_.end(), temp_fwd[v].begin(), temp_fwd[v].end());
        bwd_labels_.insert(bwd_labels_.end(), temp_bwd[v].begin(), temp_bwd[v].end());
        // Release memory as we go.
        temp_fwd[v] = std::vector<HlEntry>{};
        temp_bwd[v] = std::vector<HlEntry>{};
    }

    const double label_bytes = static_cast<double>((total_fwd_entries + total_bwd_entries) * sizeof(HlEntry));

    stats_.label_build_wall_s = to_seconds(sw.wall_elapsed());
    stats_.label_build_cpu_s = to_seconds(sw.cpu_elapsed());
    stats_.avg_label_size_fwd =
        labeled_count_ > 0 ? static_cast<double>(total_fwd_entries) / static_cast<double>(labeled_count_) : 0.0;
    stats_.avg_label_size_bwd =
        labeled_count_ > 0 ? static_cast<double>(total_bwd_entries) / static_cast<double>(labeled_count_) : 0.0;
    stats_.max_label_size_fwd = max_fwd;
    stats_.max_label_size_bwd = max_bwd;
    stats_.label_memory_mb = label_bytes / (1024.0 * 1024.0);
    stats_.prune_drops = prune_drops;
}

// ----- collect (shared upward-search helper) -----

void HubLabelsAlgorithm::collect(VertexId start, const std::vector<uint64_t> &ch_offsets,
                                 const std::vector<Edge> &ch_edges, const std::vector<uint64_t> &label_offsets,
                                 const std::vector<HlEntry> &label_data, StampedVector<Distance> &scratch,
                                 std::vector<HlEntry> &out, std::vector<VertexId> &unlabeled_settled) const {
    std::unordered_map<VertexId, Distance> hub_best;
    hub_best.reserve(256);

    Pq pq;
    scratch.set(start, Distance{0});
    pq.push({Distance{0}, start});

    while (!pq.empty()) {
        const HeapNode top = pq.top();
        pq.pop();
        if (top.key != scratch.get(top.v)) {
            continue;
        }
        if (is_labeled(top.v)) {
            const auto label_span =
                std::span(label_data.data() + label_offsets[top.v], label_data.data() + label_offsets[top.v + 1]);
            for (const HlEntry &e : label_span) {
                const Distance nd = top.key <= kUnreachable - e.dist ? top.key + e.dist : kUnreachable;
                auto [it, inserted] = hub_best.emplace(e.hub, nd);
                if (!inserted && nd < it->second) {
                    it->second = nd;
                }
            }
            continue;
        }
        unlabeled_settled.push_back(top.v);
        for (const Edge &e :
             std::span(ch_edges).subspan(static_cast<size_t>(ch_offsets[top.v]),
                                         static_cast<size_t>(ch_offsets[top.v + 1] - ch_offsets[top.v]))) {
            const Distance nd = top.key <= kUnreachable - e.weight ? top.key + e.weight : kUnreachable;
            if (nd < scratch.get(e.to)) {
                scratch.set(e.to, nd);
                pq.push({nd, e.to});
            }
        }
    }

    out.clear();
    out.reserve(hub_best.size());
    for (const auto &[h, d] : hub_best) {
        out.push_back({h, d});
    }
    std::sort(out.begin(), out.end());
}

// ----- query -----

PathResult HubLabelsAlgorithm::query(VertexId source, VertexId target) const {
    const bool s_lab = is_labeled(source);
    const bool t_lab = is_labeled(target);
    Distance dist = kUnreachable;

    if (s_lab && t_lab) {
        dist = intersect_labels(fwd_label(source), bwd_label(target));
    } else if (s_lab) {
        // source labeled, target unlabeled
        bwd_scratch_.reset();
        std::vector<VertexId> dummy;
        collect(target, ch_.backward_offsets, ch_.backward_edges, bwd_offsets_, bwd_labels_, bwd_scratch_,
                collect_buf_bwd_, dummy);
        bwd_scratch_.reset();
        dist = intersect_labels(fwd_label(source), collect_buf_bwd_);
    } else if (t_lab) {
        // source unlabeled, target labeled
        fwd_scratch_.reset();
        std::vector<VertexId> dummy;
        collect(source, ch_.forward_offsets, ch_.forward_edges, fwd_offsets_, fwd_labels_, fwd_scratch_,
                collect_buf_fwd_, dummy);
        fwd_scratch_.reset();
        dist = intersect_labels(collect_buf_fwd_, bwd_label(target));
    } else {
        // Both unlabeled: collect + mu_low from bidi CH below threshold.
        fwd_scratch_.reset();
        bwd_scratch_.reset();

        std::vector<VertexId> fwd_unlabeled;
        std::vector<VertexId> bwd_unlabeled;
        collect(source, ch_.forward_offsets, ch_.forward_edges, fwd_offsets_, fwd_labels_, fwd_scratch_,
                collect_buf_fwd_, fwd_unlabeled);
        collect(target, ch_.backward_offsets, ch_.backward_edges, bwd_offsets_, bwd_labels_, bwd_scratch_,
                collect_buf_bwd_, bwd_unlabeled);

        // mu_low: min dist_fwd[v] + dist_bwd[v] over fwd-settled unlabeled vertices.
        Distance mu_low = kUnreachable;
        for (const VertexId v : fwd_unlabeled) {
            const Distance db = bwd_scratch_.get(v);
            if (db != kUnreachable) {
                const Distance df = fwd_scratch_.get(v);
                const Distance candidate = df <= kUnreachable - db ? df + db : kUnreachable;
                mu_low = std::min(mu_low, candidate);
            }
        }

        fwd_scratch_.reset();
        bwd_scratch_.reset();

        dist = std::min(mu_low, intersect_labels(collect_buf_fwd_, collect_buf_bwd_));
    }

    return {dist, 0};
}

} // namespace transport
