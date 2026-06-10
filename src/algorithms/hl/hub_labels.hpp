#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"
#include "algorithms/routing_algorithm.hpp"
#include "algorithms/stamped_vector.hpp"
#include "graph/graph.hpp"
#include "graph/types.hpp"

#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace transport {

// Hub Labels on top of a Contraction Hierarchy (tiered variant).
//
// Only the top `label_fraction` of CH ranks receive labels.  Every shortest
// path whose apex is in the labeled tier is answered by intersecting the two
// sorted hub-label arrays.  Paths entirely below the tier are captured by a
// bidirectional CH fallback (mu_low) that runs when both endpoints are
// unlabeled.  When exactly one endpoint is unlabeled, the apex is guaranteed
// to be in the tier, so only the labeled-label intersection is needed.
//
// Label construction (descending rank order):
//   L_f(v) = {(v,0)} ∪ min-merge of {(h, d+w) : (h,d)∈L_f(u)} over upward arcs.
//   Prune (h, d) if intersect(current L_f(v), L_b(h)) < d  (keep-equality rule).
//   Symmetric for L_b.
class HubLabelsAlgorithm final : public RoutingAlgorithm {
public:
    explicit HubLabelsAlgorithm(const Graph &graph, ContractionHierarchy &&ch, double label_fraction = 0.25,
                                uint64_t memory_budget_bytes = 18ULL * 1024 * 1024 * 1024);

    std::string_view name() const override;
    void preprocess() override;
    [[nodiscard]] PathResult query(VertexId source, VertexId target) const override;

    struct HlStats {
        double label_fraction = 0.0;
        uint32_t labeled_vertices = 0;
        double avg_label_size_fwd = 0.0;
        double avg_label_size_bwd = 0.0;
        size_t max_label_size_fwd = 0;
        size_t max_label_size_bwd = 0;
        double label_memory_mb = 0.0;
        double label_build_wall_s = 0.0;
        double label_build_cpu_s = 0.0;
        uint64_t prune_drops = 0;
    };

    [[nodiscard]] const HlStats &hl_stats() const { return stats_; }

private:
    const Graph &graph_;
    ContractionHierarchy ch_;
    double label_fraction_;
    uint64_t memory_budget_bytes_;

    bool preprocessed_ = false;

    VertexId label_threshold_ = 0; // rank >= this → labeled
    VertexId labeled_count_ = 0;

    // Label entry: sorted by hub within each vertex's label.
    // HlEntry is not HeapNode: HeapNode sorts by key; HlEntry sorts by hub.
    // dist is Distance (uint64_t). sizeof(HlEntry)==16: hub(8)+dist(8), no padding.
    struct HlEntry {
        VertexId hub;
        Distance dist;
        bool operator<(const HlEntry &o) const { return hub < o.hub; }
    };
    static_assert(sizeof(HlEntry) == 16);

    // CSR labels indexed by vertex_id; unlabeled vertices have empty ranges.
    std::vector<size_t> fwd_offsets_; // size V+1
    std::vector<HlEntry> fwd_labels_;
    std::vector<size_t> bwd_offsets_; // size V+1
    std::vector<HlEntry> bwd_labels_;

    HlStats stats_;

    // Scratch for query-time upward searches; resized to V in preprocess().
    mutable StampedVector<Distance> fwd_scratch_{1, kUnreachable};
    mutable StampedVector<Distance> bwd_scratch_{1, kUnreachable};

    [[nodiscard]] bool is_labeled(VertexId v) const { return ch_.rank[v] >= label_threshold_; }

    [[nodiscard]] std::span<const HlEntry> fwd_label(VertexId v) const {
        return {fwd_labels_.data() + fwd_offsets_[v], fwd_labels_.data() + fwd_offsets_[v + 1]};
    }
    [[nodiscard]] std::span<const HlEntry> bwd_label(VertexId v) const {
        return {bwd_labels_.data() + bwd_offsets_[v], bwd_labels_.data() + bwd_offsets_[v + 1]};
    }

    // Minimum d_f + d_b over all matching hubs; returns kUnreachable if no match.
    [[nodiscard]] static Distance intersect_labels(std::span<const HlEntry> f, std::span<const HlEntry> b);

    void build_labels();

    // Dijkstra-style upward search from `start` over one CH direction (forward when
    // `forward` is true, backward otherwise), using the matching member scratch
    // (fwd_scratch_/bwd_scratch_). Stops expanding at each labeled vertex it settles and
    // instead merges that vertex's label into `out` (sorted by hub, ready for
    // intersect_labels). Unlabeled vertices it settles are expanded further and, when
    // `unlabeled` is non-null, appended to it (used by query() to compute the mu_low CH
    // fallback). Caller resets the matching scratch before and after calling. Returns the
    // number of vertices settled.
    uint32_t collect(VertexId start, bool forward, std::vector<HlEntry> &out, std::vector<VertexId> *unlabeled) const;
};

} // namespace transport
