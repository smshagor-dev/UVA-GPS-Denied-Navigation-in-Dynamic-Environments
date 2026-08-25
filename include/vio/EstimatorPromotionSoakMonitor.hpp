#pragma once

#include "vio/EstimatorPromotionReadiness.hpp"

#include <cstddef>
#include <cstdint>

namespace drone::vio {

struct PromotionSoakConfig {
    uint64_t minimum_ready_samples{500};
    uint64_t maximum_consecutive_blocked_samples{0};
    bool reset_progress_on_block{true};
};

struct PromotionSoakState {
    uint64_t total_samples{0};
    uint64_t ready_samples{0};
    uint64_t consecutive_ready_samples{0};
    uint64_t consecutive_blocked_samples{0};
    uint64_t reset_count{0};
    PromotionBlockReason last_block_reason{PromotionBlockReason::ShadowDisabled};
    bool sustained_ready{false};
};

class PromotionSoakMonitor {
public:
    explicit PromotionSoakMonitor(PromotionSoakConfig cfg = {}) : cfg_(cfg) {}

    [[nodiscard]] const PromotionSoakState& observe(const PromotionReadinessResult& sample) {
        ++state_.total_samples;
        if (sample.ready) {
            ++state_.ready_samples;
            ++state_.consecutive_ready_samples;
            state_.consecutive_blocked_samples = 0;
            state_.last_block_reason = PromotionBlockReason::None;
        } else {
            state_.last_block_reason = sample.reason;
            ++state_.consecutive_blocked_samples;
            if (cfg_.reset_progress_on_block) {
                if (state_.consecutive_ready_samples != 0u) {
                    ++state_.reset_count;
                }
                state_.consecutive_ready_samples = 0;
            }
        }

        const bool blocked_limit_ok =
            state_.consecutive_blocked_samples <= cfg_.maximum_consecutive_blocked_samples;
        state_.sustained_ready = blocked_limit_ok &&
            state_.consecutive_ready_samples >= cfg_.minimum_ready_samples;
        return state_;
    }

    [[nodiscard]] const PromotionSoakState& observe(const CoordinatorDiagnostics& diagnostics,
                                                    const PromotionReadinessConfig& readiness_cfg = {}) {
        return observe(assess_promotion_readiness(diagnostics, readiness_cfg));
    }

    void reset() { state_ = {}; }

    [[nodiscard]] const PromotionSoakState& state() const { return state_; }
    [[nodiscard]] const PromotionSoakConfig& config() const { return cfg_; }

private:
    PromotionSoakConfig cfg_{};
    PromotionSoakState state_{};
};

} // namespace drone::vio
