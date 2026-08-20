#pragma once

#include "vio/EstimatorPromotionReadiness.hpp"

#include <cstdint>
#include <string_view>

namespace drone::vio {

enum class EstimatorAuthority : uint8_t {
    Baseline = 0,
    AdvancedShadow,
};

enum class AuthorityTransitionStatus : uint8_t {
    NoOp = 0,
    Promoted,
    RolledBack,
    RejectedNotReady,
    RejectedGenerationMismatch,
};

struct AuthorityTransitionResult {
    AuthorityTransitionStatus status{AuthorityTransitionStatus::NoOp};
    EstimatorAuthority previous{EstimatorAuthority::Baseline};
    EstimatorAuthority current{EstimatorAuthority::Baseline};
    PromotionBlockReason readiness_reason{PromotionBlockReason::None};
};

[[nodiscard]] constexpr std::string_view to_string(EstimatorAuthority authority) {
    switch (authority) {
    case EstimatorAuthority::Baseline:
        return "baseline";
    case EstimatorAuthority::AdvancedShadow:
        return "advanced_shadow";
    }
    return "unknown";
}

[[nodiscard]] constexpr std::string_view to_string(AuthorityTransitionStatus status) {
    switch (status) {
    case AuthorityTransitionStatus::NoOp:
        return "no_op";
    case AuthorityTransitionStatus::Promoted:
        return "promoted";
    case AuthorityTransitionStatus::RolledBack:
        return "rolled_back";
    case AuthorityTransitionStatus::RejectedNotReady:
        return "rejected_not_ready";
    case AuthorityTransitionStatus::RejectedGenerationMismatch:
        return "rejected_generation_mismatch";
    }
    return "unknown";
}

class EstimatorAuthorityController {
public:
    void reset(uint64_t generation) {
        authority_ = EstimatorAuthority::Baseline;
        generation_ = generation;
    }

    [[nodiscard]] AuthorityTransitionResult
    request_promotion(const CoordinatorDiagnostics& diagnostics,
                      const PromotionReadinessConfig& readiness_cfg = {}) {
        const auto previous = authority_;
        if (diagnostics.reset_generation != generation_) {
            return {AuthorityTransitionStatus::RejectedGenerationMismatch, previous, authority_,
                    PromotionBlockReason::None};
        }

        const auto readiness = assess_promotion_readiness(diagnostics, readiness_cfg);
        if (!readiness.ready) {
            return {AuthorityTransitionStatus::RejectedNotReady, previous, authority_,
                    readiness.reason};
        }

        if (authority_ == EstimatorAuthority::AdvancedShadow) {
            return {AuthorityTransitionStatus::NoOp, previous, authority_,
                    PromotionBlockReason::None};
        }

        authority_ = EstimatorAuthority::AdvancedShadow;
        ++promotion_count_;
        return {AuthorityTransitionStatus::Promoted, previous, authority_, PromotionBlockReason::None};
    }

    [[nodiscard]] AuthorityTransitionResult rollback_to_baseline(uint64_t generation) {
        const auto previous = authority_;
        if (generation != generation_) {
            return {AuthorityTransitionStatus::RejectedGenerationMismatch, previous, authority_,
                    PromotionBlockReason::None};
        }
        if (authority_ == EstimatorAuthority::Baseline) {
            return {AuthorityTransitionStatus::NoOp, previous, authority_,
                    PromotionBlockReason::None};
        }

        authority_ = EstimatorAuthority::Baseline;
        ++rollback_count_;
        return {AuthorityTransitionStatus::RolledBack, previous, authority_,
                PromotionBlockReason::None};
    }

    [[nodiscard]] EstimatorAuthority authority() const { return authority_; }
    [[nodiscard]] uint64_t generation() const { return generation_; }
    [[nodiscard]] uint64_t promotion_count() const { return promotion_count_; }
    [[nodiscard]] uint64_t rollback_count() const { return rollback_count_; }

private:
    EstimatorAuthority authority_{EstimatorAuthority::Baseline};
    uint64_t generation_{0};
    uint64_t promotion_count_{0};
    uint64_t rollback_count_{0};
};

} // namespace drone::vio
