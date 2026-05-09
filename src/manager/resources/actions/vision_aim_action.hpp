#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logging.hpp>

#include "manager/core/runtime/action.hpp"
#include "manager/resources/vision_aim_profile_provider.hpp"

namespace rmcs_dart_guidance::manager {

class VisionAimAction : public IAction {
public:
    VisionAimAction(
        std::string name, const cv::Point2i& current_target, const bool& tracking,
        const uint64_t& target_seq, Eigen::Vector2d& angle_error_vector,
        const VisionAimProfileProvider& profile_provider, uint32_t fire_count)
        : IAction(std::move(name))
        , current_target_(current_target)
        , tracking_(tracking)
        , target_seq_(target_seq)
        , angle_error_vector_(angle_error_vector)
        , profile_provider_(profile_provider)
        , fire_count_(fire_count) {}

    void on_enter() override {
        angle_error_vector_ = Eigen::Vector2d::Zero();
        start_target_seq_ = target_seq_;
        observed_target_refresh_ = false;
        configuration_error_message_.clear();
        active_profile_.reset();

        active_profile_ = profile_provider_.resolve(fire_count_);
        if (!active_profile_.has_value()) {
            if (!profile_provider_.valid()) {
                configuration_error_message_ = profile_provider_.error_message();
            } else {
                configuration_error_message_ =
                    "missing vision_aim shot profile for fire_count=" + std::to_string(fire_count_);
            }
        }
    }

    ActionStatus update() override {
        if (!active_profile_.has_value()) {
            log_configuration_error();
            return fail(ActionFailureReason::CONFIGURATION_ERROR);
        }

        if (!tracking_ || current_target_ == cv::Point2i(-1, -1)) {
            return fail(ActionFailureReason::INVALID_INPUT);
        }

        observed_target_refresh_ = observed_target_refresh_ || (target_seq_ != start_target_seq_);

        const cv::Point2i desired_point =
            active_profile_->reference_point + active_profile_->offset;
        const cv::Point2i error = desired_point - current_target_;

        angle_error_vector_ =
            Eigen::Vector2d(static_cast<double>(error.x), static_cast<double>(error.y));

        if (observed_target_refresh_ && std::abs(error.x) <= active_profile_->allowable_error.x
            && std::abs(error.y) <= active_profile_->allowable_error.y) {
            return ActionStatus::SUCCESS;
        }

        if (elapsed_ticks() >= active_profile_->timeout_ticks) {
            return fail(
                observed_target_refresh_ ? ActionFailureReason::TIMEOUT
                                         : ActionFailureReason::STALE_INPUT);
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override { angle_error_vector_ = Eigen::Vector2d::Zero(); }

private:
    void log_configuration_error() const {
        if (configuration_error_message_.empty() || runtime_context().logger == nullptr) {
            return;
        }

        RCLCPP_ERROR(
            *runtime_context().logger, "[VisionAimAction] %s",
            configuration_error_message_.c_str());
    }

    const cv::Point2i& current_target_;
    const bool& tracking_;
    const uint64_t& target_seq_;
    Eigen::Vector2d& angle_error_vector_;
    const VisionAimProfileProvider& profile_provider_;
    uint32_t fire_count_;

    uint64_t start_target_seq_{0};
    bool observed_target_refresh_{false};
    std::optional<VisionAimRuntimeProfile> active_profile_;
    std::string configuration_error_message_;
};

} // namespace rmcs_dart_guidance::manager
