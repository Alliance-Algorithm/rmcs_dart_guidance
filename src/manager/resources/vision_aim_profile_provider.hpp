#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core/types.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_dart_guidance::manager {

struct VisionAimShotProfile {
    cv::Point2i reference_point;
    cv::Point2i offset;
    double pitch_angle{0.0};
    int trigger_carriage_position{0};
};

struct VisionAimAllowableError {
    int yaw_pixels{0};
    double pitch_angle{0.0};
};

struct VisionAimRuntimeProfile {
    VisionAimAllowableError allowable_error;
    uint64_t timeout_ticks{0};
    cv::Point2i reference_point;
    cv::Point2i offset;
    double pitch_angle{0.0};
    int trigger_carriage_position{0};
};

class VisionAimProfileProvider {
public:
    VisionAimProfileProvider() = default;

    explicit VisionAimProfileProvider(rclcpp::Node& node) { load_from(node); }

    void load_from(rclcpp::Node& node) {
        valid_ = false;
        error_message_.clear();
        shot_profiles_.clear();
        allowable_error_ = VisionAimAllowableError{};
        timeout_ticks_ = 0;

        const auto allowable_error = read_allowable_error(node);
        if (!allowable_error) {
            return;
        }
        allowable_error_ = *allowable_error;

        const auto timeout_ticks =
            read_uint64(node, "vision_aim.timeout_ticks", "timeout_ticks");
        if (!timeout_ticks) {
            return;
        }
        timeout_ticks_ = *timeout_ticks;

        bool saw_missing_profile = false;
        for (uint32_t index = 0; index < kMaxShotProfiles; ++index) {
            const auto profile = read_shot_profile(node, index);
            if (!profile.has_value()) {
                if (!error_message_.empty()) {
                    return;
                }
                saw_missing_profile = true;
                continue;
            }

            if (saw_missing_profile) {
                set_error("vision_aim.shot_profiles must start from 0 and stay contiguous");
                return;
            }

            shot_profiles_.push_back(*profile);
        }

        if (shot_profiles_.empty()) {
            set_error("missing vision_aim.shot_profiles.0 configuration");
            return;
        }

        valid_ = true;
    }

    [[nodiscard]] bool valid() const { return valid_; }

    [[nodiscard]] const std::string& error_message() const { return error_message_; }

    [[nodiscard]] std::optional<VisionAimRuntimeProfile> resolve(uint32_t fire_count) const {
        if (!valid_ || fire_count >= shot_profiles_.size()) {
            return std::nullopt;
        }

        const auto& shot_profile = shot_profiles_[fire_count];
        return VisionAimRuntimeProfile{
            allowable_error_,
            timeout_ticks_,
            shot_profile.reference_point,
            shot_profile.offset,
            shot_profile.pitch_angle,
            shot_profile.trigger_carriage_position,
        };
    }

private:
    static constexpr uint32_t kMaxShotProfiles = 4;

    std::optional<VisionAimShotProfile> read_shot_profile(rclcpp::Node& node, uint32_t index) {
        const std::string prefix = "vision_aim.shot_profiles." + std::to_string(index);
        const auto reference_point = read_point(
            node, prefix + ".reference_point", false, false, prefix + ".reference_point");
        if (!reference_point.has_value()) {
            if (!error_message_.empty()) {
                return std::nullopt;
            }
        }

        const auto offset = read_point(node, prefix + ".offset", false, false, prefix + ".offset");
        if (!offset.has_value()) {
            if (!error_message_.empty()) {
                return std::nullopt;
            }
        }

        const auto pitch_angle =
            read_double(node, prefix + ".pitch_angle", prefix + ".pitch_angle");
        if (!pitch_angle.has_value()) {
            if (!error_message_.empty()) {
                return std::nullopt;
            }
        }

        const auto trigger_carriage_position = read_int(
            node, prefix + ".trigger_carriage_position", prefix + ".trigger_carriage_position");
        if (!trigger_carriage_position.has_value()) {
            if (!error_message_.empty()) {
                return std::nullopt;
            }
        }

        if (!reference_point.has_value() && !offset.has_value() && !pitch_angle.has_value()
            && !trigger_carriage_position.has_value()) {
            return std::nullopt;
        }

        if (!reference_point.has_value() || !offset.has_value() || !pitch_angle.has_value()
            || !trigger_carriage_position.has_value()) {
            set_error("incomplete configuration for " + prefix);
            return std::nullopt;
        }

        return VisionAimShotProfile{
            *reference_point,
            *offset,
            *pitch_angle,
            *trigger_carriage_position,
        };
    }

    std::optional<VisionAimAllowableError> read_allowable_error(rclcpp::Node& node) {
        const std::string base_name = "vision_aim.allowable_error";
        const std::string x_name = base_name + ".x";
        const std::string y_name = base_name + ".y";
        const bool has_x = node.has_parameter(x_name);
        const bool has_y = node.has_parameter(y_name);

        if (!has_x && !has_y) {
            set_error("missing allowable_error configuration");
            return std::nullopt;
        }

        if (!has_x || !has_y) {
            set_error("incomplete allowable_error configuration");
            return std::nullopt;
        }

        const auto yaw_pixels = read_int(node, x_name, "allowable_error.x");
        if (!yaw_pixels.has_value()) {
            return std::nullopt;
        }

        const auto pitch_angle = read_number(node, y_name, "allowable_error.y");
        if (!pitch_angle.has_value()) {
            return std::nullopt;
        }

        if (*yaw_pixels < 0 || *pitch_angle < 0.0) {
            set_error("allowable_error must be non-negative");
            return std::nullopt;
        }

        return VisionAimAllowableError{*yaw_pixels, *pitch_angle};
    }

    std::optional<cv::Point2i> read_point(
        rclcpp::Node& node, const std::string& base_name, bool required, bool require_non_negative,
        const std::string& label) {
        const std::string x_name = base_name + ".x";
        const std::string y_name = base_name + ".y";
        const bool has_x = node.has_parameter(x_name);
        const bool has_y = node.has_parameter(y_name);

        if (!has_x && !has_y) {
            if (required) {
                set_error("missing " + label + " configuration");
            }
            return std::nullopt;
        }

        if (!has_x || !has_y) {
            set_error("incomplete " + label + " configuration");
            return std::nullopt;
        }

        const auto x = read_int(node, x_name, label + ".x");
        if (!x.has_value()) {
            return std::nullopt;
        }

        const auto y = read_int(node, y_name, label + ".y");
        if (!y.has_value()) {
            return std::nullopt;
        }

        if (require_non_negative && (*x < 0 || *y < 0)) {
            set_error(label + " must be non-negative");
            return std::nullopt;
        }

        return cv::Point2i(*x, *y);
    }

    std::optional<int> read_int(
        rclcpp::Node& node, const std::string& parameter_name, const std::string& label) {
        if (!node.has_parameter(parameter_name)) {
            return std::nullopt;
        }

        const int64_t raw_value = node.get_parameter(parameter_name).as_int();
        if (raw_value < static_cast<int64_t>(std::numeric_limits<int>::min())
            || raw_value > static_cast<int64_t>(std::numeric_limits<int>::max())) {
            set_error(label + " is out of int range");
            return std::nullopt;
        }

        return static_cast<int>(raw_value);
    }

    std::optional<double> read_double(
        rclcpp::Node& node, const std::string& parameter_name, const std::string&) {
        if (!node.has_parameter(parameter_name)) {
            return std::nullopt;
        }

        return node.get_parameter(parameter_name).as_double();
    }

    std::optional<double> read_number(
        rclcpp::Node& node, const std::string& parameter_name, const std::string& label) {
        if (!node.has_parameter(parameter_name)) {
            return std::nullopt;
        }

        const auto parameter = node.get_parameter(parameter_name);
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            return parameter.as_double();
        }

        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            return static_cast<double>(parameter.as_int());
        }

        set_error(label + " must be numeric");
        return std::nullopt;
    }

    std::optional<uint64_t> read_uint64(
        rclcpp::Node& node, const std::string& parameter_name, const std::string& label) {
        if (!node.has_parameter(parameter_name)) {
            set_error("missing " + label + " configuration");
            return std::nullopt;
        }

        const int64_t raw_value = node.get_parameter(parameter_name).as_int();
        if (raw_value < 0) {
            set_error(label + " must be non-negative");
            return std::nullopt;
        }

        return static_cast<uint64_t>(raw_value);
    }

    void set_error(std::string message) {
        if (error_message_.empty()) {
            error_message_ = std::move(message);
        }
    }

    bool valid_{false};
    std::string error_message_;
    VisionAimAllowableError allowable_error_{};
    uint64_t timeout_ticks_{0};
    std::vector<VisionAimShotProfile> shot_profiles_;
};

} // namespace rmcs_dart_guidance::manager
