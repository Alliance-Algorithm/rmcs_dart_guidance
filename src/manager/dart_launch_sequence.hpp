#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace rmcs_dart_guidance::manager {

struct DartLaunchSequenceConfig {
    size_t dart_count{4};
    Eigen::Vector2d aim_reference_pixel{Eigen::Vector2d::Zero()};
    std::vector<Eigen::Vector2d> aim_dart_offsets_px{};
};

struct DartLaunchSequenceRawConfig {
    int64_t dart_count{4};
    std::vector<double> aim_reference_pixel{};
    std::vector<double> aim_dart_offsets_px{};
};

class DartLaunchSequence {
public:
    void configure_from_parameter_values(DartLaunchSequenceRawConfig raw_config) {
        if (raw_config.dart_count <= 0 || raw_config.dart_count > 255) {
            throw std::runtime_error("dart_count must be within 1..255");
        }
        if (raw_config.aim_reference_pixel.size() != 2) {
            throw std::runtime_error("aim_reference_pixel must contain exactly 2 values");
        }

        const size_t dart_count = static_cast<size_t>(raw_config.dart_count);
        if (raw_config.aim_dart_offsets_px.size() != dart_count * 2) {
            throw std::runtime_error("aim_dart_offsets_px size must equal 2 * dart_count");
        }

        std::vector<Eigen::Vector2d> aim_dart_offsets_px;
        aim_dart_offsets_px.reserve(dart_count);
        for (size_t i = 0; i < dart_count; ++i) {
            aim_dart_offsets_px.emplace_back(
                raw_config.aim_dart_offsets_px[2 * i],
                raw_config.aim_dart_offsets_px[2 * i + 1]);
        }

        configure(DartLaunchSequenceConfig{
            .dart_count = dart_count,
            .aim_reference_pixel = Eigen::Vector2d(
                raw_config.aim_reference_pixel[0], raw_config.aim_reference_pixel[1]),
            .aim_dart_offsets_px = std::move(aim_dart_offsets_px),
        });
    }

    void configure(DartLaunchSequenceConfig config) {
        if (config.dart_count == 0 || config.dart_count > 255) {
            throw std::runtime_error("dart_count must be within 1..255");
        }
        if (config.aim_dart_offsets_px.size() != config.dart_count) {
            throw std::runtime_error("aim_dart_offsets_px size must equal dart_count");
        }

        config_ = std::move(config);
        reset();
    }

    void reset() { current_dart_index_ = 0; }

    bool advance_after_fire() {
        if (current_dart_index_ + 1 >= config_.dart_count) {
            return false;
        }

        ++current_dart_index_;
        return true;
    }

    size_t current_dart_index() const { return current_dart_index_; }

    uint8_t current_dart_index_u8() const {
        return static_cast<uint8_t>(current_dart_index_);
    }

    Eigen::Vector2d current_desired_target_px() const {
        if (config_.aim_dart_offsets_px.empty()) {
            return config_.aim_reference_pixel;
        }

        const size_t clamped_index =
            std::min(current_dart_index_, config_.aim_dart_offsets_px.size() - 1);
        return config_.aim_reference_pixel + config_.aim_dart_offsets_px[clamped_index];
    }

private:
    DartLaunchSequenceConfig config_{};
    size_t current_dart_index_{0};
};

} // namespace rmcs_dart_guidance::manager
