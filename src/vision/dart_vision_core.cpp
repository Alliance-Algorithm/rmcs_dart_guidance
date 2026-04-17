#include "identifier.hpp"
#include "image_recorder.hpp"
#include "tracker.hpp"
#include "video_recorder.hpp"
#include <atomic>
#include <chrono>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>

namespace rmcs_dart_guidance {

class DartVisionCore
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartVisionCore()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        camera_profile_.invert_image = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time =
            std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain = static_cast<float>(get_parameter("gain").as_double());

        lower_limit_default_ = cv::Scalar(
            get_parameter("L_H").as_double(), get_parameter("L_S").as_double(),
            get_parameter("L_V").as_double());
        upper_limit_default_ = cv::Scalar(
            get_parameter("U_H").as_double(), get_parameter("U_S").as_double(),
            get_parameter("U_V").as_double());

        enable_image_saving_ = has_parameter("enable_image_saving")
                                 ? get_parameter("enable_image_saving").as_bool()
                                 : false;
        save_directory_ = has_parameter("image_save_directory")
                            ? get_parameter("image_save_directory").as_string()
                            : "./saved_images";
        save_interval_ms_ = has_parameter("image_save_interval_ms")
                              ? static_cast<int>(get_parameter("image_save_interval_ms").as_int())
                              : 1000;
        save_raw_image_ =
            has_parameter("save_raw_image") ? get_parameter("save_raw_image").as_bool() : true;
        save_processed_image_ = has_parameter("save_processed_image")
                                  ? get_parameter("save_processed_image").as_bool()
                                  : false;

        // Video recording parameters
        enable_video_recording_ = has_parameter("enable_video_recording");
        video_save_directory_ = has_parameter("video_save_directory")
                                  ? get_parameter("video_save_directory").as_string()
                                  : "./saved_videos";
        video_codec_ =
            has_parameter("video_codec") ? get_parameter("video_codec").as_string() : "H264";
        video_fps_ =
            has_parameter("video_fps") ? static_cast<int>(get_parameter("video_fps").as_int()) : 30;
        video_rotation_minutes_ =
            has_parameter("video_rotation_minutes")
                ? static_cast<int>(get_parameter("video_rotation_minutes").as_int())
                : 5;
        record_raw_video_ =
            has_parameter("record_raw_video") ? get_parameter("record_raw_video").as_bool() : true;
        record_processed_video_ = has_parameter("record_processed_video")
                                    ? get_parameter("record_processed_video").as_bool()
                                    : true;

        image_capture_ = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);

        identifier_.set_default_limit(lower_limit_default_, upper_limit_default_);
        identifier_.Init();

        if (enable_image_saving_) {
            image_recorder_ = std::make_unique<ImageRecorder>(logger_);
            if (!image_recorder_->Init(save_directory_)) {
                enable_image_saving_ = false;
            }
        }

        if (enable_video_recording_) {
            video_recorder_ = std::make_unique<VideoRecorder>(logger_);
            if (!video_recorder_->Init(
                    video_save_directory_, video_codec_, video_fps_, video_rotation_minutes_)) {
                enable_video_recording_ = false;
                RCLCPP_ERROR(logger_, "Failed to initialize video recorder");
            }
        }

        register_output("/dart_guidance/camera/camera_image", camera_image_);
        register_output("/dart_guidance/camera/display_image", display_image_);
        register_output("/dart_guidance/camera/target_position", target_position_, PointT(-1, -1));
        register_output("/dart_guidance/tracker/tracking", tracking_);
        if (enable_image_saving_) {
            RCLCPP_INFO(logger_, "Image saving enabled:");
            RCLCPP_INFO(logger_, "  Directory: %s", save_directory_.c_str());
            RCLCPP_INFO(logger_, "  Interval: %d ms", save_interval_ms_);
            RCLCPP_INFO(logger_, "  Save raw: %s", save_raw_image_ ? "true" : "false");
            RCLCPP_INFO(logger_, "  Save processed: %s", save_processed_image_ ? "true" : "false");
        } else {
            RCLCPP_INFO(logger_, "Image saving disabled");
        }

        if (enable_video_recording_) {
            RCLCPP_INFO(logger_, "Video recording enabled:");
            RCLCPP_INFO(logger_, "  Directory: %s", video_save_directory_.c_str());
            RCLCPP_INFO(logger_, "  Codec: %s", video_codec_.c_str());
            RCLCPP_INFO(logger_, "  FPS: %d", video_fps_);
            RCLCPP_INFO(logger_, "  Rotation: %d minutes", video_rotation_minutes_);
            RCLCPP_INFO(logger_, "  Record raw: %s", record_raw_video_ ? "true" : "false");
            RCLCPP_INFO(
                logger_, "  Record processed: %s", record_processed_video_ ? "true" : "false");
        } else {
            RCLCPP_INFO(logger_, "Video recording disabled");
        }

        camera_thread_ = std::thread(&DartVisionCore::camera_update, this);
        update_time_point_ = std::chrono::steady_clock::now();
    }

    ~DartVisionCore() {
        is_running_ = false;
        if (image_recorder_) {
            image_recorder_->stop();
        }
        if (video_recorder_) {
            video_recorder_->stop();
        }
        if (camera_thread_.joinable()) {
            camera_thread_.join();
        }
    }

    void update() override {}

    std::chrono::steady_clock::time_point update_time_point_;

private:
    void camera_update() {
        int frame_counter = 0;
        int saved_raw_counter = 0;
        int saved_processed_counter = 0;

        std::chrono::steady_clock::time_point last_save_time = std::chrono::steady_clock::now();

        while (rclcpp::ok() && is_running_) {
            try {
                frame_counter++;

                cv::Mat raw_image = image_capture_->read();
                if (raw_image.empty()) {
                    RCLCPP_WARN_THROTTLE(
                        logger_, *this->get_clock(), 1000, "Received empty image from camera");
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }

                auto now = std::chrono::steady_clock::now();
                auto elapsed_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_save_time)
                        .count();

                bool time_to_save = enable_image_saving_ && elapsed_ms >= save_interval_ms_;

                if (time_to_save && save_raw_image_ && image_recorder_) {
                    image_recorder_->push_image(raw_image, "raw");
                    saved_raw_counter++;
                    RCLCPP_INFO(logger_, "Pushed raw image %d to save queue", saved_raw_counter);
                }

                if (enable_video_recording_ && record_raw_video_ && video_recorder_) {
                    video_recorder_->push_frame(raw_image, "raw");
                }

                cv::Mat published_raw = raw_image.clone();
                int cx = published_raw.cols / 2;
                int cy = published_raw.rows / 2;

                cv::line(
                    published_raw, cv::Point(cx, 0), cv::Point(cx, published_raw.rows),
                    cv::Scalar(255, 0, 255), 1);

                *camera_image_ = published_raw;

                cv::Mat preprocessed_image;
                image_to_binary(raw_image, preprocessed_image);

                cv::Mat display_image = preprocessed_image.clone();
                process_frame(preprocessed_image, display_image);

                cv::line(
                    display_image, cv::Point(0, cy), cv::Point(display_image.cols, cy),
                    cv::Scalar(255, 0, 255), 1);
                *display_image_ = display_image;

                if (time_to_save) {
                    if (save_processed_image_ && image_recorder_) {
                        image_recorder_->push_image(display_image, "processed");
                        saved_processed_counter++;
                        RCLCPP_INFO(
                            logger_, "Pushed processed image %d to save queue",
                            saved_processed_counter);
                    }
                    last_save_time = now;
                }

                if (enable_video_recording_ && record_processed_video_ && video_recorder_) {
                    video_recorder_->push_frame(display_image, "processed");
                }

                if (frame_counter % 30 == 0) {
                    RCLCPP_DEBUG(logger_, "Processed %d frames", frame_counter);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(1));

            } catch (const std::exception& e) {
                RCLCPP_ERROR(logger_, "Error in camera_update: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        RCLCPP_INFO(
            logger_, "Camera thread stopped. Frames: %d, Raw saved: %d, Processed saved: %d",
            frame_counter, saved_raw_counter, saved_processed_counter);
    }

    void process_frame(cv::Mat& preprocessed_image, cv::Mat& display_image) {
        if (!is_tracker_stage_) {
            identifier_.update(preprocessed_image);

            cv::putText(
                display_image, "Identifying", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                cv::Scalar(255, 0, 255), 2);

            if (!identifier_.result_status_()) {
                *target_position_ = PointT(-1, -1);
                *tracking_ = false;
            } else {
                cv::Point2i initial_position = identifier_.get_result();
                RCLCPP_INFO(
                    logger_, "Target initial position:(%d,%d)", initial_position.x,
                    initial_position.y);

                is_tracker_stage_ = true;
                tracker_.Init(initial_position);

                *target_position_ = initial_position;
                *tracking_ = true;
            }
        } else {
            tracker_.update(preprocessed_image);
            cv::Point2i current_position = tracker_.get_current_position();

            bool is_tracking = tracker_.get_tracking_status();
            cv::putText(
                display_image, is_tracking ? "Tracking" : "Lost", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2);

            if (!is_tracking) {
                tracker_loss_count_++;
                *target_position_ = PointT(-1, -1);
                *tracking_ = false;

                if (tracker_loss_count_ > 100) {
                    is_tracker_stage_ = false;
                    identifier_.Init();
                    tracker_loss_count_ = 0;
                    RCLCPP_INFO(logger_, "Target lost, switching to identification mode");
                }
            } else {
                tracker_loss_count_ = 0;
                *target_position_ = current_position;
                *tracking_ = true;
                cv::circle(display_image, current_position, 20, cv::Scalar(255, 0, 255), 2);
            }
        }
    }

    void image_to_binary(const cv::Mat& src, cv::Mat& output) {
        cv::Mat HSV_image;
        cv::cvtColor(src, HSV_image, cv::COLOR_BGR2HSV);

        cv::Mat binary;
        cv::inRange(HSV_image, lower_limit_default_, upper_limit_default_, binary);

        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
        output = binary;
    }

    rclcpp::Logger logger_;

    bool enable_image_saving_ = false;

    std::string save_directory_ = "./saved_images";
    int save_interval_ms_ = 1000;
    bool save_raw_image_ = true;
    bool save_processed_image_ = false;

    std::unique_ptr<ImageRecorder> image_recorder_;

    bool enable_video_recording_ = false;
    std::string video_save_directory_ = "./saved_videos";
    std::string video_codec_ = "H264";
    int video_fps_ = 30;
    int video_rotation_minutes_ = 5;
    bool record_raw_video_ = true;
    bool record_processed_video_ = true;

    std::unique_ptr<VideoRecorder> video_recorder_;

    std::thread camera_thread_;
    std::mutex camera_thread_mtx_;
    std::atomic<bool> is_running_{true};

    cv::Scalar lower_limit_default_, upper_limit_default_;

    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;

    OutputInterface<cv::Mat> camera_image_;
    OutputInterface<cv::Mat> display_image_;
    OutputInterface<bool> tracking_;
    OutputInterface<cv::Point2i> target_position_;

    DartGuidanceIdentifier identifier_;
    DartGuidanceTracker tracker_;
    bool is_tracker_stage_ = false;
    int tracker_loss_count_ = 0;
};
} // namespace rmcs_dart_guidance

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::DartVisionCore, rmcs_executor::Component)