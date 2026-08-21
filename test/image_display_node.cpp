#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace rmcs_dart_guidance {

class DebugDisplayComponent
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DebugDisplayComponent()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        raw_image_topic_ = get_parameter("raw_image_topic").as_string();
        processed_image_topic_ = get_parameter("processed_image_topic").as_string();
        target_topic_ = get_parameter("target_topic").as_string();

        display_raw_ = get_parameter("display_raw").as_bool();
        display_processed_ = get_parameter("display_processed").as_bool();
        max_fps_ = static_cast<int>(get_parameter("max_fps").as_int());
        window_scale_ = get_parameter("window_scale").as_double();

        RCLCPP_INFO(logger_, "DebugDisplayComponent constructed (Headless Mode)");
    }

    ~DebugDisplayComponent() {}

    void update() override {
        if (!initialized_) {
            initialize();
            initialized_ = true;
        }
    }

private:
    void initialize() {
        RCLCPP_INFO(logger_, "Initializing DebugDisplayComponent");

        if (display_raw_) {
            raw_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                raw_image_topic_, 10, [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
                    raw_image_callback(msg);
                });
            raw_debug_pub_ =
                this->create_publisher<sensor_msgs::msg::Image>(raw_image_topic_ + "/debug", 10);
            RCLCPP_INFO(logger_, "Subscribed to raw image topic: %s", raw_image_topic_.c_str());
        }

        if (display_processed_) {
            processed_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                processed_image_topic_, 10,
                [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
                    processed_image_callback(msg);
                });

            target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                target_topic_, 10,
                [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg) {
                    target_callback(msg);
                });

            processed_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                processed_image_topic_ + "/debug", 10);
            RCLCPP_INFO(
                logger_, "Subscribed to processed image topic: %s", processed_image_topic_.c_str());
            RCLCPP_INFO(logger_, "Subscribed to target topic: %s", target_topic_.c_str());
        }

        int frame_delay_ms = 1000 / std::max(1, max_fps_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(frame_delay_ms),
            std::bind(&DebugDisplayComponent::timer_callback, this));

        RCLCPP_INFO(logger_, "Publishing timer started with delay %d ms", frame_delay_ms);
    }

    void raw_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(raw_mutex_);
        try {
            raw_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
            raw_image_header_ = msg->header;
            raw_image_timestamp_ = std::chrono::steady_clock::now();
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(logger_, "cv_bridge exception for raw image: %s", e.what());
        }
    }

    void processed_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(processed_mutex_);
        try {
            processed_image_ = cv_bridge::toCvCopy(msg, "mono8")->image;
            processed_image_header_ = msg->header;
            processed_image_timestamp_ = std::chrono::steady_clock::now();
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(logger_, "cv_bridge exception for processed image: %s", e.what());
        }
    }

    void target_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_position_.x = static_cast<int>(msg->point.x);
        target_position_.y = static_cast<int>(msg->point.y);
        target_tracking_ = (msg->point.z > 0.5);
        last_target_time_ = std::chrono::steady_clock::now();
    }

    void timer_callback() {
        if (display_raw_) {
            cv::Mat display_raw;
            std_msgs::msg::Header header;
            {
                std::lock_guard<std::mutex> lock(raw_mutex_);
                if (!raw_image_.empty()) {
                    display_raw = raw_image_.clone();
                    header = raw_image_header_;

                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       now - raw_image_timestamp_)
                                       .count();

                    if (elapsed > 500) {
                        cv::putText(
                            display_raw, "STALE", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                            cv::Scalar(0, 0, 255), 2);
                    }
                }
            }

            if (!display_raw.empty()) {
                if (window_scale_ != 1.0) {
                    cv::resize(display_raw, display_raw, cv::Size(), window_scale_, window_scale_);
                }
                auto msg = cv_bridge::CvImage(header, "bgr8", display_raw).toImageMsg();
                raw_debug_pub_->publish(*msg);
            }
        }

        if (display_processed_) {
            cv::Mat display_processed;
            std_msgs::msg::Header header;
            bool local_target_tracking = false;
            cv::Point local_target_position;
            {
                std::lock_guard<std::mutex> lock(target_mutex_);
                local_target_tracking = target_tracking_;
                local_target_position = target_position_;
            }
            {
                std::lock_guard<std::mutex> lock(processed_mutex_);
                if (!processed_image_.empty()) {
                    display_processed = processed_image_.clone();
                    header = processed_image_header_;

                    if (display_processed.channels() == 1) {
                        cv::cvtColor(display_processed, display_processed, cv::COLOR_GRAY2BGR);
                    }

                    if (local_target_tracking) {
                        cv::circle(
                            display_processed, local_target_position, 20, cv::Scalar(0, 255, 255),
                            2);
                        cv::putText(
                            display_processed, "TRACKING", cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                    }

                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       now - processed_image_timestamp_)
                                       .count();

                    if (elapsed > 500) {
                        cv::putText(
                            display_processed, "STALE", cv::Point(500, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
                    }
                }
            }

            if (!display_processed.empty()) {
                if (window_scale_ != 1.0) {
                    cv::resize(
                        display_processed, display_processed, cv::Size(), window_scale_,
                        window_scale_);
                }
                auto msg = cv_bridge::CvImage(header, "bgr8", display_processed).toImageMsg();
                processed_debug_pub_->publish(*msg);
            }
        }
    }

private:
    rclcpp::Logger logger_;

    std::string raw_image_topic_;
    std::string processed_image_topic_;
    std::string target_topic_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr processed_image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_debug_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::Mat raw_image_;
    std_msgs::msg::Header raw_image_header_;
    cv::Mat processed_image_;
    std_msgs::msg::Header processed_image_header_;
    std::chrono::steady_clock::time_point raw_image_timestamp_;
    std::chrono::steady_clock::time_point processed_image_timestamp_;

    cv::Point target_position_;
    bool target_tracking_ = false;
    std::chrono::steady_clock::time_point last_target_time_;

    std::mutex raw_mutex_;
    std::mutex processed_mutex_;
    std::mutex target_mutex_;

    std::atomic<bool> initialized_{false};

    bool display_raw_ = false;
    bool display_processed_ = true;
    int max_fps_ = 30;
    double window_scale_ = 1.0;
};

} // namespace rmcs_dart_guidance

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::DebugDisplayComponent, rmcs_executor::Component)