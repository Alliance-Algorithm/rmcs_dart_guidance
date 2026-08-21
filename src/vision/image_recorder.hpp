#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <iomanip>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

namespace rmcs_dart_guidance {

class ImageRecorder {
public:
    explicit ImageRecorder(rclcpp::Logger logger)
        : logger_(std::move(logger)) {}

    ~ImageRecorder() { stop(); }

    bool Init(const std::string& save_directory) {
        save_directory_ = save_directory;
        save_counter_ = 0;

        try {
            std::filesystem::create_directories(save_directory_);
            RCLCPP_INFO(logger_, "Created image save directory: %s", save_directory_.c_str());

            if (!test_write_permission()) {
                return false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                logger_, "Failed to create save directory %s: %s", save_directory_.c_str(),
                e.what());
            return false;
        }

        is_running_ = true;
        worker_thread_ = std::thread(&ImageRecorder::worker_loop, this);
        return true;
    }

    void push_image(const cv::Mat& image, const std::string& type) {
        if (!is_running_)
            return;

        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (job_queue_.size() < max_queue_size_) {
            job_queue_.push({image.clone(), type, std::chrono::system_clock::now()});
            queue_cv_.notify_one();
        } else {
            if (dropped_frames_++ % 30 == 0) {
                RCLCPP_WARN(logger_, "Image save queue is full, dropping frame (%s)", type.c_str());
            }
        }
    }

    void stop() {
        if (is_running_) {
            is_running_ = false;
            queue_cv_.notify_all();
            if (worker_thread_.joinable()) {
                worker_thread_.join();
            }
        }
    }

private:
    struct RecordJob {
        cv::Mat image;
        std::string type;
        std::chrono::system_clock::time_point timestamp;
    };

    bool test_write_permission() {
        std::string test_file = save_directory_ + "/test_write.jpg";
        cv::Mat test_image(10, 10, CV_8UC3, cv::Scalar(0, 0, 0));

        try {
            bool success = cv::imwrite(test_file, test_image);
            if (success && std::filesystem::exists(test_file)) {
                std::filesystem::remove(test_file);
                return true;
            } else {
                RCLCPP_ERROR(logger_, "Write test failed: %s", test_file.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Write test exception: %s", e.what());
            return false;
        }
    }

    void worker_loop() {
        while (is_running_ || !job_queue_.empty()) {
            RecordJob job;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this] { return !job_queue_.empty() || !is_running_; });

                if (job_queue_.empty()) {
                    continue;
                }

                job = std::move(job_queue_.front());
                job_queue_.pop();
            }

            save_to_disk(job);
        }
    }

    void save_to_disk(const RecordJob& job) {
        try {
            auto time_t = std::chrono::system_clock::to_time_t(job.timestamp);
            std::tm tm_buf;
            localtime_r(&time_t, &tm_buf);

            std::ostringstream oss;
            oss << save_directory_ << "/" << job.type << "_"
                << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << "_" << std::setw(4)
                << std::setfill('0') << save_counter_++ << ".jpg";

            std::string filename = oss.str();
            bool success = cv::imwrite(filename, job.image);
            if (!success) {
                RCLCPP_ERROR(logger_, "cv::imwrite failed for: %s", filename.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Error saving image: %s", e.what());
        }
    }

    rclcpp::Logger logger_;
    std::string save_directory_;
    int save_counter_ = 0;
    int dropped_frames_ = 0;

    std::queue<RecordJob> job_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    std::atomic<bool> is_running_{false};
    const size_t max_queue_size_ = 50;
};

} // namespace rmcs_dart_guidance
