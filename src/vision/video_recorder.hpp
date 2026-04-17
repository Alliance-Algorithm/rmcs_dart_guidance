#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
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

class VideoRecorder {
public:
    explicit VideoRecorder(rclcpp::Logger logger) : logger_(std::move(logger)) {}

    ~VideoRecorder() {
        stop();
    }

    bool Init(const std::string& save_directory, const std::string& codec, int fps, int rotation_minutes) {
        save_directory_ = save_directory;
        fps_ = fps;
        rotation_minutes_ = rotation_minutes;

        // Parse codec
        if (codec == "H264") {
            fourcc_code_ = cv::VideoWriter::fourcc('H', '2', '6', '4');
            file_extension_ = ".mp4";
        } else if (codec == "MJPEG") {
            fourcc_code_ = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            file_extension_ = ".avi";
        } else {
            RCLCPP_ERROR(logger_, "Unknown codec: %s, defaulting to H264", codec.c_str());
            fourcc_code_ = cv::VideoWriter::fourcc('H', '2', '6', '4');
            file_extension_ = ".mp4";
        }

        try {
            std::filesystem::create_directories(save_directory_);
            RCLCPP_INFO(logger_, "Created video save directory: %s", save_directory_.c_str());

            if (!test_write_permission()) {
                return false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to create save directory %s: %s", save_directory_.c_str(), e.what());
            return false;
        }

        is_running_ = true;
        worker_thread_ = std::thread(&VideoRecorder::worker_loop, this);
        return true;
    }

    void push_frame(const cv::Mat& frame, const std::string& type) {
        if (!is_running_) return;

        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (job_queue_.size() < max_queue_size_) {
            job_queue_.push({frame.clone(), type, std::chrono::system_clock::now()});
            queue_cv_.notify_one();
        } else {
            if (dropped_frames_++ % 30 == 0) {
                RCLCPP_WARN(logger_, "Video frame queue is full, dropping frame (%s)", type.c_str());
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

            // Release all video writers
            for (auto& [type, writer] : video_writers_) {
                if (writer.isOpened()) {
                    writer.release();
                    RCLCPP_INFO(logger_, "Finalized video: %s", type.c_str());
                }
            }
            video_writers_.clear();
            video_start_times_.clear();
        }
    }

private:
    struct RecordJob {
        cv::Mat frame;
        std::string type;
        std::chrono::system_clock::time_point timestamp;
    };

    bool test_write_permission() {
        std::string test_file = save_directory_ + "/test_write.txt";
        try {
            std::ofstream ofs(test_file);
            if (ofs.is_open()) {
                ofs << "test";
                ofs.close();
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

            process_frame(job);
        }
    }

    void process_frame(const RecordJob& job) {
        try {
            // Check if we need to rotate video
            rotate_video_if_needed(job.type, job.timestamp);

            // Convert grayscale to BGR if needed (VideoWriter requires BGR)
            cv::Mat frame_to_write;
            if (job.frame.channels() == 1) {
                cv::cvtColor(job.frame, frame_to_write, cv::COLOR_GRAY2BGR);
            } else {
                frame_to_write = job.frame;
            }

            // Get or create video writer for this type
            auto& writer = video_writers_[job.type];

            // Initialize writer on first frame
            if (!writer.isOpened()) {
                std::string filename = generate_filename(job.type);
                writer.open(filename, fourcc_code_, fps_, frame_to_write.size());

                if (!writer.isOpened()) {
                    RCLCPP_ERROR(logger_, "Failed to open VideoWriter: %s", filename.c_str());
                    return;
                }

                video_start_times_[job.type] = job.timestamp;
                RCLCPP_INFO(logger_, "Started recording video: %s (channels: %d)",
                           filename.c_str(), frame_to_write.channels());
            }

            // Write frame
            writer.write(frame_to_write);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Error processing video frame: %s", e.what());
        }
    }

    void rotate_video_if_needed(const std::string& type, const std::chrono::system_clock::time_point& current_time) {
        auto it = video_start_times_.find(type);
        if (it == video_start_times_.end()) {
            return;  // No video started yet
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(current_time - it->second);
        if (elapsed.count() >= rotation_minutes_) {
            // Close current video
            auto& writer = video_writers_[type];
            if (writer.isOpened()) {
                writer.release();
                RCLCPP_INFO(logger_, "Rotated video file for type: %s", type.c_str());
            }
            // Next frame will create a new video file
            video_start_times_.erase(it);
        }
    }

    std::string generate_filename(const std::string& type) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf;
        localtime_r(&time_t, &tm_buf);

        std::ostringstream oss;
        oss << save_directory_ << "/" << type << "_"
            << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
            << file_extension_;

        return oss.str();
    }

    rclcpp::Logger logger_;
    std::string save_directory_;
    int fourcc_code_;
    int fps_;
    int rotation_minutes_;
    std::string file_extension_;
    int dropped_frames_ = 0;

    // Video writers per type
    std::map<std::string, cv::VideoWriter> video_writers_;
    std::map<std::string, std::chrono::system_clock::time_point> video_start_times_;

    // Threading
    std::queue<RecordJob> job_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    std::atomic<bool> is_running_{false};
    const size_t max_queue_size_ = 100;
};

} // namespace rmcs_dart_guidance
