#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/bool.hpp>

namespace rmcs_dart_guidance::test {

// ForceSensorRecorder
//   独立的力传感器数据记录组件，在fire命令前异步保存双通道力传感器读数
//   - 保留双重计数逻辑：总发射次数和当前轮次（1-4循环）
//   - 异步写入，不阻塞控制循环
class ForceSensorRecorder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ForceSensorRecorder()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {

        // 输入：力传感器双通道数据
        register_input("/force_sensor/channel_1/weight", force_ch1_);
        register_input("/force_sensor/channel_2/weight", force_ch2_);

        // 输入：fire触发信号（由DartManager在fire命令时发出）
        register_input("/dart/manager/fire_trigger", fire_trigger_);

        // 获取配置参数（使用相对路径，默认保存到当前工作目录）
        output_file_path_ = get_parameter("output_file_path").as_string();
        dart_forebody_ = get_parameter("dart_forebody").as_string();

        // 初始化文件（写入表头）
        initialize_output_file();

        RCLCPP_INFO(
            logger_, "[ForceSensorRecorder] Initialized, output: %s, forebody: %s",
            output_file_path_.c_str(), dart_forebody_.c_str());
    }

    ~ForceSensorRecorder() {
        // 等待所有异步写入完成
        if (write_thread_.joinable()) {
            write_thread_.join();
        }
    }

    void update() override {
        // 检测fire触发信号的上升沿
        if (fire_trigger_.ready()) {
            bool current_trigger = *fire_trigger_;
            if (current_trigger && !last_trigger_state_) {
                // 上升沿：记录当前力传感器数据
                record_force_data();
            }
            last_trigger_state_ = current_trigger;
        }
    }

private:
    void initialize_output_file() {
        std::ofstream file(output_file_path_, std::ios::trunc);
        if (!file.is_open()) {
            RCLCPP_ERROR(
                logger_, "[ForceSensorRecorder] Failed to create file: %s",
                output_file_path_.c_str());
            return;
        }

        file << "# 发射前力传感器数值记录\n\n";
        file << "**镖体构型**: " << dart_forebody_ << "\n\n";
        file << "| 总发射次数 | 当前轮次 | 通道1力值 | 通道2力值 |\n";
        file << "|-----------|---------|----------|----------|\n";
        file.close();

        RCLCPP_INFO(logger_, "[ForceSensorRecorder] Output file initialized with header");
    }

    void record_force_data() {
        // 读取当前力传感器数据
        int ch1_value = force_ch1_.ready() ? *force_ch1_ : 0;
        int ch2_value = force_ch2_.ready() ? *force_ch2_ : 0;

        // 递增总发射次数
        total_shot_count_++;

        // 计算当前轮次 (1-4循环)
        uint32_t round_number = ((total_shot_count_ - 1) % 4) + 1;

        RCLCPP_INFO(
            logger_, "[ForceSensorRecorder] Recording shot #%u (round %u): ch1=%d, ch2=%d",
            total_shot_count_, round_number, ch1_value, ch2_value);

        // 异步写入文件（避免阻塞控制循环）
        async_write_to_file(total_shot_count_, round_number, ch1_value, ch2_value);
    }

    void async_write_to_file(
        uint32_t shot_number, uint32_t round_number, int ch1_value, int ch2_value) {

        // 如果上一个写入线程还在运行，等待它完成
        if (write_thread_.joinable()) {
            write_thread_.join();
        }

        // 启动新的写入线程
        write_thread_ = std::thread([this, shot_number, round_number, ch1_value, ch2_value]() {
            std::lock_guard<std::mutex> lock(file_mutex_);

            std::ofstream file(output_file_path_, std::ios::app);
            if (!file.is_open()) {
                RCLCPP_WARN(
                    logger_, "[ForceSensorRecorder] Failed to open file for writing: %s",
                    output_file_path_.c_str());
                return;
            }

            file << "| " << shot_number << " | " << round_number << " | " << ch1_value << " | "
                 << ch2_value << " |\n";
            file.close();
        });
    }

    rclcpp::Logger logger_;

    // 输入接口
    InputInterface<int> force_ch1_;
    InputInterface<int> force_ch2_;
    InputInterface<bool> fire_trigger_;

    // 配置参数
    std::string output_file_path_;
    std::string dart_forebody_;

    // 状态变量
    uint32_t total_shot_count_{0};   // 总发射次数（持续递增）
    bool last_trigger_state_{false}; // 上一次触发信号状态

    // 异步写入
    std::mutex file_mutex_;
    std::thread write_thread_;
};

} // namespace rmcs_dart_guidance::test

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::test::ForceSensorRecorder, rmcs_executor::Component)
