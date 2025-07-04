#include <yolo_track/controlNode.hpp>
#include <yolo_track/sysfsControl.hpp>
#include <thread>
#include <atomic>
#include <algorithm>

namespace servo {
    ControlNode::ControlNode() : Node("control_node") {
        error_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "track_error", 10,
            std::bind(&ControlNode::error_callback, this, std::placeholders::_1));
        // 使用sysfsControl初始化GPIO
        SysfsControl::export_gpio(0);
        SysfsControl::export_gpio(1);
        SysfsControl::set_gpio_direction(0, "out");
        SysfsControl::set_gpio_direction(1, "out");
        // Start threads for PWM control
        horizontal_running_ = true;
        vertical_running_ = true;
        horizontal_thread_ = std::thread([this]() {
            while (horizontal_running_) {
                set_pwm(0, 50, horizontal_running_); // Default duty cycle
            }
        });
        vertical_thread_ = std::thread([this]() {
            while (vertical_running_) {
                set_pwm(1, 50, vertical_running_); // Default duty cycle
            }
        });
    }

    ControlNode::~ControlNode() {
        horizontal_running_ = false;
        vertical_running_ = false;
        if (horizontal_thread_.joinable()) {
            horizontal_thread_.join();
        }
        if (vertical_thread_.joinable()) {
            vertical_thread_.join();
        }
        SysfsControl::unexport_gpio(0);
        SysfsControl::unexport_gpio(1);
    }

    void ControlNode::error_callback(
        const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 2) {
            RCLCPP_WARN(get_logger(), "Invalid error message size.");
            return;
        }
        int err_x = msg->data[0];
        int err_y = msg->data[1];

        // Convert error to PWM duty cycle (range: 0-100)
        int duty_cycle_x = std::clamp(50 + err_x, 0, 100);
        int duty_cycle_y = std::clamp(50 + err_y, 0, 100);

        // Update duty cycles
        horizontal_running_ = false;
        vertical_running_ = false;
        if (horizontal_thread_.joinable()) horizontal_thread_.join();
        if (vertical_thread_.joinable()) vertical_thread_.join();
        // Restart threads with new duty cycles
        horizontal_running_ = true;
        vertical_running_ = true;
        horizontal_thread_ = std::thread([this, duty_cycle_x]() {
            set_pwm(0, duty_cycle_x, horizontal_running_);
        });
        vertical_thread_ = std::thread([this, duty_cycle_y]() {
            set_pwm(1, duty_cycle_y, vertical_running_);
        });
    }

    void ControlNode::set_pwm(int gpio, int duty_cycle, std::atomic<bool>& running_flag) {
        using namespace std::chrono;
        auto period = 20ms; // 20ms period for standard servo PWM
        auto high_time = period * duty_cycle / 100;
        auto low_time = period - high_time;

        while (running_flag) {
            SysfsControl::write_gpio(gpio, 1);
            std::this_thread::sleep_for(high_time);
            SysfsControl::write_gpio(gpio, 0);
            std::this_thread::sleep_for(low_time);
        }
    }
}