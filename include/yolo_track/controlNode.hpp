#include <gpiod.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <thread>
namespace servo {
    class ControlNode : public rclcpp::Node {
      public:
        ControlNode();
        ~ControlNode(); // Add destructor to stop threads
      private:
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr error_sub_;
        /** @brief Callback function for the error subscription.
        * This function processes the incoming error messages and controls the
        * Raspberry Pi GPIO pins accordingly.
        */
        void error_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
        void set_pwm(gpiod::line& line, int duty_cycle, std::atomic<bool>& running_flag); // New method for PWM control

        gpiod::chip chip_;
        gpiod::line horizontal_line_;
        gpiod::line vertical_line_;
        std::thread horizontal_thread_;
        std::thread vertical_thread_;
        std::atomic<bool> horizontal_running_{true};
        std::atomic<bool> vertical_running_{true};
    };
}