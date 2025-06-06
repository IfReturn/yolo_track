#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <yolo_msgs/msg/detection.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <yolo_track/sysfsControl.hpp>
#include <rclcpp/parameter.hpp>
#include <thread>

namespace yolo_track {
class trackNode : public rclcpp::Node {
 public:
  trackNode();
  ~trackNode();

 private:
  std::pair<int, int> resolution;

  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr yolo_sub_;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr error_pub_;

  /// @brief Subscribe to the image topic to get the image resolution
  /// @note This subscription will be destructed after the first message is
  /// received
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  /**
   * @brief Callback function for the YOLO detection subscription.
   * This function processes the incoming detection messages and calculates the
   * error between the detected object's position and the center of the image.
   *
   * @param msg The incoming detection message containing the detected objects.
   * @note publish int32[] type error_msg to topic `track_error` The calculated
   * error in the x and y directions, up and right are positive.
   */
  void sub_callback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);
  std::string current_track_id_{};

  // PWM/GPIO control
  SysfsControl sysfs_ctrl_;
  bool use_hard_pwm_ = true;
  int pwm_chip_ = 0;
  int pwm_channel_ = 0;
  int gpio_x_ = 0;
  int gpio_y_ = 1;
  int pwm_channel_x_ = 0;
  int pwm_channel_y_ = 1;
  int pwm_period_ns_ = 20000000; // 20ms default for servo
  int min_duty_ns_ = 500000;     // 0.5ms
  int max_duty_ns_ = 2500000;    // 2.5ms
  std::thread soft_pwm_thread_x_;
  std::thread soft_pwm_thread_y_;
  bool running_ = true;
  void set_servo_angle(int axis, int err);
  void soft_pwm_loop(int gpio, int* duty_ns_ptr);
  int duty_x_ = 1500000;
  int duty_y_ = 1500000;
};
}  // namespace yolo_track
