#include <memory>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detection.hpp>
#include <yolo_track/trackNode.hpp>
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<yolo_track::trackNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
namespace yolo_track {
trackNode::trackNode() : Node("yolo_track_node") {
  // Declare and get parameters
  this->declare_parameter<bool>("use_hard_pwm", true);
  this->declare_parameter<int>("pwm_chip", 0);
  this->declare_parameter<int>("pwm_channel_x", 0);
  this->declare_parameter<int>("pwm_channel_y", 1);
  this->declare_parameter<int>("gpio_x", 0);
  this->declare_parameter<int>("gpio_y", 1);
  this->declare_parameter<int>("pwm_period_ns", 20000000);
  this->declare_parameter<int>("min_duty_ns", 500000);
  this->declare_parameter<int>("max_duty_ns", 2500000);

  use_hard_pwm_ = this->get_parameter("use_hard_pwm").as_bool();
  pwm_chip_ = this->get_parameter("pwm_chip").as_int();
  pwm_channel_x_ = this->get_parameter("pwm_channel_x").as_int();
  pwm_channel_y_ = this->get_parameter("pwm_channel_y").as_int();
  gpio_x_ = this->get_parameter("gpio_x").as_int();
  gpio_y_ = this->get_parameter("gpio_y").as_int();
  pwm_period_ns_ = this->get_parameter("pwm_period_ns").as_int();
  min_duty_ns_ = this->get_parameter("min_duty_ns").as_int();
  max_duty_ns_ = this->get_parameter("max_duty_ns").as_int();
  
  if (use_hard_pwm_) {
    SysfsControl::export_pwm(pwm_chip_, pwm_channel_x_);
    SysfsControl::export_pwm(pwm_chip_, pwm_channel_y_);
    SysfsControl::set_pwm_period(pwm_chip_, pwm_channel_x_, pwm_period_ns_);
    SysfsControl::set_pwm_period(pwm_chip_, pwm_channel_y_, pwm_period_ns_);
    SysfsControl::set_pwm_duty_cycle(pwm_chip_, pwm_channel_x_, 1500000);
    SysfsControl::set_pwm_duty_cycle(pwm_chip_, pwm_channel_y_, 1500000);
    SysfsControl::enable_pwm(pwm_chip_, pwm_channel_x_, true);
    SysfsControl::enable_pwm(pwm_chip_, pwm_channel_y_, true);
  } else {
    SysfsControl::export_gpio(gpio_x_);
    SysfsControl::export_gpio(gpio_y_);
    SysfsControl::set_gpio_direction(gpio_x_, "out");
    SysfsControl::set_gpio_direction(gpio_y_, "out");
    running_ = true;
    soft_pwm_thread_x_ = std::thread(&trackNode::soft_pwm_loop, this, gpio_x_, &duty_x_);
    soft_pwm_thread_y_ = std::thread(&trackNode::soft_pwm_loop, this, gpio_y_, &duty_y_);
  }

  yolo_sub_ = create_subscription<yolo_msgs::msg::DetectionArray>(
      "yolo_detections", 10,
      std::bind(&trackNode::sub_callback, this, std::placeholders::_1));

  error_pub_ =
      create_publisher<std_msgs::msg::Int32MultiArray>("track_error", 10);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        resolution.first = msg->width;
        resolution.second = msg->height;
        RCLCPP_INFO(get_logger(), "Image resolution: %d x %d", resolution.first,
                    resolution.second);
        image_sub_.reset();  // Unsubscribe after getting the resolution
      });
}

trackNode::~trackNode() {
  running_ = false;
  if (soft_pwm_thread_x_.joinable()) soft_pwm_thread_x_.join();
  if (soft_pwm_thread_y_.joinable()) soft_pwm_thread_y_.join();
  if (use_hard_pwm_) {
    SysfsControl::enable_pwm(pwm_chip_, pwm_channel_x_, false);
    SysfsControl::enable_pwm(pwm_chip_, pwm_channel_y_, false);
    SysfsControl::unexport_pwm(pwm_chip_, pwm_channel_x_);
    SysfsControl::unexport_pwm(pwm_chip_, pwm_channel_y_);
  } else {
    SysfsControl::unexport_gpio(gpio_x_);
    SysfsControl::unexport_gpio(gpio_y_);
  }
}

void trackNode::set_servo_angle(int axis, int err) {
  int* duty_ptr = (axis == 0) ? &duty_x_ : &duty_y_;
  int pwm_channel = (axis == 0) ? pwm_channel_x_ : pwm_channel_y_;
  int max_err = 200;
  int duty = 1500000 + (err * (max_duty_ns_ - min_duty_ns_) / (2 * max_err));
  if (duty < min_duty_ns_) duty = min_duty_ns_;
  if (duty > max_duty_ns_) duty = max_duty_ns_;
  *duty_ptr = duty;
  if (use_hard_pwm_) {
    SysfsControl::set_pwm_duty_cycle(pwm_chip_, pwm_channel, duty);
  }
}

void trackNode::soft_pwm_loop(int gpio, int* duty_ns_ptr) {
  while (running_) {
    SysfsControl::write_gpio(gpio, 1);
    std::this_thread::sleep_for(std::chrono::nanoseconds(*duty_ns_ptr));
    SysfsControl::write_gpio(gpio, 0);
    std::this_thread::sleep_for(std::chrono::nanoseconds(pwm_period_ns_ - *duty_ns_ptr));
  }
}

void yolo_track::trackNode::sub_callback(
    const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
  yolo_msgs::msg::Detection current_detection;
  if (resolution.first == 0 || resolution.second == 0) {
    RCLCPP_WARN(get_logger(), "Image resolution not set yet.");
    return;
  }
  if (msg->detections.empty()) {
    RCLCPP_WARN(get_logger(), "No detections received.");
    return;
  }
  if (current_track_id_.empty()) {
    current_track_id_ = msg->detections.back().id;
  } else {
    for (auto it = msg->detections.end(); it != msg->detections.begin(); --it) {
      if (it->id == current_track_id_) {
        current_detection = *it;
        break;
      }
    }
  }
  int mid_x = resolution.first / 2;
  int mid_y = resolution.second / 2;
  int err_x = mid_x - current_detection.bbox.center.position.x;
  int err_y = mid_y - current_detection.bbox.center.position.y;
  set_servo_angle(0, err_x);
  set_servo_angle(1, err_y);
  auto tracking_obj = msg->detections.back().class_name;
  RCLCPP_INFO(get_logger(), "Current track ID: %s", current_track_id_.c_str());
  std_msgs::msg::Int32MultiArray error_msg;
  error_msg.data.push_back(err_x);
  error_msg.data.push_back(err_y);
  error_pub_->publish(error_msg);
}
}  // namespace yolo_track