#include <memory>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detection.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <yolo_track/trackNode.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detection.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<yolo_track::trackNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
namespace yolo_track {
trackNode::trackNode() : Node("yolo_track_node") {
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
trackNode::~trackNode()=default;
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
  auto tracking_obj = msg->detections.back().class_name;
  RCLCPP_INFO(get_logger(), "Current track ID: %s", current_track_id_.c_str());
  std_msgs::msg::Int32MultiArray error_msg;
  error_msg.data.push_back(err_x);
  error_msg.data.push_back(err_y);
  error_pub_->publish(error_msg);
}
}  // namespace yolo_track