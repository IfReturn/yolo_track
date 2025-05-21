
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <yolo_msgs/msg/detection.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

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
};
}  // namespace yolo_track
