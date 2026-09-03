#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class CameraInfoSplitter : public rclcpp::Node
{
public:
  CameraInfoSplitter()
  : Node("camera_info_splitter")
  {
    color_frame_ = this->declare_parameter<std::string>(
      "color_frame",
      "camera_color_optical_frame");

    depth_frame_ = this->declare_parameter<std::string>(
      "depth_frame",
      "camera_depth_optical_frame");

    auto qos = rclcpp::SensorDataQoS();

    color_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info",
      qos);

    depth_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/depth/camera_info",
      qos);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info_raw",
      qos,
      std::bind(
        &CameraInfoSplitter::camera_info_callback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Color frame: %s",
      color_frame_.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Depth frame: %s",
      depth_frame_.c_str());
  }

private:
  void camera_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (msg->header.frame_id == color_frame_) {
      color_pub_->publish(*msg);
    }
    else if (msg->header.frame_id == depth_frame_) {
      depth_pub_->publish(*msg);
    }
    else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Unknown CameraInfo frame_id: %s",
        msg->header.frame_id.c_str());
    }
  }

  std::string color_frame_;
  std::string depth_frame_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_pub_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    camera_info_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<CameraInfoSplitter>());

  rclcpp::shutdown();

  return 0;
}