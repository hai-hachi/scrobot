#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

class DepthObstacleScan : public rclcpp::Node
{
public:
  DepthObstacleScan()
  : Node("depth_obstacle_scan"),
    target_frame_(declare_parameter<std::string>("target_frame", "base_footprint")),
    min_height_(declare_parameter<double>("min_height", 0.08)),
    max_height_(declare_parameter<double>("max_height", 0.70)),
    range_min_(declare_parameter<double>("range_min", 0.20)),
    range_max_(declare_parameter<double>("range_max", 3.00)),
    angle_min_(declare_parameter<double>("angle_min", -0.80)),
    angle_max_(declare_parameter<double>("angle_max", 0.80)),
    angle_increment_(declare_parameter<double>("angle_increment", 0.00872664626)),
    row_stride_(std::max(1, static_cast<int>(declare_parameter<int64_t>("row_stride", 4)))),
    col_stride_(std::max(1, static_cast<int>(declare_parameter<int64_t>("col_stride", 2)))),
    depth_scale_(declare_parameter<double>("depth_scale", 0.001)),
    scan_time_(declare_parameter<double>("scan_time", 1.0 / 15.0)),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto qos = rclcpp::SensorDataQoS();

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", qos,
      std::bind(&DepthObstacleScan::info_callback, this, std::placeholders::_1));

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "depth", qos,
      std::bind(&DepthObstacleScan::depth_callback, this, std::placeholders::_1));

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(5));

    const int bins = static_cast<int>(
      std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1;
    scan_bins_ = std::max(1, bins);

    RCLCPP_INFO(
      get_logger(),
      "Depth obstacle scan: target=%s height=[%.2f, %.2f] m range=[%.2f, %.2f] m "
      "stride=%dx%d bins=%d",
      target_frame_.c_str(), min_height_, max_height_, range_min_, range_max_,
      row_stride_, col_stride_, scan_bins_);
  }

private:
  void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_info_ = msg;
    ++camera_info_count_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "CameraInfo received: %lu messages, %ux%u",
      static_cast<unsigned long>(camera_info_count_), msg->width, msg->height);
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    ++depth_count_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Depth received: %lu messages, encoding=%s, frame=%s",
      static_cast<unsigned long>(depth_count_), msg->encoding.c_str(),
      msg->header.frame_id.c_str());

    if (!camera_info_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for depth camera_info");
      return;
    }

    if (msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
        msg->encoding != "16UC1") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unsupported depth encoding '%s'; expected 16UC1", msg->encoding.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(
        target_frame_, msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Depth transform %s -> %s unavailable: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    tf2::Transform tf;
    tf2::fromMsg(tf_msg.transform, tf);

    const double fx = camera_info_->k[0];
    const double fy = camera_info_->k[4];
    const double cx = camera_info_->k[2];
    const double cy = camera_info_->k[5];

    if (fx <= 0.0 || fy <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Invalid camera intrinsics");
      return;
    }

    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp = msg->header.stamp;
    scan.header.frame_id = target_frame_;
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(angle_min_ + (scan_bins_ - 1) * angle_increment_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.time_increment = 0.0f;
    scan.scan_time = static_cast<float>(scan_time_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges.assign(scan_bins_, std::numeric_limits<float>::infinity());

    for (uint32_t v = 0; v < msg->height; v += static_cast<uint32_t>(row_stride_)) {
      const auto * row = reinterpret_cast<const uint16_t *>(
        msg->data.data() + static_cast<size_t>(v) * msg->step);

      const double yn = (static_cast<double>(v) - cy) / fy;

      for (uint32_t u = 0; u < msg->width; u += static_cast<uint32_t>(col_stride_)) {
        const uint16_t raw = row[u];
        if (raw == 0) {
          continue;
        }

        const double z_opt = static_cast<double>(raw) * depth_scale_;
        if (z_opt <= 0.0 || z_opt > range_max_ + 1.0) {
          continue;
        }

        // RealSense optical coordinates: +X right, +Y down, +Z forward.
        const double x_opt = (static_cast<double>(u) - cx) / fx * z_opt;
        const double y_opt = yn * z_opt;

        const tf2::Vector3 p_target = tf * tf2::Vector3(x_opt, y_opt, z_opt);

        const double h = p_target.z();
        if (h < min_height_ || h > max_height_) {
          continue;
        }

        const double x = p_target.x();
        const double y = p_target.y();
        const double range = std::hypot(x, y);
        if (range < range_min_ || range > range_max_) {
          continue;
        }

        const double angle = std::atan2(y, x);
        if (angle < angle_min_ || angle > angle_max_) {
          continue;
        }

        const int bin = static_cast<int>(
          std::lround((angle - angle_min_) / angle_increment_));
        if (bin < 0 || bin >= scan_bins_) {
          continue;
        }

        auto & current = scan.ranges[static_cast<size_t>(bin)];
        if (range < current) {
          current = static_cast<float>(range);
        }
      }
    }

    scan_pub_->publish(scan);
    ++scan_count_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Published scan: %lu messages",
      static_cast<unsigned long>(scan_count_));
  }

  std::string target_frame_;
  double min_height_;
  double max_height_;
  double range_min_;
  double range_max_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  int row_stride_;
  int col_stride_;
  double depth_scale_;
  double scan_time_;
  int scan_bins_;

  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  uint64_t camera_info_count_{0};
  uint64_t depth_count_{0};
  uint64_t scan_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthObstacleScan>());
  rclcpp::shutdown();
  return 0;
}
