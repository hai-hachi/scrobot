#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

class ImuTransformerSensorQos : public rclcpp::Node
{
public:
  ImuTransformerSensorQos()
  : Node("imu_transformer_sensor_qos"),
    target_frame_(declare_parameter<std::string>("target_frame", "base_link")),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // RealSense motion/combined-IMU topics are published with SENSOR_DATA
    // (best effort). Use SensorDataQoS on the input so the subscription is
    // compatible, then republish transformed data reliably for downstream
    // filters and command-line inspection.
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu_out", rclcpp::QoS(10));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_in",
      rclcpp::SensorDataQoS(),
      std::bind(&ImuTransformerSensorQos::imu_callback, this, std::placeholders::_1));
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    try {
      // The camera-to-base transform is static in the robot description, so
      // using the latest available transform avoids timestamp synchronization
      // issues while preserving the original sensor timestamp below.
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, msg->header.frame_id, tf2::TimePointZero);

      sensor_msgs::msg::Imu out;
      tf2::doTransform(*msg, out, transform);

      out.header.stamp = msg->header.stamp;
      out.header.frame_id = target_frame_;
      imu_pub_->publish(out);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU transform %s -> %s unavailable: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
    }
  }

  std::string target_frame_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuTransformerSensorQos>());
  rclcpp::shutdown();
  return 0;
}
