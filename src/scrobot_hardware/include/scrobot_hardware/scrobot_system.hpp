#ifndef SCROBOT_HARDWARE__SCROBOT_SYSTEM_HPP_
#define SCROBOT_HARDWARE__SCROBOT_SYSTEM_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "scrobot_interfaces/msg/collector_command.hpp"
#include "scrobot_interfaces/msg/hardware_status.hpp"

namespace scrobot_hardware
{

class ScrobotSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ~ScrobotSystemHardware() override;

private:
  bool open_serial();
  void close_serial();
  bool configure_serial(int baud_rate);

  bool send_drive_frame(double right_rpm, double left_rpm);
  bool send_aux_frame(double brush_right_rpm, double brush_left_rpm, double conveyor_rpm);
  bool write_all(const uint8_t * data, size_t size);

  void process_rx_buffer();
  bool process_frame(const uint8_t * frame, size_t size);
  void publish_status(bool force = false);

  static uint8_t crc8(const uint8_t * data, size_t size);
  static float read_float_le(const uint8_t * data);
  static void write_float_le(uint8_t * data, float value);
  static size_t expected_frame_length(uint8_t type);
  static double rpm_to_rad_s(double rpm);
  static double rad_s_to_rpm(double rad_s);

  int serial_fd_{-1};
  std::string serial_port_{"/dev/scrobot_mcu"};
  int baud_rate_{1000000};
  double max_wheel_rpm_{200.0};
  double max_brush_rpm_{400.0};
  double max_conveyor_rpm_{600.0};
  int state_timeout_ms_{250};
  int collector_timeout_ms_{500};

  bool active_{false};
  bool connected_{false};
  bool estop_{false};
  uint32_t rx_frame_count_{0};
  std::vector<uint8_t> rx_buffer_;

  std::mutex collector_mutex_;
  scrobot_interfaces::msg::CollectorCommand collector_command_;
  std::chrono::steady_clock::time_point last_collector_command_{};
  std::chrono::steady_clock::time_point last_state_{};
  std::chrono::steady_clock::time_point last_status_publish_{};

  rclcpp::Subscription<scrobot_interfaces::msg::CollectorCommand>::SharedPtr collector_sub_;
  rclcpp::Publisher<scrobot_interfaces::msg::HardwareStatus>::SharedPtr status_pub_;
};

}  // namespace scrobot_hardware

#endif  // SCROBOT_HARDWARE__SCROBOT_SYSTEM_HPP_
