#ifndef SCROBOT_HARDWARE__SCROBOT_SYSTEM_HPP_
#define SCROBOT_HARDWARE__SCROBOT_SYSTEM_HPP_

#include <chrono>
#include <cstdint>
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

  bool send_packet(uint8_t type, const std::vector<uint8_t> & payload = {});
  bool send_setpoint(
    double wr_rpm,
    double wl_rpm,
    double br_rpm,
    double bl_rpm,
    double cv_rpm);
  bool send_arm();
  bool send_disarm();
  bool send_info_request();

  bool poll_serial();
  void parse_rx_buffer();
  void handle_frame(
    uint8_t version,
    uint8_t type,
    uint16_t sequence,
    const uint8_t * payload,
    uint8_t payload_len);

  void handle_feedback(
    uint16_t sequence,
    const uint8_t * payload,
    uint8_t payload_len);
  void handle_info_response(const uint8_t * payload, uint8_t payload_len);
  void handle_error_frame(const uint8_t * payload, uint8_t payload_len);

  void publish_status(bool force = false);
  void reset_protocol_state();

  static uint16_t crc16_ccitt_false(const uint8_t * data, size_t len);
  static uint16_t read_u16_le(const uint8_t * data);
  static uint32_t read_u32_le(const uint8_t * data);
  static int32_t read_i32_le(const uint8_t * data);
  static float read_float_le(const uint8_t * data);
  static void append_float_le(std::vector<uint8_t> & payload, float value);
  static int64_t wrapped_count_delta(int32_t current, int32_t previous);
  static double rpm_to_rad_s(double rpm);
  static double rad_s_to_rpm(double rad_s);

  int serial_fd_{-1};
  std::string serial_port_{"/dev/ttyAMA0"};
  int baud_rate_{1000000};

  double counts_per_wheel_rev_{3264.0};
  double left_count_sign_{1.0};
  double right_count_sign_{-1.0};
  double left_command_sign_{1.0};
  double right_command_sign_{1.0};

  double max_wheel_rpm_{100.0};
  double max_brush_rpm_{400.0};
  double max_conveyor_rpm_{80.0};

  int feedback_timeout_ms_{250};
  int collector_timeout_ms_{500};
  int handshake_timeout_ms_{750};
  int arm_timeout_ms_{300};

  bool active_{false};
  bool connected_{false};
  bool armed_{false};
  bool estop_{false};
  bool comm_timeout_{true};
  bool sysid_{false};
  bool uart_error_{false};
  bool invalid_output_{false};
  bool tx_queue_drop_{false};
  bool invalid_command_{false};

  uint32_t status_flags_{0};
  uint32_t control_tick_{0};
  uint16_t last_setpoint_seq_{0};
  uint16_t feedback_sequence_{0};
  uint16_t tx_sequence_{0};

  uint8_t firmware_major_{0};
  uint8_t firmware_minor_{0};
  uint8_t firmware_patch_{0};
  uint8_t protocol_version_{0};
  bool info_received_{false};

  bool wheel_counts_initialized_{false};
  int32_t previous_wr_count_{0};
  int32_t previous_wl_count_{0};
  int64_t accumulated_wr_count_{0};
  int64_t accumulated_wl_count_{0};

  double feedback_br_rpm_{0.0};
  double feedback_bl_rpm_{0.0};
  double feedback_cv_rpm_{0.0};

  std::vector<uint8_t> rx_buffer_;

  std::mutex collector_mutex_;
  scrobot_interfaces::msg::CollectorCommand collector_command_;
  std::chrono::steady_clock::time_point last_collector_command_{};
  std::chrono::steady_clock::time_point last_feedback_{};
  std::chrono::steady_clock::time_point last_status_publish_{};

  rclcpp::Subscription<scrobot_interfaces::msg::CollectorCommand>::SharedPtr collector_sub_;
  rclcpp::Publisher<scrobot_interfaces::msg::HardwareStatus>::SharedPtr status_pub_;
};

}  // namespace scrobot_hardware

#endif  // SCROBOT_HARDWARE__SCROBOT_SYSTEM_HPP_
