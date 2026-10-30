#ifndef MOTION_2350_HARDWARE__MOTION_2350_SYSTEM_HPP_
#define MOTION_2350_HARDWARE__MOTION_2350_SYSTEM_HPP_

#include <vector>
#include <string>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace motion_2350_hardware
{
class Motion2350SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Motion2350SystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void parse_json_line(const std::string & line);

  // Trạng thái 4 bánh xe: [0]=LF, [1]=RF, [2]=LB, [3]=RB
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Cấu hình Serial
  std::string port_ = "/dev/ttyACM0";
  int baud_rate_ = 115200;
  int serial_fd_ = -1;
  std::string rx_buffer_;

  // Node nội bộ publish thẳng /imu/data (cho ekf.yaml) và /ultrasonic/range (HC-SR04)
  std::shared_ptr<rclcpp::Node> sensor_node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
};
}  // namespace motion_2350_hardware

#endif  // MOTION_2350_HARDWARE__MOTION_2350_SYSTEM_HPP_