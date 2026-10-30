#ifndef MOTION_2350_HARDWARE__MOTION_2350_SYSTEM_HPP_
#define MOTION_2350_HARDWARE__MOTION_2350_SYSTEM_HPP_

#include <vector>
#include <string>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace motion_2350_hardware
{
class Motion2350SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Motion2350SystemHardware)

  hardware_interface::CallbackReturn on_init(const HardwareComponentInterfaceParams & params & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Khai báo biến lưu trạng thái của 4 bánh xe
  std::vector<double> hw_commands_; // Vận tốc ROS 2 muốn xe chạy
  std::vector<double> hw_positions_; // Góc quay thực tế (từ Encoder)
  std::vector<double> hw_velocities_; // Vận tốc thực tế (từ Encoder)

  // Thông số cổng Serial
  std::string port_;
  int baud_rate_;
  int serial_fd_; // File descriptor cho Serial trên Linux

  // Node ẩn để publish thẳng IMU (Mẹo để không cần viết thêm IMU Broadcaster phức tạp)
  std::shared_ptr<rclcpp::Node> imu_node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};
}  // namespace motion_2350_hardware

#endif  // MOTION_2350_HARDWARE__MOTION_2350_SYSTEM_HPP_