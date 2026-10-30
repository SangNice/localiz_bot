#include "motion_2350_hardware/motion_2350_system.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace motion_2350_hardware
{
hardware_interface::CallbackReturn Motion2350SystemHardware::on_init(const HardwareComponentInterfaceParams & params))
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Khởi tạo vector cho 4 bánh xe (LF, RF, LB, RB)
  hw_positions_.assign(4, 0.0);
  hw_velocities_.assign(4, 0.0);
  hw_commands_.assign(4, 0.0);

  // Lấy thông số cổng Serial từ file Xacro
  port_ = info_.hardware_parameters["device"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  // Tạo một Node nội bộ để đẩy dữ liệu MPU6050 thẳng lên topic /imu/data
  imu_node_ = std::make_shared<rclcpp::Node>("motion_2350_imu_node");
  imu_pub_ = imu_node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Motion2350SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Motion2350SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn Motion2350SystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // --- CODE MỞ CỔNG SERIAL (Linux POSIX) ---
  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("Motion2350System"), "Failed to open serial port %s", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Cấu hình termios cho baud_rate tại đây...
  
  RCLCPP_INFO(rclcpp::get_logger("Motion2350System"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Motion2350SystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  close(serial_fd_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Motion2350SystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. Đọc dữ liệu từ Serial (Ví dụ Cytron gửi lên: "DATA,pos1,pos2,pos3,pos4,gz\n")
  // Bạn viết hàm đọc chuỗi ký tự từ serial_fd_ tại đây.
  
  // 2. Parse chuỗi và cập nhật vào mảng (Giả mã):
  // hw_positions_[0] = parse_lf_pos;
  // hw_positions_[1] = parse_rf_pos;
  // ...
  
  // 3. Đọc góc gz (từ MPU6050) và publish IMU cho EKF:
  // float gz_rad = parse_gz; 
  // sensor_msgs::msg::Imu imu_msg;
  // imu_msg.header.stamp = imu_node_->now();
  // imu_msg.header.frame_id = "base_link";
  // imu_msg.angular_velocity.z = gz_rad;
  // imu_pub_->publish(imu_msg);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Motion2350SystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // ROS 2 đã tính toán và điền vận tốc muốn chạy vào hw_commands_
  // Bạn gói nó thành chuỗi gửi xuống Cytron RP2350.
  
  char cmd_str[256];
  sprintf(cmd_str, "CMD,%.2f,%.2f,%.2f,%.2f\n", hw_commands_[0], hw_commands_[1], hw_commands_[2], hw_commands_[3]);
  
  ::write(serial_fd_, cmd_str, strlen(cmd_str));

  return hardware_interface::return_type::OK;
}

}  // namespace motion_2350_hardware

PLUGINLIB_EXPORT_CLASS(motion_2350_hardware::Motion2350SystemHardware, hardware_interface::SystemInterface)