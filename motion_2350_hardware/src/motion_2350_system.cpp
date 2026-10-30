#include "motion_2350_hardware/motion_2350_system.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <cstdio>

namespace motion_2350_hardware
{

hardware_interface::CallbackReturn Motion2350SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Khởi tạo mảng cho 4 bánh xe (LF, RF, LB, RB)
  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);
  hw_commands_.assign(info_.joints.size(), 0.0);

  if (info_.hardware_parameters.count("device")) {
    port_ = info_.hardware_parameters.at("device");
  }
  if (info_.hardware_parameters.count("baud_rate")) {
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  }

  // Tạo Node nội bộ đẩy dữ liệu MPU6050 lên /imu/data (khớp với ekf.yaml) và HC-SR04 lên /ultrasonic/range
  sensor_node_ = std::make_shared<rclcpp::Node>("motion_2350_sensor_node");
  imu_pub_ = sensor_node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  range_pub_ = sensor_node_->create_publisher<sensor_msgs::msg::Range>("/ultrasonic/range", 10);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Motion2350SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Motion2350SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn Motion2350SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ == -1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Motion2350System"),
      "Không thể mở cổng Serial %s. Kiểm tra cáp USB hoặc quyền sudo chmod 666!", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Cấu hình chuẩn Linux POSIX termios (115200, 8N1, Non-blocking Raw mode)
  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Motion2350System"), "Lỗi tcgetattr!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  speed_t speed = B115200;
  if (baud_rate_ == 57600) speed = B57600;
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  // Raw mode (không chờ nhấn Enter, không echo ngược lại mạch)
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  tcflush(serial_fd_, TCIFLUSH);
  tcsetattr(serial_fd_, TCSANOW, &tty);

  RCLCPP_INFO(rclcpp::get_logger("Motion2350System"), "Đã kết nối Motion 2350 Pro tại %s!", port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Motion2350SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_fd_ != -1) {
    const char * stop_cmd = "CMD,0.00,0.00,0.00,0.00\n";
    ::write(serial_fd_, stop_cmd, std::strlen(stop_cmd));
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Hàm tách nhanh chuỗi JSON từ code.py: {"p": [p0, p1, p2, p3], "v": [v0, v1, v2, v3], "gz": 0.01, "d": 19.2}
void Motion2350SystemHardware::parse_json_line(const std::string & line)
{
  auto parse_array4 = [&](const std::string & key, std::vector<double> & target) {
    size_t pos = line.find(key);
    if (pos == std::string::npos) return;
    size_t start = line.find('[', pos);
    size_t end = line.find(']', start);
    if (start == std::string::npos || end == std::string::npos) return;

    std::string arr_str = line.substr(start + 1, end - start - 1);
    std::stringstream ss(arr_str);
    std::string item;
    size_t idx = 0;
    while (std::getline(ss, item, ',') && idx < target.size()) {
      try {
        target[idx++] = std::stod(item);
      } catch (...) {}
    }
  };

  auto parse_scalar = [&](const std::string & key, double & val) -> bool {
    size_t pos = line.find(key);
    if (pos == std::string::npos) return false;
    size_t colon = line.find(':', pos);
    if (colon == std::string::npos) return false;
    size_t end = line.find_first_of(",}", colon + 1);
    if (end == std::string::npos) return false;
    try {
      val = std::stod(line.substr(colon + 1, end - colon - 1));
      return true;
    } catch (...) {
      return false;
    }
  };

  // 1. Cập nhật Position (rad) và Velocity (rad/s) cho 4 bánh xe
  parse_array4("\"p\"", hw_positions_);
  parse_array4("\"v\"", hw_velocities_);

  auto now = sensor_node_->now();

  // 2. Đọc vận tốc góc gz (rad/s) từ MPU6050 và publish lên /imu/data cho ekf.yaml
  double gz_rad = 0.0;
  if (parse_scalar("\"gz\"", gz_rad)) {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = "base_link"; // Khớp trực tiếp với base_link_frame trong ekf.yaml
    imu_msg.angular_velocity.z = gz_rad;
    imu_msg.angular_velocity_covariance[8] = 0.001; // Phương sai nhỏ để EKF tin tưởng vyaw của Gyro
    imu_pub_->publish(imu_msg);
  }

  // 3. Đọc khoảng cách d (cm) từ HC-SR04 và publish lên /ultrasonic/range (đơn vị: mét)
  double dist_cm = 0.0;
  if (parse_scalar("\"d\"", dist_cm)) {
    sensor_msgs::msg::Range range_msg;
    range_msg.header.stamp = now;
    range_msg.header.frame_id = "base_link";
    range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    range_msg.field_of_view = 0.26;
    range_msg.min_range = 0.02;
    range_msg.max_range = 4.00;
    range_msg.range = dist_cm / 100.0;
    range_pub_->publish(range_msg);
  }
}

hardware_interface::return_type Motion2350SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ == -1) return hardware_interface::return_type::ERROR;

  char buf[512];
  ssize_t n = ::read(serial_fd_, buf, sizeof(buf) - 1);
  if (n > 0) {
    buf[n] = '\0';
    rx_buffer_ += buf;

    size_t pos = 0;
    while ((pos = rx_buffer_.find('\n')) != std::string::npos) {
      std::string line = rx_buffer_.substr(0, pos);
      rx_buffer_.erase(0, pos + 1);
      if (!line.empty() && line.front() == '{') {
        parse_json_line(line);
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Motion2350SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ == -1) return hardware_interface::return_type::ERROR;

  char cmd_str[128];
  int len = std::snprintf(
    cmd_str, sizeof(cmd_str), "CMD,%.2f,%.2f,%.2f,%.2f\n",
    hw_commands_[0], hw_commands_[1], hw_commands_[2], hw_commands_[3]);

  if (len > 0) {
    ::write(serial_fd_, cmd_str, len);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace motion_2350_hardware

PLUGINLIB_EXPORT_CLASS(
  motion_2350_hardware::Motion2350SystemHardware,
  hardware_interface::SystemInterface)