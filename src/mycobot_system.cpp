#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mycobot_hardware
{
constexpr double kPi = 3.14159265358979323846;
using Joints = std::array<double, 6>;  // radians, indexed by servo (0..5)

// Conservative per-servo limits in radians (about 168, 135, 150, 145, 165, 180 deg).
constexpr Joints kLimits = {2.93, 2.35, 2.62, 2.53, 2.88, 3.14};

// 1) Driver interface: the ONLY part that knows how to talk to a specific arm.
class ArmDriver
{
public:
  virtual ~ArmDriver() = default;
  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool read_angles(Joints & out, double dt) = 0;
  virtual bool write_angles(const Joints & cmd) = 0;
};

// Fake arm: moves toward the target at a limited speed, like a real one.
class MockDriver : public ArmDriver
{
public:
  bool open() override { pos_.fill(0.0); target_.fill(0.0); return true; }
  void close() override {}
  bool read_angles(Joints & out, double dt) override
  {
    const double max_step = kMaxSpeed * dt;
    for (size_t i = 0; i < 6; ++i) {
      pos_[i] += std::clamp(target_[i] - pos_[i], -max_step, max_step);
    }
    out = pos_;
    return true;
  }
  bool write_angles(const Joints & cmd) override { target_ = cmd; return true; }

private:
  static constexpr double kMaxSpeed = 1.0;  // rad/s
  Joints pos_{};
  Joints target_{};
};

// Real myCobot over serial. Frame format: FE FE <len> <cmd> <data...> FA
// where <len> = number of data bytes + 2. Angles are int16, big-endian, 0.01 deg.
// UNTESTED: verify against Elephant Robotics' protocol docs / pymycobot first.
class SerialDriver : public ArmDriver
{
public:
  SerialDriver(std::string port, int baud, int speed)
  : port_(std::move(port)), baud_(baud), speed_(speed) {}
  ~SerialDriver() override { close(); }

  bool open() override
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {return false;}
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {close(); return false;}
    cfmakeraw(&tty);
    const speed_t s = (baud_ == 1000000) ? B1000000 : B115200;
    cfsetispeed(&tty, s);
    cfsetospeed(&tty, s);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {close(); return false;}
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close() override
  {
    if (fd_ >= 0) {::close(fd_); fd_ = -1;}
  }

  bool read_angles(Joints & out, double) override
  {
    const uint8_t req[] = {0xFE, 0xFE, 0x02, kGetAngles, 0xFA};
    tcflush(fd_, TCIFLUSH);
    if (::write(fd_, req, sizeof(req)) != static_cast<ssize_t>(sizeof(req))) {return false;}
    std::vector<uint8_t> data;
    if (!read_frame(kGetAngles, data) || data.size() != 12) {return false;}
    for (size_t i = 0; i < 6; ++i) {
      const uint16_t u = static_cast<uint16_t>((data[2 * i] << 8) | data[2 * i + 1]);
      const int16_t raw = static_cast<int16_t>(u);
      out[i] = (raw / 100.0) * kPi / 180.0;
    }
    return true;
  }

  bool write_angles(const Joints & cmd) override
  {
    std::vector<uint8_t> f = {0xFE, 0xFE, 0x0F, kSendAngles};
    for (double rad : cmd) {
      const int16_t raw = static_cast<int16_t>(std::lround(rad * 180.0 / kPi * 100.0));
      f.push_back(static_cast<uint8_t>((raw >> 8) & 0xFF));
      f.push_back(static_cast<uint8_t>(raw & 0xFF));
    }
    f.push_back(static_cast<uint8_t>(speed_));
    f.push_back(0xFA);
    return ::write(fd_, f.data(), f.size()) == static_cast<ssize_t>(f.size());
  }

private:
  static constexpr uint8_t kGetAngles = 0x20;
  static constexpr uint8_t kSendAngles = 0x22;

  bool read_byte(uint8_t & b, int timeout_ms)
  {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd_, &set);
    timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    if (select(fd_ + 1, &set, nullptr, nullptr, &tv) <= 0) {return false;}
    return ::read(fd_, &b, 1) == 1;
  }

  bool read_frame(uint8_t cmd, std::vector<uint8_t> & data)
  {
    uint8_t b = 0;
    int sync = 0;
    for (int i = 0; i < 64 && sync < 2; ++i) {   // find FE FE
      if (!read_byte(b, 30)) {return false;}
      sync = (b == 0xFE) ? sync + 1 : 0;
    }
    if (sync < 2) {return false;}
    uint8_t len = 0, got_cmd = 0, footer = 0;
    if (!read_byte(len, 30) || !read_byte(got_cmd, 30) || len < 2) {return false;}
    data.resize(len - 2);
    for (auto & d : data) {
      if (!read_byte(d, 30)) {return false;}
    }
    if (!read_byte(footer, 30) || footer != 0xFA) {return false;}
    return got_cmd == cmd;
  }

  std::string port_;
  int baud_;
  int speed_;
  int fd_ = -1;
};

// 2) The ros2_control hardware interface
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

static std::string param(
  const std::unordered_map<std::string, std::string> & m, const std::string & key,
  const std::string & fallback)
{
  auto it = m.find(key);
  return it == m.end() ? fallback : it->second;
}

class MyCobotSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
    const auto & info = params.hardware_info;

    if (info.joints.size() != 6) {
      RCLCPP_ERROR(get_logger(), "Expected 6 joints, got %zu", info.joints.size());
      return CallbackReturn::ERROR;
    }

    // Map each URDF joint name -> servo id (the "joint ID to motor ID" table).
    std::array<bool, 6> seen{};
    for (size_t i = 0; i < 6; ++i) {
      const auto & j = info.joints[i];
      names_[i] = j.name;

      const std::string id_text = param(j.parameters, "servo_id", "");
      int id = 0;
      try {id = std::stoi(id_text);} catch (...) {id = 0;}
      if (id < 1 || id > 6 || seen[id - 1]) {
        RCLCPP_ERROR(get_logger(), "Joint '%s' needs a unique servo_id 1..6", j.name.c_str());
        return CallbackReturn::ERROR;
      }
      seen[id - 1] = true;
      servo_index_[i] = static_cast<size_t>(id - 1);

      if (j.command_interfaces.size() != 1 || j.command_interfaces[0].name != "position" ||
        j.state_interfaces.size() != 1 || j.state_interfaces[0].name != "position")
      {
        RCLCPP_ERROR(get_logger(), "Joint '%s' must have only a position command and state",
          j.name.c_str());
        return CallbackReturn::ERROR;
      }
    }

    const auto & hp = info.hardware_parameters;
    driver_name_ = param(hp, "driver", "mock");
    port_ = param(hp, "port", "/dev/ttyUSB0");
    baud_ = std::stoi(param(hp, "baud", "115200"));
    speed_ = std::clamp(std::stoi(param(hp, "speed", "30")), 1, 50);  // capped for safety

    if (driver_name_ != "mock" && driver_name_ != "serial") {
      RCLCPP_ERROR(get_logger(), "driver must be 'mock' or 'serial'");
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    if (driver_name_ == "serial") {
      driver_ = std::make_unique<SerialDriver>(port_, baud_, speed_);
    } else {
      driver_ = std::make_unique<MockDriver>();
    }
    if (!driver_->open()) {
      RCLCPP_ERROR(get_logger(), "Could not open driver '%s' (port %s)",
        driver_name_.c_str(), port_.c_str());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Driver '%s' opened", driver_name_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    if (driver_) {driver_->close();}
    return CallbackReturn::SUCCESS;
  }

  // Start commanding from where the arm actually is, so it never jumps.
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    Joints now{};
    if (!driver_->read_angles(now, 0.0)) {
      RCLCPP_ERROR(get_logger(), "Could not read the arm's starting pose");
      return CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < 6; ++i) {
      const double v = now[servo_index_[i]];
      set_state(names_[i] + "/position", v);
      set_command(names_[i] + "/position", v);
    }
    read_failures_ = 0;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    Joints now{};
    if (!driver_->read_angles(now, period.seconds())) {
      if (++read_failures_ > kMaxFailures) {
        RCLCPP_ERROR(get_logger(), "Lost communication with the arm");
        return return_type::ERROR;
      }
      return return_type::OK;  // tolerate a few dropped packets, keep last state
    }
    read_failures_ = 0;
    for (size_t i = 0; i < 6; ++i) {
      set_state(names_[i] + "/position", now[servo_index_[i]]);
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    Joints cmd{};
    for (size_t i = 0; i < 6; ++i) {
      const double v = get_command(names_[i] + "/position");
      if (i == 0) {RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "write(): joint0 command = %f", v);}
      if (!std::isfinite(v)) {return return_type::OK;}  // never send NaN to hardware
      const size_t k = servo_index_[i];
      cmd[k] = std::clamp(v, -kLimits[k], kLimits[k]);
    }
    if (!driver_->write_angles(cmd)) {
      RCLCPP_ERROR(get_logger(), "Failed to send command to the arm");
      return return_type::ERROR;
    }
    return return_type::OK;
  }

private:
  static constexpr int kMaxFailures = 10;

  std::array<std::string, 6> names_;
  std::array<size_t, 6> servo_index_{};
  std::string driver_name_, port_;
  int baud_ = 115200;
  int speed_ = 30;
  int read_failures_ = 0;
  std::unique_ptr<ArmDriver> driver_;
};

}  // namespace mycobot_hardware

PLUGINLIB_EXPORT_CLASS(mycobot_hardware::MyCobotSystem, hardware_interface::SystemInterface)
