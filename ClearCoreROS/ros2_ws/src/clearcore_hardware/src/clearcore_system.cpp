/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */
/*
 * ros2_control SystemInterface for ClearCoreROS.
 * Linux only. Wire format: firmware/RosProtocol.h.
 */

#include "clearcore_hardware/clearcore_system.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <sstream>
#include <string>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace clearcore_hardware
{
namespace
{
constexpr uint8_t kMagic = 0xC5;
constexpr uint8_t kVersion = 1;
constexpr uint8_t kTypeState = 1;
constexpr uint8_t kTypePosition = 2;
constexpr uint8_t kTypeVelocity = 3;
constexpr uint8_t kTypeHeartbeat = 4;
constexpr uint8_t kFlagWatchdog = 0x10;

void put_u16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xff);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xff);
}

void put_f32(uint8_t * p, float v)
{
  uint32_t u = 0;
  std::memcpy(&u, &v, sizeof(u));
  p[0] = static_cast<uint8_t>(u & 0xff);
  p[1] = static_cast<uint8_t>((u >> 8) & 0xff);
  p[2] = static_cast<uint8_t>((u >> 16) & 0xff);
  p[3] = static_cast<uint8_t>((u >> 24) & 0xff);
}

uint16_t get_u16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t get_u32(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

float get_f32(const uint8_t * p)
{
  uint32_t u = get_u32(p);
  float v = 0.f;
  std::memcpy(&v, &u, sizeof(v));
  return v;
}

int connect_tcp(const std::string & host, int port, bool nonblock)
{
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return -1;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    ::close(fd);
    return -1;
  }
  if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    return -1;
  }
  const int one = 1;
  ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  if (nonblock) {
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  }
  return fd;
}
}  // namespace

ClearCoreSystemHardware::~ClearCoreSystemHardware()
{
  close_all();
}

void ClearCoreSystemHardware::close_all()
{
  if (session_fd_ >= 0) {
    ::close(session_fd_);
    session_fd_ = -1;
  }
  if (stream_fd_ >= 0) {
    ::close(stream_fd_);
    stream_fd_ = -1;
  }
}

std::string ClearCoreSystemHardware::param(
  const std::string & key, const std::string & fallback) const
{
  const auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end() || it->second.empty()) {
    return fallback;
  }
  return it->second;
}

hardware_interface::CallbackReturn ClearCoreSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  host_ = param("host", host_);
  session_port_ = std::stoi(param("session_port", "9200"));
  stream_port_ = std::stoi(param("stream_port", "9201"));
  axis_mask_ = std::stoi(param("axis_mask", "3"));
  velocity_stream_ = param("stream_mode", "position") == "velocity";

  const char * names[4] = {"joint_x", "joint_y", "joint_z", "joint_a"};
  for (const auto & joint : info_.joints) {
    int axis = -1;
    for (int a = 0; a < 4; ++a) {
      if (joint.name == names[a]) {
        axis = a;
      }
    }
    if (axis < 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("clearcore_system"),
        "joint '%s' is not joint_x/y/z/a", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    bool has_position = false;
    for (const auto & cmd : joint.command_interfaces) {
      if (cmd.name == hardware_interface::HW_IF_POSITION) {
        has_position = true;
      }
    }
    if (!has_position) {
      RCLCPP_ERROR(
        rclcpp::get_logger("clearcore_system"),
        "joint '%s' needs a position command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    axis_of_joint_.push_back(axis);
  }
  if (axis_of_joint_.empty()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto n = axis_of_joint_.size();
  hw_cmd_.assign(n, 0.0);
  hw_pos_.assign(n, 0.0);
  hw_vel_.assign(n, 0.0);
  hw_eff_.assign(n, 0.0);
  last_cmd_.assign(n, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ClearCoreSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    states.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_pos_[i]);
    states.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]);
    states.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_eff_[i]);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface>
ClearCoreSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    cmds.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_cmd_[i]);
  }
  return cmds;
}

bool ClearCoreSystemHardware::session_call(
  const std::string & method, const std::string & params_json)
{
  std::ostringstream msg;
  msg << "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"" << method << "\"";
  if (!params_json.empty()) {
    msg << ",\"params\":" << params_json;
  }
  msg << "}\n";
  const std::string line = msg.str();
  size_t sent = 0;
  while (sent < line.size()) {
    const ssize_t n = ::send(session_fd_, line.data() + sent, line.size() - sent, MSG_NOSIGNAL);
    if (n <= 0) {
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  std::string reply;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (reply.find('\n') == std::string::npos) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(session_fd_, &fds);
    timeval tv{0, 200000};
    const int ready = ::select(session_fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ready < 0) {
      return false;
    }
    if (ready == 0) {
      continue;
    }
    char tmp[512];
    const ssize_t n = ::recv(session_fd_, tmp, sizeof(tmp), 0);
    if (n <= 0) {
      return false;
    }
    reply.append(tmp, tmp + n);
  }
  return reply.find("\"error\"") == std::string::npos;
}

hardware_interface::CallbackReturn ClearCoreSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  close_all();
  session_fd_ = connect_tcp(host_, session_port_, false);
  if (session_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("clearcore_system"), "session connect %s:%d failed",
      host_.c_str(), session_port_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::ostringstream cfg;
  cfg << "{\"axis_mask\":" << axis_mask_
      << ",\"steps_per_rev\":" << param("steps_per_rev", "800")
      << ",\"pitch_mm\":" << param("pitch_mm", "5.0")
      << ",\"vel_steps\":" << param("vel_steps", "27000")
      << ",\"accel_steps\":" << param("accel_steps", "250000")
      << ",\"decel_steps\":" << param("decel_steps", param("accel_steps", "250000"))
      << ",\"watchdog_ms\":" << param("watchdog_ms", "500") << "}";
  if (!session_call("disable", "") || !session_call("configure", cfg.str()) ||
      !session_call("enable", ""))
  {
    RCLCPP_ERROR(rclcpp::get_logger("clearcore_system"), "enable/configure rejected");
    close_all();
    return hardware_interface::CallbackReturn::ERROR;
  }
  stream_fd_ = connect_tcp(host_, stream_port_, true);
  if (stream_fd_ < 0) {
    close_all();
    return hardware_interface::CallbackReturn::ERROR;
  }
  have_state_ = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!have_state_ && std::chrono::steady_clock::now() < deadline) {
    if (drain_stream() != hardware_interface::return_type::OK) {
      close_all();
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!have_state_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  if (!have_state_) {
    RCLCPP_ERROR(rclcpp::get_logger("clearcore_system"), "no joint state from stream");
    close_all();
    return hardware_interface::CallbackReturn::ERROR;
  }
  hw_cmd_ = hw_pos_;
  last_cmd_ = hw_pos_;
  stable_cycles_ = 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ClearCoreSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (session_fd_ >= 0) {
    session_call("disable", "");
  }
  close_all();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ClearCoreSystemHardware::drain_stream()
{
  uint8_t tmp[256];
  while (true) {
    const ssize_t n = ::recv(stream_fd_, tmp, sizeof(tmp), MSG_DONTWAIT);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      return hardware_interface::return_type::ERROR;
    }
    if (n == 0) {
      return hardware_interface::return_type::ERROR;
    }
    rx_.insert(rx_.end(), tmp, tmp + n);
  }
  size_t off = 0;
  while (rx_.size() - off >= 6) {
    if (rx_[off] != kMagic) {
      ++off;
      continue;
    }
    if (rx_[off + 1] != kVersion) {
      ++off;
      continue;
    }
    const uint8_t type = rx_[off + 2];
    const uint16_t plen = get_u16(&rx_[off + 4]);
    if (plen > 60 || (type == kTypeState && plen != 60)) {
      ++off;
      continue;
    }
    if (rx_.size() - off < static_cast<size_t>(6 + plen)) {
      break;
    }
    if (type == kTypeState) {
      const uint8_t * p = &rx_[off + 6];
      const uint8_t flags = p[6];
      if (flags & kFlagWatchdog) {
        session_call("keepalive", "");
      }
      for (size_t i = 0; i < axis_of_joint_.size(); ++i) {
        const int axis = axis_of_joint_[i];
        hw_pos_[i] = get_f32(p + 12 + (axis * 4));
        hw_vel_[i] = get_f32(p + 28 + (axis * 4));
        hw_eff_[i] = get_f32(p + 44 + (axis * 4));
      }
      have_state_ = true;
    }
    off += static_cast<size_t>(6 + plen);
  }
  if (off > 0) {
    rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(off));
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ClearCoreSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (stream_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }
  return drain_stream();
}

bool ClearCoreSystemHardware::send_frame(const uint8_t * data, size_t n)
{
  size_t sent = 0;
  int spins = 0;
  while (sent < n) {
    const ssize_t w = ::send(stream_fd_, data + sent, n - sent, MSG_NOSIGNAL | MSG_DONTWAIT);
    if (w < 0) {
      if ((errno == EAGAIN || errno == EWOULDBLOCK) && spins < 5) {
        ++spins;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      return false;
    }
    sent += static_cast<size_t>(w);
  }
  return true;
}

hardware_interface::return_type ClearCoreSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (stream_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }
  float q[4] = {0, 0, 0, 0};
  float v[4] = {0, 0, 0, 0};
  uint8_t mask = 0;
  bool changed = false;
  const double dt = period.seconds() > 1e-4 ? period.seconds() : 0.02;
  for (size_t i = 0; i < axis_of_joint_.size(); ++i) {
    const int axis = axis_of_joint_[i];
    q[axis] = static_cast<float>(hw_cmd_[i]);
    v[axis] = static_cast<float>((hw_cmd_[i] - last_cmd_[i]) / dt);
    if (std::fabs(hw_cmd_[i] - last_cmd_[i]) > 1e-9) {
      changed = true;
    }
    mask = static_cast<uint8_t>(mask | (1u << axis));
  }

  uint8_t frame[32];
  frame[0] = kMagic;
  frame[1] = kVersion;
  frame[3] = 0;
  const bool land = velocity_stream_ && !changed && stable_cycles_ >= 2;
  if (velocity_stream_ && changed) {
    frame[2] = kTypeVelocity;
    put_u16(frame + 4, 20);
    put_u16(frame + 6, seq_);
    frame[8] = mask;
    frame[9] = 0;
    for (int a = 0; a < 4; ++a) {
      put_f32(frame + 10 + (a * 4), v[a]);
    }
    stable_cycles_ = 0;
    last_cmd_ = hw_cmd_;
  } else if (!velocity_stream_ || land) {
    frame[2] = kTypePosition;
    put_u16(frame + 4, 20);
    put_u16(frame + 6, seq_);
    frame[8] = mask;
    frame[9] = 0;
    for (int a = 0; a < 4; ++a) {
      put_f32(frame + 10 + (a * 4), q[a]);
    }
    if (land) {
      stable_cycles_ = 0;
    }
    last_cmd_ = hw_cmd_;
  } else {
    frame[2] = kTypeHeartbeat;
    put_u16(frame + 4, 2);
    put_u16(frame + 6, seq_);
    ++stable_cycles_;
  }
  const size_t n = (frame[2] == kTypeHeartbeat) ? 8u : 26u;
  seq_ = static_cast<uint16_t>(seq_ + 1);
  if (!send_frame(frame, n)) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace clearcore_hardware

PLUGINLIB_EXPORT_CLASS(
  clearcore_hardware::ClearCoreSystemHardware, hardware_interface::SystemInterface)
