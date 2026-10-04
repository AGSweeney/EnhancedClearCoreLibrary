/* ClearCoreSystemHardware against localhost session and stream sockets. */

#include "clearcore_hardware/clearcore_system.hpp"
#include "RosProtocol.h"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
int listen_loopback()
{
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  EXPECT_GE(fd, 0);
  int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  EXPECT_EQ(::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0);
  EXPECT_EQ(::listen(fd, 4), 0);
  return fd;
}

int bound_port(int fd)
{
  sockaddr_in addr{};
  socklen_t len = sizeof(addr);
  ::getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &len);
  return ntohs(addr.sin_port);
}

std::string extract_method(const std::string & line)
{
  const std::string key = "\"method\":\"";
  const auto at = line.find(key);
  if (at == std::string::npos) {
    return {};
  }
  const auto start = at + key.size();
  const auto end = line.find('"', start);
  if (end == std::string::npos) {
    return {};
  }
  return line.substr(start, end - start);
}

struct MockBoard
{
  int session_listen{-1};
  int stream_listen{-1};
  std::atomic<bool> stop{false};
  std::atomic<uint8_t> flags{CCROS_FLAG_ENABLED};
  std::atomic<bool> drop_stream{false};
  std::mutex mu;
  std::vector<std::string> methods;
  std::thread session_th;
  std::thread stream_th;

  MockBoard()
  {
    session_listen = listen_loopback();
    stream_listen = listen_loopback();
    session_th = std::thread([this] { session_loop(); });
    stream_th = std::thread([this] { stream_loop(); });
  }

  ~MockBoard() { close(); }

  int session_port() const { return bound_port(session_listen); }
  int stream_port() const { return bound_port(stream_listen); }

  void close()
  {
    stop = true;
    if (session_listen >= 0) {
      ::shutdown(session_listen, SHUT_RDWR);
      ::close(session_listen);
      session_listen = -1;
    }
    if (stream_listen >= 0) {
      ::shutdown(stream_listen, SHUT_RDWR);
      ::close(stream_listen);
      stream_listen = -1;
    }
    if (session_th.joinable()) {
      session_th.join();
    }
    if (stream_th.joinable()) {
      stream_th.join();
    }
  }

  void session_loop()
  {
    while (!stop) {
      sockaddr_in addr{};
      socklen_t len = sizeof(addr);
      const int conn = ::accept(session_listen, reinterpret_cast<sockaddr *>(&addr), &len);
      if (conn < 0) {
        continue;
      }
      std::string buf;
      char tmp[512];
      while (!stop) {
        const ssize_t n = ::recv(conn, tmp, sizeof(tmp), 0);
        if (n <= 0) {
          break;
        }
        buf.append(tmp, tmp + n);
        size_t nl;
        while ((nl = buf.find('\n')) != std::string::npos) {
          const std::string line = buf.substr(0, nl);
          buf.erase(0, nl + 1);
          const std::string method = extract_method(line);
          {
            std::lock_guard<std::mutex> lock(mu);
            methods.push_back(method);
          }
          const char * reply = "{\"jsonrpc\":\"2.0\",\"id\":1,\"result\":{\"ok\":true}}\n";
          ::send(conn, reply, std::strlen(reply), MSG_NOSIGNAL);
        }
      }
      ::close(conn);
    }
  }

  void stream_loop()
  {
    while (!stop) {
      sockaddr_in addr{};
      socklen_t len = sizeof(addr);
      const int conn = ::accept(stream_listen, reinterpret_cast<sockaddr *>(&addr), &len);
      if (conn < 0) {
        continue;
      }
      while (!stop && !drop_stream) {
        CcrosState state{};
        state.flags = flags.load();
        state.axis_mask = 0x03;
        state.position[0] = 0.01f;
        uint8_t frame[CCROS_MAX_FRAME];
        const int n = CcrosEncodeState(frame, sizeof(frame), &state);
        if (n > 0) {
          if (::send(conn, frame, static_cast<size_t>(n), MSG_NOSIGNAL) <= 0) {
            break;
          }
        }
        char sink[256];
        ::recv(conn, sink, sizeof(sink), MSG_DONTWAIT);
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
      }
      ::shutdown(conn, SHUT_RDWR);
      ::close(conn);
      drop_stream = false;
    }
  }
};

hardware_interface::HardwareInfo make_info(const MockBoard & board)
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["host"] = "127.0.0.1";
  info.hardware_parameters["session_port"] = std::to_string(board.session_port());
  info.hardware_parameters["stream_port"] = std::to_string(board.stream_port());
  info.hardware_parameters["axis_mask"] = "3";
  info.hardware_parameters["stream_mode"] = "position";
  info.hardware_parameters["test_mode"] = "false";
  for (const char * name : {"joint_x", "joint_y"}) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    hardware_interface::InterfaceInfo cmd;
    cmd.name = hardware_interface::HW_IF_POSITION;
    joint.command_interfaces.push_back(cmd);
    hardware_interface::InterfaceInfo pos;
    pos.name = hardware_interface::HW_IF_POSITION;
    hardware_interface::InterfaceInfo vel;
    vel.name = hardware_interface::HW_IF_VELOCITY;
    hardware_interface::InterfaceInfo hlfb;
    hlfb.name = "hlfb_duty";
    joint.state_interfaces.push_back(pos);
    joint.state_interfaces.push_back(vel);
    joint.state_interfaces.push_back(hlfb);
    info.joints.push_back(joint);
  }
  return info;
}

const std::vector<std::string> kActivate = {
  "disable", "clear_alerts", "configure", "set_test_mode", "enable"};
}  // namespace

TEST(ClearCoreSystem, ActivateWatchdogEofAndReconnect)
{
  MockBoard board;
  clearcore_hardware::ClearCoreSystemHardware hw;
  ASSERT_EQ(hw.on_init(make_info(board)), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    hw.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);

  {
    std::lock_guard<std::mutex> lock(board.mu);
    ASSERT_EQ(board.methods, kActivate);
    EXPECT_EQ(std::count(board.methods.begin(), board.methods.end(), "home"), 0);
  }

  const rclcpp::Time t(0, 0, RCL_STEADY_TIME);
  const rclcpp::Duration dt(0, 20000000);
  EXPECT_EQ(hw.write(t, dt), hardware_interface::return_type::OK);

  board.flags = static_cast<uint8_t>(CCROS_FLAG_ENABLED | CCROS_FLAG_WATCHDOG);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(hw.read(t, dt), hardware_interface::return_type::ERROR);
  EXPECT_EQ(hw.write(t, dt), hardware_interface::return_type::ERROR);

  EXPECT_EQ(
    hw.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  board.flags = CCROS_FLAG_ENABLED;
  ASSERT_EQ(
    hw.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  {
    std::lock_guard<std::mutex> lock(board.mu);
    ASSERT_GE(board.methods.size(), kActivate.size() * 2u + 1u);
    EXPECT_EQ(board.methods.back(), "enable");
    EXPECT_EQ(std::count(board.methods.begin(), board.methods.end(), "disable"), 3);
    EXPECT_EQ(std::count(board.methods.begin(), board.methods.end(), "clear_alerts"), 2);
    EXPECT_EQ(std::count(board.methods.begin(), board.methods.end(), "home"), 0);
  }
  EXPECT_EQ(hw.write(t, dt), hardware_interface::return_type::OK);

  board.drop_stream = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  EXPECT_EQ(hw.read(t, dt), hardware_interface::return_type::ERROR);

  hw.on_deactivate(rclcpp_lifecycle::State());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return rc;
}
