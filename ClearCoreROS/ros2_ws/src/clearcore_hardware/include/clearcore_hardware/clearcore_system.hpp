/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#ifndef CLEARCORE_HARDWARE__CLEARCORE_SYSTEM_HPP_
#define CLEARCORE_HARDWARE__CLEARCORE_SYSTEM_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace clearcore_hardware
{

class ClearCoreSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ClearCoreSystemHardware)

  ~ClearCoreSystemHardware() override;

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
  std::string host_{"192.168.0.109"};
  int session_port_{9200};
  int stream_port_{9201};
  int axis_mask_{3};
  int session_fd_{-1};
  int stream_fd_{-1};
  std::vector<uint8_t> rx_;
  bool have_state_{false};
  bool velocity_stream_{false};
  int stable_cycles_{0};
  uint16_t seq_{1};

  std::vector<int> axis_of_joint_;
  std::vector<double> hw_cmd_;
  std::vector<double> hw_pos_;
  std::vector<double> hw_vel_;
  std::vector<double> hw_eff_;
  std::vector<double> last_cmd_;

  void close_all();
  bool session_call(const std::string & method, const std::string & params_json);
  hardware_interface::return_type drain_stream();
  bool send_frame(const uint8_t * data, size_t n);
  std::string param(const std::string & key, const std::string & fallback) const;
};

}  // namespace clearcore_hardware

#endif
