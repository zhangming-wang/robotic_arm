#pragma once

#include "pluginlib/class_list_macros.hpp" // 必须有这个，才能让插件被发现
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "servo_manager/servo_manager.h"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

class ServoHardware : public hardware_interface::SystemInterface {
  public:
    ServoHardware() = default;
    ~ServoHardware() = default;

    ServoHardware(const ServoHardware &) = delete;
    ServoHardware &operator=(const ServoHardware &) = delete;

    CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    std::vector<uint8_t> joint_ids_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_cmds_pos_;
    std::vector<double> hw_cmds_vel_;
    std::vector<double> hw_cmds_acc_;

    std::string serial_port_;
    int baud_rate_;
};