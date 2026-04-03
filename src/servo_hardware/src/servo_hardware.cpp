#include "servo_hardware/servo_hardware.h"

ServoHardware::CallbackReturn ServoHardware::on_init(const hardware_interface::HardwareInfo &hardware_info) {
    if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // 1. 获取全局硬件参数
    serial_port_ = info_.hardware_parameters.at("device");
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));

    // 2. 动态调整容器大小并提取 ID
    int num_joints = info_.joints.size();
    hw_positions_.assign(num_joints, 0.0);
    hw_velocities_.assign(num_joints, 0.0);
    hw_cmds_pos_.assign(num_joints, 0.0);
    hw_cmds_vel_.assign(num_joints, 0.0);
    hw_cmds_acc_.assign(num_joints, 0.0);

    for (const auto &joint : info_.joints) {
        joint_ids_.push_back(std::stoi(joint.parameters.at("id")));
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ServoHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(info_.joints[i].name, "position", &hw_positions_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, "velocity", &hw_velocities_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ServoHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(info_.joints[i].name, "position", &hw_cmds_pos_[i]);
        command_interfaces.emplace_back(info_.joints[i].name, "velocity", &hw_cmds_vel_[i]);
        command_interfaces.emplace_back(info_.joints[i].name, "acceleration", &hw_cmds_acc_[i]);
    }
    return command_interfaces;
}

ServoHardware::CallbackReturn ServoHardware::on_configure(const rclcpp_lifecycle::State &previous_state) {
    if (ServoManager::instance().init(serial_port_, baud_rate_)) {
        return CallbackReturn::SUCCESS;
    } else {
        return CallbackReturn::ERROR;
    }
}

ServoHardware::CallbackReturn ServoHardware::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    ServoManager::instance().close();
    return CallbackReturn::SUCCESS;
}

ServoHardware::CallbackReturn ServoHardware::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    ServoManager::instance().stop(joint_ids_, 0);
    return CallbackReturn::SUCCESS;
}

ServoHardware::CallbackReturn ServoHardware::on_activate(const rclcpp_lifecycle::State &previous_state) {
    if (ServoManager::instance().enable_torque(joint_ids_, true)) {
        return CallbackReturn::SUCCESS;
    } else {
        return CallbackReturn::ERROR;
    }
}

ServoHardware::CallbackReturn ServoHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    if (ServoManager::instance().enable_torque(joint_ids_, false)) {
        return CallbackReturn::SUCCESS;
    } else {
        return CallbackReturn::ERROR;
    }
}

// hardware_interface::return_type ServoHardware::prepare_command_mode_switch(
//     const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) {
//     return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type ServoHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
//     return hardware_interface::return_type::OK;
// }
// hardware_interface::return_type ServoHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
//     return hardware_interface::return_type::OK;
// }