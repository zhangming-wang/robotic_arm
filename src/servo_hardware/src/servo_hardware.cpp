#include "servo_hardware/servo_hardware.h"

namespace servo_hardware {

ServoHardware::CallbackReturn
ServoHardware::on_init(const hardware_interface::HardwareInfo &hardware_info) {
    if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // 1. 获取全局硬件参数
    serial_port_ = info_.hardware_parameters.at("device");
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
    RCLCPP_INFO(rclcpp::get_logger("ServoHardware"), "Parsed hardware parameters: device = %s, baud_rate = %d", serial_port_.c_str(), baud_rate_);

    joint_ids_.clear();
    joint_names_.clear();
    joint_dir_.clear();

    for (const auto &joint : info_.joints) {
        uint8_t id = std::stoi(joint.parameters.at("id"));
        int8_t dir = joint.parameters.at("inverted") == "true" || joint.parameters.at("inverted") == "True" ? -1 : 1;
        std::string name = joint.name;
        joint_ids_.push_back(id);
        joint_names_.push_back(name);
        joint_dir_.push_back(dir);
        RCLCPP_INFO(rclcpp::get_logger("ServoHardware"), "Found joint with ID: %d, name: %s, inverted: %s", (int)id, name.c_str(), dir == -1 ? "true" : "false");
    }

    // 2. 动态调整容器大小并提取 ID
    int num_joints = info_.joints.size();
    hw_positions_.assign(num_joints, 0.0);
    hw_velocities_.assign(num_joints, 0.0);
    hw_cmds_pos_.assign(num_joints, 0.0);
    hw_cmds_vel_.assign(num_joints, 0.0);
    hw_cmds_acc_.assign(num_joints, 0.0);
    last_hw_cmds_pos_.assign(num_joints, 0.0);
    last_hw_cmds_vel_.assign(num_joints, 0.0);
    last_hw_cmds_acc_.assign(num_joints, 0.0);

    return CallbackReturn::SUCCESS;
} // namespace ServoHardware::CallbackReturn

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
        RCLCPP_INFO(rclcpp::get_logger("ServoHardware"), "Successfully initialized ServoManager with port %s and baud rate %d", serial_port_.c_str(), baud_rate_);
        return CallbackReturn::SUCCESS;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ServoHardware"), "Failed to initialize ServoManager with port %s and baud rate %d", serial_port_.c_str(), baud_rate_);
        return CallbackReturn::ERROR;
    }
}

ServoHardware::CallbackReturn ServoHardware::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    ServoManager::instance().close();
    RCLCPP_INFO(rclcpp::get_logger("ServoHardware"), "Cleaned up ServoManager");
    return CallbackReturn::SUCCESS;
}

ServoHardware::CallbackReturn ServoHardware::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    ServoManager::instance().stop(joint_ids_, 0);
    RCLCPP_INFO(rclcpp::get_logger("ServoHardware"), "Stopped ServoManager on shutdown");
    return CallbackReturn::SUCCESS;
}

ServoHardware::CallbackReturn ServoHardware::on_activate(const rclcpp_lifecycle::State &previous_state) {
    if (ServoManager::instance().get_state(joint_ids_, hw_positions_, hw_velocities_)) {
        for (size_t i = 0; i < hw_positions_.size(); ++i) {
            hw_positions_[i] *= joint_dir_[i];
            hw_velocities_[i] *= joint_dir_[i];
            hw_cmds_pos_[i] = hw_positions_[i];
            hw_cmds_vel_[i] = hw_velocities_[i];
            hw_cmds_acc_[i] = 0.0;
            last_hw_cmds_pos_[i] = hw_cmds_pos_[i];
            last_hw_cmds_vel_[i] = hw_cmds_vel_[i];
            last_hw_cmds_acc_[i] = hw_cmds_acc_[i];
        }
        if (ServoManager::instance().enable_torque(joint_ids_, true)) {
            RCLCPP_INFO(rclcpp::get_logger("ServoHardware"), "Enabled torque for joints on activation");
            return CallbackReturn::SUCCESS;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("ServoHardware"), "Failed to enable torque for joints");
            return CallbackReturn::ERROR;
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ServoHardware"), "Failed to get servo state");
        return CallbackReturn::ERROR;
    }
}

ServoHardware::CallbackReturn ServoHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    if (ServoManager::instance().enable_torque(joint_ids_, false)) {
        RCLCPP_INFO(rclcpp::get_logger("ServoHardware"), "Disabled torque for joints on deactivation");
        return CallbackReturn::SUCCESS;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ServoHardware"), "Failed to disable torque for joints");
        return CallbackReturn::ERROR;
    }
}

hardware_interface::return_type ServoHardware::prepare_command_mode_switch(
    const std::vector<std::string> &start_interfaces, const std::vector<std::string> &stop_interfaces) {
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ServoHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    auto res = ServoManager::instance().get_state(joint_ids_, hw_positions_, hw_velocities_);
    if (!res) {
        return hardware_interface::return_type::ERROR;
    } else {
        for (size_t i = 0; i < hw_positions_.size(); ++i) {
            hw_positions_[i] *= joint_dir_[i];
            hw_velocities_[i] *= joint_dir_[i];
        }
        return hardware_interface::return_type::OK;
    }
}

hardware_interface::return_type ServoHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    bool is_changed = false;
    for (size_t i = 0; i < hw_cmds_pos_.size(); ++i) {
        if (std::abs(hw_cmds_pos_[i] - last_hw_cmds_pos_[i]) > 0.01) {
            is_changed = true;
            last_hw_cmds_pos_[i] = hw_cmds_pos_[i];
        }
        if (std::abs(hw_cmds_vel_[i] - last_hw_cmds_vel_[i]) > 0.01) {
            is_changed = true;
            last_hw_cmds_vel_[i] = hw_cmds_vel_[i];
        }
        if (std::abs(hw_cmds_acc_[i] - last_hw_cmds_acc_[i]) > 0.01) {
            is_changed = true;
            last_hw_cmds_acc_[i] = hw_cmds_acc_[i];
        }
    }
    if (is_changed) {
        auto servo_pos = hw_cmds_pos_;
        auto servo_speed = hw_cmds_vel_;
        auto servo_acc = hw_cmds_acc_;
        for (size_t i = 0; i < servo_pos.size(); ++i) {
            servo_pos[i] *= joint_dir_[i];
            servo_speed[i] *= joint_dir_[i];
            // servo_acc[i] = 0;
        }

        auto res = ServoManager::instance().move(joint_ids_, servo_pos, servo_speed, servo_acc);
        if (!res) {
            return hardware_interface::return_type::ERROR;
        } else {
            return hardware_interface::return_type::OK;
        }
    } else {
        return hardware_interface::return_type::OK;
    }
}
} // namespace servo_hardware

PLUGINLIB_EXPORT_CLASS(
    servo_hardware::ServoHardware,
    hardware_interface::SystemInterface)
