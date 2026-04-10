#include "arm_controller/arm_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <limits>
#include <unordered_map>

namespace arm_controller {

controller_interface::CallbackReturn ArmController::on_init() {
    try {
        auto_declare<std::vector<std::string>>("joints", {});
        auto_declare<bool>("update_by_time", false);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ArmController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &joint : joints_) {
        cfg.names.push_back(joint + "/position");
        cfg.names.push_back(joint + "/velocity");
        // cfg.names.push_back(joint + "/acceleration");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration ArmController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &joint : joints_) {
        cfg.names.push_back(joint + "/position");
        cfg.names.push_back(joint + "/velocity");
    }
    return cfg;
}

controller_interface::CallbackReturn ArmController::on_configure(const rclcpp_lifecycle::State &) {
    joints_ = get_node()->get_parameter("joints").as_string_array();

    if (joints_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
        return controller_interface::CallbackReturn::ERROR;
    }

    joint_tolerances_.clear();

    for (const auto &joint_name : joints_) {
        JointTolerance tolerance;

        // 2. 拼接参数路径，例如 "constraints.joint_limits.joint0.position_tolerance"
        std::string pos_param = "constraints.joint_limits." + joint_name + ".position_tolerance";
        std::string vel_param = "constraints.joint_limits." + joint_name + ".velocity_tolerance";

        if (!get_node()->has_parameter(pos_param)) {
            get_node()->declare_parameter<double>(pos_param, 0.01);
        }
        if (!get_node()->has_parameter(vel_param)) {
            get_node()->declare_parameter<double>(vel_param, 0.01);
        }

        tolerance.position = get_node()->get_parameter(pos_param).as_double();
        tolerance.velocity = get_node()->get_parameter(vel_param).as_double();

        joint_tolerances_[joint_name] = tolerance;

        RCLCPP_INFO(get_node()->get_logger(), "关节 %s 容限设定为: Pos: %.3f, Vel: %.3f",
                    joint_name.c_str(), tolerance.position, tolerance.velocity);
    }

    update_by_time_ = get_node()->get_parameter("update_by_time").as_bool();
    RCLCPP_INFO(get_node()->get_logger(), "轨迹推进模式: %s", update_by_time_ ? "按时间调度" : "按位置容差");

    target_positions_.assign(joints_.size(), std::numeric_limits<double>::quiet_NaN());
    target_velocities_.assign(joints_.size(), 0.0);
    target_accelerations_.assign(joints_.size(), 0.0);
    current_positions_.assign(joints_.size(), 0.0);
    current_velocities_.assign(joints_.size(), 0.0);

    trajectory_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "~/joint_trajectory",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&ArmController::on_trajectory_command, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        get_node(),
        "~/follow_joint_trajectory",
        std::bind(&ArmController::on_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ArmController::on_cancel, this, std::placeholders::_1),
        std::bind(&ArmController::on_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_node()->get_logger(), "Configured custom controller with standard trajectory interfaces.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_activate(const rclcpp_lifecycle::State &) {
    std::scoped_lock<std::mutex> lock(target_mutex_);

    if (state_interfaces_.size() != current_positions_.size() + current_velocities_.size()) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Expected %zu state interfaces, got %zu",
            current_positions_.size() + current_velocities_.size(),
            state_interfaces_.size());
        return controller_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < joints_.size(); ++i) {
        current_positions_[i] = state_interfaces_[i * 2 + 0].get_value();
        current_velocities_[i] = state_interfaces_[i * 2 + 1].get_value();

        target_positions_[i] = current_positions_[i];
        target_velocities_[i] = 0.0;
        target_accelerations_[i] = 0.0;
    }

    finalize_trajectory("Controller (re)activated.", GoalStatus::ABORTED);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_deactivate(const rclcpp_lifecycle::State &) {
    std::scoped_lock<std::mutex> lock(target_mutex_);
    finalize_trajectory("Controller deactivated.", GoalStatus::ABORTED);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ArmController::update(const rclcpp::Time &time, const rclcpp::Duration &) {
    std::scoped_lock<std::mutex> lock(target_mutex_);

    for (size_t i = 0; i < joints_.size(); ++i) {
        current_positions_[i] = state_interfaces_[i * 2 + 0].get_value();
        current_velocities_[i] = state_interfaces_[i * 2 + 1].get_value();
    }

    int command_size = command_interfaces_.size() / joints_.size();

    if (has_active_trajectory_) {
        bool last_trajectory_finished = true;
        double elapsed_seconds = (time - trajectory_start_time_).seconds();

        if (current_trajectory_point_index_ == 0) {
            trajectory_start_time_ = time;
        } else {
            bool move_timeout = elapsed_seconds >= rclcpp::Duration(
                                                       active_trajectory_.points[current_trajectory_point_index_ - 1].time_from_start)
                                                       .seconds();
            if (update_by_time_) {
                last_trajectory_finished = move_timeout;
            } else {
                if (!move_timeout) {
                    const auto &prev_point = active_trajectory_.points[current_trajectory_point_index_ - 1];
                    for (size_t i = 0; i < joints_.size(); ++i) {
                        if (fabs(current_positions_[i] - prev_point.positions[i]) > joint_tolerances_[joints_[i]].position) {
                            last_trajectory_finished = false;
                            break;
                        }
                    }
                }
            }
        }

        if (last_trajectory_finished) {
            if (current_trajectory_point_index_ >= active_trajectory_.points.size()) {
                finalize_trajectory("Trajectory execution completed.", GoalStatus::SUCCEEDED);
            } else {
                const auto &point = active_trajectory_.points[current_trajectory_point_index_];
                if (command_size == 1) {
                    target_positions_ = point.positions;
                } else if (command_size == 2) {
                    target_positions_ = point.positions;
                    target_velocities_ = point.velocities;
                } else if (command_size == 3) {
                    target_positions_ = point.positions;
                    target_velocities_ = point.velocities;
                    target_accelerations_ = point.accelerations;
                }
                current_trajectory_point_index_++;
            }
        }
    }

    for (size_t i = 0; i < joints_.size(); ++i) {
        if (!std::isfinite(target_positions_[i]))
            continue;

        if (command_size == 1) {
            command_interfaces_[i].set_value(target_positions_[i]);
        } else if (command_size == 2) {
            command_interfaces_[i * 2].set_value(target_positions_[i]);
            command_interfaces_[i * 2 + 1].set_value(target_velocities_[i]);
        } else if (command_size == 3) {
            command_interfaces_[i * 3].set_value(target_positions_[i]);
            command_interfaces_[i * 3 + 1].set_value(target_velocities_[i]);
            command_interfaces_[i * 3 + 2].set_value(target_accelerations_[i]);
        }
    }

    return controller_interface::return_type::OK;
}

void ArmController::on_trajectory_command(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    trajectory_msgs::msg::JointTrajectory normalized;
    std::string error;
    if (!load_trajectory(*msg, normalized, error)) {
        RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            2000,
            "Ignore trajectory command: %s",
            error.c_str());
        return;
    }

    std::scoped_lock<std::mutex> lock(target_mutex_);
    finalize_trajectory("Trajectory overridden by topic command.", GoalStatus::ABORTED);
    init_trajectory(std::move(normalized));
}

rclcpp_action::GoalResponse ArmController::on_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmController::on_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
    std::scoped_lock<std::mutex> lock(target_mutex_);
    if (active_goal_ && active_goal_ == goal_handle) {
        finalize_trajectory("Goal canceled by client.", GoalStatus::CANCELED);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmController::on_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
    trajectory_msgs::msg::JointTrajectory normalized;
    std::string error;
    if (!load_trajectory(goal_handle->get_goal()->trajectory, normalized, error)) {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
        result->error_string = error;
        goal_handle->abort(result);
        return;
    }

    std::scoped_lock<std::mutex> lock(target_mutex_);
    finalize_trajectory("New goal accepted, preempting previous goal.", GoalStatus::CANCELED);
    init_trajectory(std::move(normalized), goal_handle);
}

bool ArmController::load_trajectory(
    const trajectory_msgs::msg::JointTrajectory &input,
    trajectory_msgs::msg::JointTrajectory &normalized,
    std::string &error) const {
    error.clear();
    if (input.points.empty()) {
        error = "trajectory has no points";
        return false;
    }

    std::vector<size_t> index_map(joints_.size(), 0);
    if (input.joint_names.empty()) {
        for (size_t i = 0; i < joints_.size(); ++i) {
            index_map[i] = i;
        }
    } else {
        std::unordered_map<std::string, size_t> name_to_index;
        name_to_index.reserve(input.joint_names.size());
        for (size_t i = 0; i < input.joint_names.size(); ++i) {
            name_to_index[input.joint_names[i]] = i;
        }

        for (size_t i = 0; i < joints_.size(); ++i) {
            const auto it = name_to_index.find(joints_[i]);
            if (it == name_to_index.end()) {
                error = "joint '" + joints_[i] + "' not found in trajectory";
                return false;
            }
            index_map[i] = it->second;
        }
    }

    normalized.joint_names = joints_;
    normalized.points.reserve(input.points.size());

    double last_time = -1.0;
    for (const auto &src : input.points) {
        const double t = rclcpp::Duration(src.time_from_start).seconds();
        if (t < 0.0 || t < last_time) {
            error = "time_from_start must be nondecreasing";
            return false;
        }
        last_time = t;

        if (src.positions.size() < joints_.size()) {
            error = "point.positions size is smaller than joints size";
            return false;
        }

        trajectory_msgs::msg::JointTrajectoryPoint dst;
        dst.time_from_start = src.time_from_start;
        dst.positions.resize(joints_.size(), 0.0);
        dst.velocities.resize(joints_.size(), 0.0);
        dst.accelerations.resize(joints_.size(), 0.0);

        for (size_t i = 0; i < joints_.size(); ++i) {
            const size_t idx = index_map[i];
            dst.positions[i] = src.positions[idx];
            if (idx < src.velocities.size()) {
                dst.velocities[i] = src.velocities[idx];
            }
            if (idx < src.accelerations.size()) {
                dst.accelerations[i] = src.accelerations[idx];
            }
        }

        normalized.points.push_back(std::move(dst));
    }

    return true;
}

void ArmController::finalize_trajectory(const std::string &reason, const GoalStatus status) {
    has_active_trajectory_ = false;
    current_trajectory_point_index_ = 0;

    RCLCPP_INFO(get_node()->get_logger(), "Trajectory finalized: %s", reason.c_str());

    if (!active_goal_) {
        return;
    }

    auto result = std::make_shared<FollowJointTrajectory::Result>();
    switch (status) {
    case GoalStatus::SUCCEEDED:
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        break;
    case GoalStatus::CANCELED:
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        break;
    case GoalStatus::ABORTED:
        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
        break;
    }
    result->error_string = reason;

    if (status == GoalStatus::SUCCEEDED) {
        active_goal_->succeed(result);
    } else if (status == GoalStatus::CANCELED) {
        active_goal_->canceled(result);
    } else if (status == GoalStatus::ABORTED) {
        active_goal_->abort(result);
    }
    active_goal_.reset();
}

void ArmController::init_trajectory(trajectory_msgs::msg::JointTrajectory normalized, const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
    active_trajectory_ = std::move(normalized);
    has_active_trajectory_ = true;
    current_trajectory_point_index_ = 0;
    trajectory_start_time_ = get_node()->get_clock()->now();

    if (goal_handle)
        active_goal_ = goal_handle;

    RCLCPP_INFO(get_node()->get_logger(), "New trajectory initialized with %zu points. First point at %.3f seconds.",
                active_trajectory_.points.size(),
                rclcpp::Duration(active_trajectory_.points.front().time_from_start).seconds());
}

} // namespace arm_controller

PLUGINLIB_EXPORT_CLASS(arm_controller::ArmController, controller_interface::ControllerInterface)
