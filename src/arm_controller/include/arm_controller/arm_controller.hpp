#pragma once

#include <atomic>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_interface/controller_interface.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

namespace arm_controller {

struct JointTolerance {
    double position;
    double velocity;
};

enum class GoalStatus {
    SUCCEEDED,
    CANCELED,
    ABORTED
};

class ArmController : public controller_interface::ControllerInterface {
  public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    void on_trajectory_command(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

    rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
    void on_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

    bool load_trajectory(const trajectory_msgs::msg::JointTrajectory &input, trajectory_msgs::msg::JointTrajectory &normalized, std::string &error) const;
    void finalize_trajectory(const std::string &reason = "", const GoalStatus status = GoalStatus::SUCCEEDED);
    void init_trajectory(trajectory_msgs::msg::JointTrajectory normalized, const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle = nullptr);

    std::vector<std::string> joints_;
    std::vector<double> target_positions_;
    std::vector<double> target_velocities_;
    std::vector<double> target_accelerations_;
    std::vector<double> current_positions_;
    std::vector<double> current_velocities_;
    std::map<std::string, JointTolerance> joint_tolerances_;

    trajectory_msgs::msg::JointTrajectory active_trajectory_;
    std::shared_ptr<GoalHandleFollowJointTrajectory> active_goal_;

    rclcpp::Time trajectory_start_time_;
    size_t current_trajectory_point_index_{0};
    bool has_active_trajectory_{false};
    bool update_by_time_{false};

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    std::mutex target_mutex_;
};

} // namespace arm_controller
