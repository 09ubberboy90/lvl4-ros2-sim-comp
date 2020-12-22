#ifndef SIM_ACTION_SERVER_H
#define SIM_ACTION_SERVER_H

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"

namespace sim_action_server
{

class ActionServer : public rclcpp::Node
{
public:

    ActionServer(std::string node_name = "trajectory_control");

    bool execute_plan(trajectory_msgs::msg::JointTrajectory trajectory);

private:

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
    bool common_goal_accepted = false;
    rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
    int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
    void common_goal_response(
        std::shared_future<rclcpp_action::ClientGoalHandle
        <control_msgs::action::FollowJointTrajectory>::SharedPtr> future);

    void common_result_response(
        const rclcpp_action::ClientGoalHandle
        <control_msgs::action::FollowJointTrajectory>::WrappedResult & result);

    void common_feedback(
        rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
        const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback);
};
} // namespace sim_action_server

#endif // SIM_ACTION_SERVER_H
