#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/pose.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sim_action_server.hpp"

class VrSubscriber : public rclcpp::Node
{
public:
    VrSubscriber()
        : Node("vr_subscriber")
    {
        subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/fake_joint_trajectory_controller/joint_trajectory", 10, std::bind(&VrSubscriber::execute_goal, this, std::placeholders::_1));
    }
    std::shared_ptr<sim_action_server::ActionServer> server;

private:
    void execute_goal(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) const
    {
        server->execute_plan(*msg);
    }
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto subscriber = std::make_shared<VrSubscriber>();
    auto server = std::make_shared<sim_action_server::ActionServer>("trajectory_control", "/follow_joint_trajectory");    
    subscriber->server = server;
    rclcpp::spin(subscriber);
    rclcpp::shutdown();
    return 0;
}