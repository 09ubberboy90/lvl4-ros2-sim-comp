#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/moveit_cpp/planning_component.h>

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
#include "sim_action_server.hpp"

class VrSubscriber : public rclcpp::Node
{
public:
    VrSubscriber()
        : Node("vr_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/controller/RightHand", 10, std::bind(&VrSubscriber::execute_goal, this, std::placeholders::_1));
    }
    std::shared_ptr<sim_action_server::ActionServer> server;
    moveit::planning_interface::MoveGroupInterface *move_group;

private:
    void execute_goal(const geometry_msgs::msg::Pose::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%f, %f, %f'", msg->position.x, msg->position.y, msg->position.z );
        move_group->setPoseTarget(*msg);

        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group
        // to actually move the robot.
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(server->get_logger(), "Plan 1 (pose goal) %s", success ? "SUCCEEDED" : "FAILED");

        if (success)
        {
            std::thread([this]() { move_group->asyncMove(); }).detach();
            server->execute_plan(my_plan.trajectory_.joint_trajectory);
        }
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    // For current state monitor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface(move_group_node->get_name());

    std::string action_node_name;
    if (!move_group_node->get_parameter("action_node_name", action_node_name))
    {
        // In case the parameter was not created use default
        action_node_name = "/follow_joint_trajectory";
    }
    auto server = std::make_shared<sim_action_server::ActionServer>("trajectory_control", action_node_name);    
    auto subscriber = std::make_shared<VrSubscriber>();
    subscriber->server = server;
    subscriber->move_group = &move_group;
    rclcpp::spin(subscriber);
    rclcpp::shutdown();
    return 0;
}