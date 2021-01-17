#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/macros/console_colors.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sim_action_server.hpp"
#include "simple_interface/srv/get_poses.hpp"

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
    bool use_spawn_obj;
    if (!move_group_node->get_parameter("action_node_name", action_node_name))
    {
        // In case the parameter was not created use default
        action_node_name = "/follow_joint_trajectory";
    }
    auto server = std::make_shared<sim_action_server::ActionServer>("trajectory_control", action_node_name);
    std::vector<std::string> joint_names = {"panda_joint1",
                                            "panda_joint2",
                                            "panda_joint3",
                                            "panda_joint4",
                                            "panda_joint5",
                                            "panda_joint6",
                                            "panda_joint7"};
    // Planning to a Pose goal

    if (move_group_node->get_parameter("use_spawn_obj", use_spawn_obj))
    {
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cube_pos_client");

        rclcpp::Client<simple_interface::srv::GetPoses>::SharedPtr client =
            node->create_client<simple_interface::srv::GetPoses>("get_cubes_pose");

        auto request = std::make_shared<simple_interface::srv::GetPoses::Request>();
        std::chrono::seconds timeout(1);
        while (!client->wait_for_service(timeout))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return 0;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            moveit_msgs::msg::CollisionObject collision_object[result.get()->poses.poses.size()];
            for (int i=0; i< result.get()->poses.poses.size(); i++)
            {
                auto pose = result.get()->poses.poses[i];
                collision_object[i].header.frame_id = move_group.getPlanningFrame();
                collision_object[i].id = "Box"+i;
                shape_msgs::msg::SolidPrimitive primitive;
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                if (i == 0) // table
                {
                    primitive.dimensions[0] = 0.913;
                    primitive.dimensions[1] = 0.913;
                    primitive.dimensions[2] = 0.82;
                }
                else // cube
                {
                    primitive.dimensions[0] = 0.05;
                    primitive.dimensions[1] = 0.05;
                    primitive.dimensions[2] = 0.05;

                }
                if (i == 1){
                    geometry_msgs::msg::Pose target_pose1;
                    target_pose1.orientation = pose.orientation;
                    target_pose1.position.x = pose.position.x;
                    target_pose1.position.y = pose.position.y;
                    target_pose1.position.z = pose.position.z;

                    move_group.setPoseTarget(target_pose1);

                }

                // Define a pose for the box (specified relative to frame_id)
                geometry_msgs::msg::Pose box_pose;
                box_pose.orientation = pose.orientation;
                box_pose.position.x = pose.position.x;
                box_pose.position.y = pose.position.y;
                box_pose.position.z = pose.position.z;
                if (i == 0) // table
                {
                    box_pose.position.z += 0.45;
                }
                collision_object[i].primitives.push_back(primitive);
                collision_object[i].primitive_poses.push_back(box_pose);
                collision_object[i].operation = collision_object[i].ADD;

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Add an object into the world");
                planning_scene_interface.applyCollisionObject(collision_object[i]);
            }



        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
        }

        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group
        // to actually move the robot.
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(server->get_logger(), "Plan 1 (pose goal) %s", success ? "SUCCEEDED" : "FAILED");

        if (success)
        {
            std::thread([&move_group]() { move_group.asyncMove(); }).detach();
            server->execute_plan(my_plan.trajectory_.joint_trajectory);
        }
    }


    rclcpp::shutdown();
    return 0;
}