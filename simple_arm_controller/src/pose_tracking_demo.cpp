/*******************************************************************************
 *      Title     : pose_tracking_example.cpp -> vr_controller
 *      Project   : moveit_servo -> simple_arm_controller
 *      Created   : 09/04/2020
 *      Author    : Adam Pettinger, Florent Audonnet
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo_parameters.cpp>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>

#include <memory>
#include <string>
#include <vector>

class StatusMonitor
{
public:
  StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    sub_ = node->create_subscription<std_msgs::msg::Int8>(
        topic, 1, std::bind(&StatusMonitor::statusCB, this, std::placeholders::_1));
  }

private:
  void statusCB(const std_msgs::msg::Int8::ConstSharedPtr msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Servo status: " << status_str);
    }
  }

  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

class VrSubscriber : public rclcpp::Node
{
public:
    VrSubscriber()
        : Node("vr_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/controller/RightHand", 10, std::bind(&VrSubscriber::execute_goal, this, std::placeholders::_1));
    }
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> target_pose_pub;
    std::string header;
private:
    void execute_goal(const geometry_msgs::msg::Pose::SharedPtr msg) const
    {
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f, %f, %f'", msg->position.x, msg->position.y, msg->position.z );
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = header;
        target_pose.pose.position.x = msg->position.z + 0.5;
        target_pose.pose.position.y = msg->position.x;
        target_pose.pose.position.z = msg->position.y - 0.5;
        target_pose.pose.orientation.w = 1.0;
        target_pose.header.stamp = this->now();
        target_pose_pub->publish(target_pose);
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pose_tracking_demo");
    // For current state monitor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    moveit_servo::ServoParametersPtr servo_parameters;
    servo_parameters = std::make_shared<moveit_servo::ServoParameters>();
    if (!moveit_servo::readParameters(servo_parameters, node, rclcpp::get_logger("rclcpp")))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Could not get servo parameters!");
      exit(EXIT_FAILURE);
    }

    // Load the planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
    if (!planning_scene_monitor->getPlanningScene())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error in setting up the PlanningSceneMonitor.");
      exit(EXIT_FAILURE);
    }

    planning_scene_monitor->providePlanningSceneService();
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false /* skip octomap monitor */);
    planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    // Wait for Planning Scene Monitor to setup
    if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 5.0 /* seconds */))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Error waiting for current robot state in PlanningSceneMonitor.");
      exit(EXIT_FAILURE);
    }

    // Create the pose tracker
    moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);

    // Make a publisher for sending pose commands
    auto target_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 1 /* queue */);

    // Subscribe to servo status (and log it when it changes)
    StatusMonitor status_monitor(node, servo_parameters->status_topic);
    Eigen::Vector3d lin_tol{ 0.001, 0.001, 0.001 };
    double rot_tol = 0.01;

    // Get the current EE transform
    geometry_msgs::msg::TransformStamped current_ee_tf;
    tracker.getCommandFrameTransform(current_ee_tf);

    // Convert it to a Pose
    geometry_msgs::msg::PoseStamped target_pose;
    auto header = current_ee_tf.header.frame_id;
    target_pose.header.frame_id = header;
    target_pose.pose.position.x = current_ee_tf.transform.translation.x;
    target_pose.pose.position.y = current_ee_tf.transform.translation.y;
    target_pose.pose.position.z = current_ee_tf.transform.translation.z;
    target_pose.pose.orientation = current_ee_tf.transform.rotation;

    // Modify it a little bit
    target_pose.pose.position.x += 0.1;

    // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
    // waypoints
    tracker.resetTargetPose();

    // Publish target pose
    target_pose.header.stamp = move_group_node->now();
    target_pose_pub->publish(target_pose);
    std::thread move_to_pose_thread([&tracker, &lin_tol, &rot_tol] {
      moveit_servo::PoseTrackingStatusCode tracking_status =
          tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Pose tracker exited with status: "
                                    << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status));
    });

    std::string action_node_name;
    if (!move_group_node->get_parameter("action_node_name", action_node_name))
    {
        // In case the parameter was not created use default
        action_node_name = "/follow_joint_trajectory";
    }
    auto subscriber = std::make_shared<VrSubscriber>();
    subscriber->target_pose_pub = target_pose_pub;
    subscriber->header = header;
    rclcpp::spin(subscriber);
    rclcpp::shutdown();
    return 0;
}
