//BSD 3-Clause License
//
//Copyright (c) 2021, Florent Audonnet
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//1. Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//2. Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
//3. Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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