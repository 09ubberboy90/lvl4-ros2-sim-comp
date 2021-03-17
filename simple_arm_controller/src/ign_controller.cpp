/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <iostream>
#include <string>
#include <thread>


class Republisher : public rclcpp::Node
{
public:
    Republisher()
        : Node("republisher")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Republisher::listener_callback, this, std::placeholders::_1));
    }
    std::map<std::string, ignition::transport::Node::Publisher> joint_pub;

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

    void listener_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (int i = 0; i < msg->name.size(); i++)
        {
            auto joint_name = msg->name[i];
            auto position = msg->position[i];
            publish_joint_name(joint_name, position);
        }
        
    }

    void publish_joint_name(std::string joint_name, double position)
    {
        ignition::msgs::Double _msg;
        std::string topic = "/model/panda/joint/" + joint_name + "/0/cmd_pos";
        _msg.set_data(position);
        if (!joint_pub[joint_name].Publish(_msg))
        {
            std::cout << "Failed to publish" << std::endl;
        }
    }
};


//////////////////////////////////////////////////
int main(int argc, char **argv)
{
    // Install a signal handler for SIGINT and SIGTERM.
    rclcpp::init(argc, argv);

    auto republisher = std::make_shared<Republisher>();
    std::vector<std::string> joint_name = {"panda_joint1",
                                           "panda_joint2",
                                           "panda_joint3",
                                           "panda_joint4",
                                           "panda_joint5",
                                           "panda_joint6",
                                           "panda_joint7",
                                           "panda_finger_joint1",
                                           "panda_finger_joint2",};

    for (int i = 0; i < joint_name.size(); i++)
    {

        auto topic_name = joint_name[i];
        ignition::transport::Node pub;
        std::string topic = "/model/panda/joint/" + topic_name + "/0/cmd_pos";
        republisher->joint_pub[topic_name] = pub.Advertise<ignition::msgs::Double>(topic);
    }
    // Zzzzzz.
    rclcpp::spin(republisher);
    ignition::transport::waitForShutdown();
    rclcpp::shutdown();

    return 0;
}
