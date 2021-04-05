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
