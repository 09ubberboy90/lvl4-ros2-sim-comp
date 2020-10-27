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

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

/// \brief Flag used to break the publisher loop and terminate the program.
static std::atomic<bool> g_terminatePub(false);
std::map<std::string, ignition::transport::Node::Publisher> joint_pub;
std::map<std::string, std::function<void(const ignition::msgs::Double&)>> func_map;
//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminatePub = true;
}


void publish_joint_name(std::string joint_name,const ignition::msgs::Double &_msg)
{
  std::string topic = "/model/ur10/joint/"+joint_name+"/0/cmd_pos";
  std::cout << "Msg: " << _msg.data() << std::endl << std::endl;
  if (!joint_pub[joint_name].Publish(_msg))
  {
    std::cout << "Failed to publish"<< std::endl;
  }
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void shoulder_pan_cb(const ignition::msgs::Double &_msg)
{
  publish_joint_name("shoulder_pan_joint", _msg);
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void shoulder_lift_cb(const ignition::msgs::Double &_msg)
{
  publish_joint_name("shoulder_lift_joint", _msg);
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void elbow_cb(const ignition::msgs::Double &_msg)
{
  publish_joint_name("elbow_joint", _msg);
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void wrist_1_cb(const ignition::msgs::Double &_msg)
{
  publish_joint_name("wrist_1_joint", _msg);
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void wrist_2_cb(const ignition::msgs::Double &_msg)
{
  publish_joint_name("wrist_2_joint", _msg);
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void wrist_3_cb(const ignition::msgs::Double &_msg)
{
  publish_joint_name("wrist_3_joint", _msg);
}

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb(const ignition::msgs::Double &_msg)
{
  std::cout << "Msg: " << _msg.data() << std::endl << std::endl;

}


//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);

  func_map = {
    {"shoulder_pan_joint",shoulder_pan_cb},
    {"shoulder_lift_joint",shoulder_lift_cb},
    {"elbow_joint",elbow_cb},
    {"wrist_1_joint",wrist_1_cb},
    {"wrist_2_joint",wrist_2_cb},
    {"wrist_3_joint",wrist_3_cb},
  };
  std::vector<std::string> joint_name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  ignition::transport::Node node_list[joint_name.size()];
  for (int i=0; i<joint_name.size(); i++) 
  {  

    auto topic_name = joint_name[i];
    ignition::transport::Node pub;
    std::string topic = "/model/ur10/joint/"+topic_name+"/0/cmd_pos";
    joint_pub[topic_name] = pub.Advertise<ignition::msgs::Double>(topic);

    if (!node_list[0].Subscribe("/robot/"+topic_name,func_map[topic_name] ))
    {
      std::cerr << "Error subscribing to topic [" << topic_name << "]" << std::endl;
      return -1;
    }

  }
  // Zzzzzz.
  ignition::transport::waitForShutdown();

  return 0;
}
