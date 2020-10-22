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
std::map<std::string, ignition::transport::Node> joint_pub;
//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminatePub = true;
}

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb(const ignition::msgs::Model &_msg)
{
  std::cout << "Msg: " << _msg.joint_size() << std::endl;
  for (auto &joint_name : _msg.joint()) 
  {  
    std::cout << joint_name.name() << std::endl;
  }

  // std::string topic = "/foo";

  // auto pub = node.Advertise<ignition::msgs::Double>(topic);
  // if (!pub)
  // {
  //   std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
  //   return -1;
  // }

  // // Prepare the message.
  // ignition::msgs::StringMsg msg;
  // msg.set_data("HELLO");

  // // Publish messages at 1Hz.
  // while (!g_terminatePub)
  // {
  //   if (!pub.Publish(msg))
  //     break;

  //   std::cout << "Publishing hello on topic [" << topic << "]" << std::endl;
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // }

}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Install a signal handler for SIGINT and SIGTERM.
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::vector<std::string> joint_name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  // for (auto &topic_name : joint_name) 
  // {  
  //   joint_pub[topic_name] = ignition::transport::Node();
  // }

  // Create a transport node and advertise a topic.
  ignition::transport::Node listener;
  std::string listener_topic = "/joint_states";

  if (!listener.Subscribe(listener_topic, cb))
  {
    std::cerr << "Error subscribing to topic [" << listener_topic << "]" << std::endl;
    return -1;
  }

  ignition::transport::waitForShutdown();

  return 0;
}
