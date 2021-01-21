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
  std::string topic = "/model/panda/joint/"+joint_name+"/0/cmd_pos";
  std::cout << "Msg: " << _msg.data() << std::endl << std::endl;
  if (!joint_pub[joint_name].Publish(_msg))
  {
    std::cout << "Failed to publish"<< std::endl;
  }
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void panda_joint1(const ignition::msgs::Double &_msg)
{
  publish_joint_name("panda_joint1", _msg);
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void panda_joint2(const ignition::msgs::Double &_msg)
{
  publish_joint_name("panda_joint2", _msg);
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void panda_joint3(const ignition::msgs::Double &_msg)
{
  publish_joint_name("panda_joint3", _msg);
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void panda_joint4(const ignition::msgs::Double &_msg)
{
  publish_joint_name("panda_joint4", _msg);
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void panda_joint5(const ignition::msgs::Double &_msg)
{
  publish_joint_name("panda_joint5", _msg);
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void panda_joint6(const ignition::msgs::Double &_msg)
{
  publish_joint_name("panda_joint6", _msg);
}
//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void panda_joint7(const ignition::msgs::Double &_msg)
{
  publish_joint_name("panda_joint7", _msg);
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
    {"panda_joint1",panda_joint1},
    {"panda_joint2",panda_joint2},
    {"panda_joint3",panda_joint3},
    {"panda_joint4",panda_joint4},
    {"panda_joint5",panda_joint5},
    {"panda_joint6",panda_joint6},
    {"panda_joint7",panda_joint7},
  };
  std::vector<std::string> joint_name = {"panda_joint1",
                   "panda_joint2",
                   "panda_joint3",
                   "panda_joint4",
                   "panda_joint5",
                   "panda_joint6",
                   "panda_joint7"};
  ignition::transport::Node node_list[joint_name.size()];
  for (int i=0; i<joint_name.size(); i++) 
  {  

    auto topic_name = joint_name[i];
    ignition::transport::Node pub;
    std::string topic = "/model/panda/joint/"+topic_name+"/0/cmd_pos";
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
