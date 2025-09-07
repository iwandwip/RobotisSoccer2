/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
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
*******************************************************************************/

/* Author: Kayman Jung */

#ifndef ACTION_DEMO_H_
#define ACTION_DEMO_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "op3_demo/op_demo.h"
#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/srv/set_module.hpp"
#include "op3_action_module_msgs/srv/is_running.hpp"

namespace robotis_op
{

class ActionDemo : public OPDemo
{
 public:
  ActionDemo();
  ~ActionDemo();

  void setDemoEnable();
  void setDemoDisable();

  void process();

  rclcpp::Node::SharedPtr node_;
  void setNode(rclcpp::Node::SharedPtr node);
  void buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg);
  void demoCommandCallback(const std_msgs::msg::String::SharedPtr msg);

 protected:
  enum ActionCommandIndex
  {
    BrakeActionCommand = -2,
    StopActionCommand = -1,
  };

  enum ActionStatus
  {
    PlayAction = 1,
    PauseAction = 2,
    StopAction = 3,
    ReadyAction = 4,
  };

  const int SPIN_RATE;
  const bool DEBUG_PRINT;

  // void processThread();
  // void callbackThread();

  void startProcess(const std::string &set_name = "default");
  void resumeProcess();
  void pauseProcess();
  void stopProcess();

  void parseActionScript(const std::string &path);
  bool parseActionScriptSetName(const std::string &path, const std::string &set_name);

  bool playActionWithSound(int motion_index);

  void playMP3(std::string &path);
  void stopMP3();

  void playAction(int motion_index);
  void stopAction();
  void brakeAction();
  bool isActionRunning();

  void setModuleToDemo(const std::string &module_name);
  void callServiceSettingModule(const std::string &module_name);
  // void actionSetNameCallback(const std_msgs::msg::String::SharedPtr msg);

  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr module_control_pub_;
  // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motion_index_pub_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr play_sound_pub_;

  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr button_sub_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr demo_command_sub_;

  // rclcpp::Client<op3_action_module_msgs::srv::IsRunning>::SharedPtr is_running_client_;
  // rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedPtr set_joint_module_client_;

  std::map<int, std::string> action_sound_table_;
  std::vector<int> play_list_;

  std::string script_path_;
  std::string play_list_name_;
  int play_index_;

  int play_status_;
};

} /* namespace robotis_op */

#endif /* ACTION_DEMO_H_ */
