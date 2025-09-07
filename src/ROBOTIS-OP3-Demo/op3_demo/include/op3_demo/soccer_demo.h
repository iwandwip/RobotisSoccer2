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

#ifndef SOCCER_DEMO_H
#define SOCCER_DEMO_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "op3_action_module_msgs/srv/is_running.hpp"
#include "robotis_controller_msgs/msg/sync_write_item.hpp"
#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/srv/set_joint_module.hpp"

#include "op3_demo/op_demo.h"
#include "op3_demo/ball_tracker.h"
#include "op3_demo/ball_follower.h"
#include "robotis_math/robotis_linear_algebra.h"

namespace robotis_op
{

class SoccerDemo : public OPDemo
{
 public:
  enum Stand_Status
  {
    Stand = 0,
    Fallen_Forward = 1,
    Fallen_Behind = 2,
  };

  enum Robot_Status
  {
    Waited = 0,
    TrackingAndFollowing = 1,
    ReadyToKick = 2,
    ReadyToCeremony = 3,
    ReadyToGetup = 4,
  };

  SoccerDemo();
  ~SoccerDemo();

  void setDemoEnable();
  void setDemoDisable();

  void process();

  rclcpp::Node::SharedPtr node_;
  void setNode(rclcpp::Node::SharedPtr node);
  void buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg);
  void demoCommandCallback(const std_msgs::msg::String::SharedPtr msg);

 protected:
  const double FALL_FORWARD_LIMIT;
  const double FALL_BACK_LIMIT;
  const int SPIN_RATE;
  const bool DEBUG_PRINT;

  // void processThread();
  // void callbackThread();
  // void trackingThread();

  void setBodyModuleToDemo(const std::string &body_module, bool with_head_control = true);
  void setModuleToDemo(const std::string &module_name);
  void callServiceSettingModule(const robotis_controller_msgs::msg::JointCtrlModule &modules);
  void parseJointNameFromYaml(const std::string &path);
  bool getJointNameFromID(const int &id, std::string &joint_name);
  bool getIDFromJointName(const std::string &joint_name, int &id);
  int getJointCount();
  bool isHeadJoint(const int &id);

  void imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void startSoccerMode();
  void stopSoccerMode();

  void handleKick(int ball_position);
  void handleKick();
  bool handleFallen(int fallen_status);

  void playMotion(int motion_index);
  void setRGBLED(int blue, int green, int red);
  bool isActionRunning();

  void sendDebugTopic(const std::string &msgs);

  BallTracker ball_tracker_;
  BallFollower ball_follower_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;

  // rclcpp::Publisher<robotis_controller_msgs::msg::JointCtrlModule>::SharedPtr module_control_pub_;
  // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motion_index_pub_;
  // rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr rgb_led_pub_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr button_sub_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr demo_command_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;

  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr test_pub_;

  // rclcpp::Client<op3_action_module_msgs::srv::IsRunning>::SharedPtr is_running_client_;
  // rclcpp::Client<robotis_controller_msgs::srv::SetJointModule>::SharedPtr set_joint_module_client_;

  std::map<int, std::string> id_joint_table_;
  std::map<std::string, int> joint_id_table_;

  bool is_start_soccer_running_;
  bool is_grass_;
  int wait_count_;
  bool on_following_ball_;
  bool on_tracking_ball_;
  bool restart_soccer_;
  bool start_following_;
  bool stop_following_;
  bool stop_fallen_check_;
  int robot_status_;
  int tracking_status_;
  int stand_state_;
  double present_pitch_;

  // std::unique_ptr<std::thread> spin_thread_;
  // std::thread process_thread_;
  // std::thread tracking_thread_;
};

}  // namespace robotis_op
#endif // SOCCER_DEMO_H
