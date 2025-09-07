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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "op3_demo/soccer_demo.h"
#include "op3_demo/action_demo.h"
#include "op3_demo/vision_demo.h"
#include "op3_demo/button_test.h"
#include "op3_demo/mic_test.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "robotis_controller_msgs/msg/sync_write_item.hpp"

enum Demo_Status
{
  Ready = 0,
  ButtonTest = 1,
  MicTest = 2,
  SoccerDemo = 3,
  VisionDemo = 4,
  ActionDemo = 5,
  DemoCount = 6,
};

void buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg);
void goInitPose();
void playSound(const std::string &path);
void setLED(int led);
bool checkManagerRunning(std::string& manager_name);
void dxlTorqueChecker();

void demoModeCommandCallback(const std_msgs::msg::String::SharedPtr msg);

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = true;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr init_pose_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr play_sound_pub;
rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr led_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dxl_torque_pub;

std::string default_mp3_path = "";
int current_status = Ready;
int desired_status = Ready;
bool apply_desired = false;

//node main
int main(int argc, char **argv)
{
  //init ros
  rclcpp::init(argc, argv);

  //create ros wrapper object
  robotis_op::OPDemo *current_demo = NULL;
  robotis_op::SoccerDemo *soccer_demo = new robotis_op::SoccerDemo();
  robotis_op::ActionDemo *action_demo = new robotis_op::ActionDemo();
  robotis_op::VisionDemo *vision_demo = new robotis_op::VisionDemo();
  robotis_op::ButtonTest *button_test = new robotis_op::ButtonTest();
  robotis_op::MicTest *mic_test = new robotis_op::MicTest();

  auto node = rclcpp::Node::make_shared("self_test_node");

  init_pose_pub = node->create_publisher<std_msgs::msg::String>("/robotis/base/ini_pose", 10);
  play_sound_pub = node->create_publisher<std_msgs::msg::String>("/play_sound_file", 10);
  led_pub = node->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 10);
  dxl_torque_pub = node->create_publisher<std_msgs::msg::String>("/robotis/dxl_torque", 10);
  auto button_sub = node->create_subscription<std_msgs::msg::String>("/robotis/open_cr/button", 10, buttonHandlerCallback);
  auto mode_command_sub = node->create_subscription<std_msgs::msg::String>("/robotis/mode_command", 10, demoModeCommandCallback);

  default_mp3_path = ament_index_cpp::get_package_share_directory("op3_demo") + "/data/mp3/";

  rclcpp::Rate loop_rate(SPIN_RATE);

  // wait for starting of manager
  std::string manager_name = "/op3_manager";
  while (rclcpp::ok())
  {
    rclcpp::sleep_for(std::chrono::seconds(1));

    if (checkManagerRunning(manager_name) == true)
    {
      break;
      if (DEBUG_PRINT)
        RCLCPP_INFO(node->get_logger(), "Succeed to connect");
    }
    RCLCPP_WARN(node->get_logger(), "Waiting for op3 manager");
  }

  // init procedure
  playSound(default_mp3_path + "test/Self test ready mode.mp3");
  setLED(0x01 | 0x02 | 0x04);

  //node loop
  while (rclcpp::ok())
  {
    // process
    if (apply_desired == true)
    {
      switch (desired_status)
      {
        case Ready:
        {

          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = NULL;

          goInitPose();

          if (DEBUG_PRINT)
            RCLCPP_INFO(node->get_logger(), "[Go to Demo READY!]");
          break;
        }

        case SoccerDemo:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = soccer_demo;
          current_demo->setDemoEnable();

          if (DEBUG_PRINT)
            RCLCPP_INFO(node->get_logger(), "[Start] Soccer Demo");
          break;
        }

        case VisionDemo:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = vision_demo;
          current_demo->setDemoEnable();
          if (DEBUG_PRINT)
            RCLCPP_INFO(node->get_logger(), "[Start] Vision Demo");
          break;
        }
        case ActionDemo:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = action_demo;
          current_demo->setDemoEnable();
          if (DEBUG_PRINT)
            RCLCPP_INFO(node->get_logger(), "[Start] Action Demo");
          break;
        }
        case ButtonTest:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = button_test;
          current_demo->setDemoEnable();
          if (DEBUG_PRINT)
            RCLCPP_INFO(node->get_logger(), "[Start] Button Test");
          break;
        }
        case MicTest:
        {
          if (current_demo != NULL)
            current_demo->setDemoDisable();

          current_demo = mic_test;
          current_demo->setDemoEnable();
          if (DEBUG_PRINT)
            RCLCPP_INFO(node->get_logger(), "[Start] Mic Test");
          break;
        }


        default:
        {
          break;
        }
      }

      apply_desired = false;
      current_status = desired_status;
    }

    //execute pending callbacks
    rclcpp::spin_some(node);

    //relax to fit output rate
    loop_rate.sleep();

    // for debug
    if (checkManagerRunning(manager_name) == false)
      return 0;
  }

  //exit program
  rclcpp::shutdown();
  return 0;
}

void buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if(apply_desired == true)
    return;

  // in the middle of playing demo
  if (current_status != Ready)
  {
    if (msg->data == "mode_long")
    {
      // go to mode selection status
      desired_status = Ready;
      apply_desired = true;

      playSound(default_mp3_path + "test/Self test ready mode.mp3");
      setLED(0x01 | 0x02 | 0x04);
    }
    else if (msg->data == "user_long")
    {
      // it's using in op3_manager
      // torque on and going to init pose
    }
  }
  // ready to start demo
  else
  {
    if (msg->data == "start")
    {
      // select current demo
      desired_status = (desired_status == Ready) ? desired_status + 1 : desired_status;
      apply_desired = true;

      // sound out desired status
      switch (desired_status)
      {
        case SoccerDemo:
          dxlTorqueChecker();
          playSound(default_mp3_path + "Start soccer demonstration.mp3");
          break;

        case VisionDemo:
          dxlTorqueChecker();
          playSound(default_mp3_path + "Start vision processing demonstration.mp3");
          break;

        case ActionDemo:
          dxlTorqueChecker();
          playSound(default_mp3_path + "Start motion demonstration.mp3");
          break;

        case ButtonTest:
          dxlTorqueChecker();
          playSound(default_mp3_path + "test/Start button test mode.mp3");
          break;

        case MicTest:
          dxlTorqueChecker();
          playSound(default_mp3_path + "test/Start mic test mode.mp3");
          break;

        default:
          break;
      }

      RCLCPP_INFO(rclcpp::get_logger("self_test_node"), "= Start Demo Mode : %d", desired_status);
    }
    else if (msg->data == "mode")
    {
      // change to next demo
      desired_status = (desired_status + 1) % DemoCount;
      desired_status = (desired_status == Ready) ? desired_status + 1 : desired_status;

      // sound out desired status and changign LED
      switch (desired_status)
      {
        case SoccerDemo:
          playSound(default_mp3_path + "Autonomous soccer mode.mp3");
          setLED(0x01);
          break;

        case VisionDemo:
          playSound(default_mp3_path + "Vision processing mode.mp3");
          setLED(0x02);
          break;

        case ActionDemo:
          playSound(default_mp3_path + "Interactive motion mode.mp3");
          setLED(0x04);
          break;

        case ButtonTest:
          playSound(default_mp3_path + "test/Button test mode.mp3");
          setLED(0x01 | 0x02);
          break;

        case MicTest:
          playSound(default_mp3_path + "test/Mic test mode.mp3");
          setLED(0x01 | 0x04);
          break;

        default:
          break;
      }

      RCLCPP_INFO(rclcpp::get_logger("self_test_node"), "= Demo Mode : %d", desired_status);
    }
  }
}

void goInitPose()
{
  std_msgs::msg::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub->publish(init_msg);
}

void playSound(const std::string &path)
{
  std_msgs::msg::String sound_msg;
  sound_msg.data = path;

  play_sound_pub->publish(sound_msg);
}

void setLED(int led)
{
  robotis_controller_msgs::msg::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led);

  led_pub->publish(syncwrite_msg);
}

bool checkManagerRunning(std::string& manager_name)
{
  auto node = rclcpp::Node::make_shared("check_manager_node");
  auto node_list = node->get_node_names();

  for (const auto& node_name : node_list)
  {
    if (node_name == manager_name)
      return true;
  }

  RCLCPP_ERROR(node->get_logger(), "Can't find op3_manager");
  return false;
}

void dxlTorqueChecker()
{
  std_msgs::msg::String check_msg;
  check_msg.data = "check";

  dxl_torque_pub->publish(check_msg);
}

void demoModeCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  auto node = rclcpp::Node::make_shared("self_test_node");

  // In demo mode
  if (current_status != Ready)
  {
    if (msg->data == "ready")
    {
      // go to mode selection status
      desired_status = Ready;
      apply_desired = true;

      playSound(default_mp3_path + "Demonstration ready mode.mp3");
      setLED(0x01 | 0x02 | 0x04);
    }
  }
  // In ready mode
  else
  {
    if(msg->data == "soccer")
    {
      desired_status = SoccerDemo;
      apply_desired = true;

      // play sound
      dxlTorqueChecker();
      playSound(default_mp3_path + "Start soccer demonstration.mp3");
      if (DEBUG_PRINT)
        RCLCPP_INFO(node->get_logger(), "= Start Demo Mode : %d", desired_status);
    }
    else if(msg->data == "vision")
    {
      desired_status = VisionDemo;
      apply_desired = true;

      // play sound
      dxlTorqueChecker();
      playSound(default_mp3_path + "Start vision processing demonstration.mp3");
      if (DEBUG_PRINT)
        RCLCPP_INFO(node->get_logger(), "= Start Demo Mode : %d", desired_status);
    }
    else if(msg->data == "action")
    {
      desired_status = ActionDemo;
      apply_desired = true;

      // play sound
      dxlTorqueChecker();
      playSound(default_mp3_path + "Start motion demonstration.mp3");
      if (DEBUG_PRINT)
        RCLCPP_INFO(node->get_logger(), "= Start Demo Mode : %d", desired_status);
    }
  }
}
