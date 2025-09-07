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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "op3_demo/button_test.h"

namespace robotis_op
{

ButtonTest::ButtonTest()
  : Node("button_test"),
    SPIN_RATE(30),
    led_count_(0),
    rgb_led_count_(0)
{
  enable_ = false;
  default_mp3_path_ = ament_index_cpp::get_package_share_directory("op3_demo") + "/data/mp3/test/";

  // Create publishers
  rgb_led_pub_ = this->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 10);
  play_sound_pub_ = this->create_publisher<std_msgs::msg::String>("/play_sound_file", 10);

  // Create subscribers
  button_sub_ = this->create_subscription<std_msgs::msg::String>("/robotis/open_cr/button", 10, std::bind(&ButtonTest::buttonHandlerCallback, this, std::placeholders::_1));

  // Use timer for loop rate
  create_wall_timer(std::chrono::duration<double>(1.0 / SPIN_RATE), std::bind(&ButtonTest::process, this));
}

ButtonTest::~ButtonTest()
{
}

void ButtonTest::setDemoEnable()
{
  enable_ = true;
  RCLCPP_INFO(this->get_logger(), "Start Button Test");
}

void ButtonTest::setDemoDisable()
{
  enable_ = false;
  RCLCPP_INFO(this->get_logger(), "Stop Button Test");
}

void ButtonTest::process()
{

}

// button test
void ButtonTest::buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "mode")
  {
    playSound(default_mp3_path_ + "Mode button pressed.mp3");
  }
  else if (msg->data == "start")
  {
    // RGB led color test
    playSound(default_mp3_path_ + "Start button pressed.mp3");
    int rgb_selector[3] = { 1, 0, 0 };
    setRGBLED((0x1F * rgb_selector[rgb_led_count_ % 3]), (0x1F * rgb_selector[(rgb_led_count_ + 1) % 3]),
              (0x1F * rgb_selector[(rgb_led_count_ + 2) % 3]));
    rgb_led_count_ += 1;
  }
  else if (msg->data == "user")
  {
    // Monochromatic led color test
    playSound(default_mp3_path_ + "User button pressed.mp3");
    setLED(0x01 << (led_count_++ % 3));
  }
  else if (msg->data == "mode_long")
  {
    playSound(default_mp3_path_ + "Mode button long pressed.mp3");
  }
  else if (msg->data == "start_long")
  {
    playSound(default_mp3_path_ + "Start button long pressed.mp3");
  }
  else if (msg->data == "user_long")
  {
    playSound(default_mp3_path_ + "User button long pressed.mp3");
  }
}

void ButtonTest::setRGBLED(int blue, int green, int red)
{
  int led_full_unit = 0x1F;
  int led_value = (blue & led_full_unit) << 10 | (green & led_full_unit) << 5 | (red & led_full_unit);
  robotis_controller_msgs::msg::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED_RGB";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led_value);

  rgb_led_pub_->publish(syncwrite_msg);
}

void ButtonTest::setLED(int led)
{
  robotis_controller_msgs::msg::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led);

  rgb_led_pub_->publish(syncwrite_msg);
}

void ButtonTest::playSound(const std::string &path)
{
  std_msgs::msg::String sound_msg;
  sound_msg.data = path;

  play_sound_pub_->publish(sound_msg);
}

} /* namespace robotis_op */
