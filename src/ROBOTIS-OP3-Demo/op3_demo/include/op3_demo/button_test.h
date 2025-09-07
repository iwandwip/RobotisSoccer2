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

#ifndef BUTTON_TEST_H_
#define BUTTON_TEST_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/thread.hpp>

#include "op3_demo/op_demo.h"
#include "robotis_controller_msgs/msg/sync_write_item.hpp"

namespace robotis_op
{

class ButtonTest : public OPDemo, public rclcpp::Node
{
 public:
  ButtonTest();
  ~ButtonTest();

  void setDemoEnable();
  void setDemoDisable();

 protected:
  const int SPIN_RATE;

  void processThread();
  void callbackThread();

  void process();

  void setRGBLED(int blue, int green, int red);
  void setLED(int led);

  void playSound(const std::string &path);

  void buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr rgb_led_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr play_sound_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr button_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::shared_ptr<std::thread> queue_thread_;
  std::shared_ptr<std::thread> process_thread_;

  std::string default_mp3_path_;
  int led_count_;
  int rgb_led_count_;
};

} /* namespace robotis_op */

#endif /* BUTTON_TEST_H_ */
