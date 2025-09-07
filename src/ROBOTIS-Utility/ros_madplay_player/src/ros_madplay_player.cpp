/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/*
 * ros_madplay_player.cpp
 *
 *  Created on: 2017. 02. 03.
 *      Author: Jay Song
 */

#include "ros_madplay_player/ros_madplay_player.h"

pid_t       g_play_pid = -1;
std::string g_sound_file_path = "";
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_done_msg_pub;

void play_sound_callback(const std_msgs::msg::String::SharedPtr msg)
{
  auto done_msg = std::make_shared<std_msgs::msg::String>();

  if(msg->data == "")
  {
    if(g_play_pid != -1)
      kill(g_play_pid, SIGKILL);

    g_play_pid = -1;
    done_msg->data = "play_sound_fail";
    g_done_msg_pub->publish(*done_msg);
    return;
  }

  if(g_play_pid != -1)
    kill(g_play_pid, SIGKILL);

  g_play_pid = fork();

  switch(g_play_pid)
  {
  case -1:
    fprintf(stderr, "Fork Failed!! \n");
    done_msg->data = "play_sound_fail";
    g_done_msg_pub->publish(*done_msg);
    break;
  case 0:
    execl("/usr/bin/madplay", "madplay", (g_sound_file_path + msg->data).c_str(), "-Q", (char*)0);
    break;
  default:
    done_msg->data = "play_sound";
    g_done_msg_pub->publish(*done_msg);
    break;
  }

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("ros_madplay_player");

  g_sound_file_path = nh->declare_parameter<std::string>("sound_file_path", "");
  if(g_sound_file_path != "" && g_sound_file_path.compare(g_sound_file_path.size()-1, 1, "/") != 0)
    g_sound_file_path += "/";

  auto play_mp3_sub = nh->create_subscription<std_msgs::msg::String>("/play_sound_file", 10, play_sound_callback);
  g_done_msg_pub = nh->create_publisher<std_msgs::msg::String>("/robotis/movement_done", 5);

  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}
