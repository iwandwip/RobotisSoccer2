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
#include "op3_demo/mic_test.h"

namespace robotis_op
{

MicTest::MicTest()
  : Node("mic_test"),
    SPIN_RATE(30),
    is_wait_(false),
    wait_time_(-1),
    test_status_(Ready),
    record_pid_(-1),
    play_pid_(-1)
{
  enable_ = false;
  start_time_ = rclcpp::Clock().now();

  recording_file_name_ = ament_index_cpp::get_package_share_directory("op3_demo") + "/data/mp3/test/mic-test.wav";
  default_mp3_path_ = ament_index_cpp::get_package_share_directory("op3_demo") + "/data/mp3/test/";

  // subscriber & publisher
  button_sub_ = this->create_subscription<std_msgs::msg::String>("/robotis/open_cr/button", 1, std::bind(&MicTest::buttonHandlerCallback, this, std::placeholders::_1));
  play_sound_pub_ = this->create_publisher<std_msgs::msg::String>("/play_sound_file", 10);

  // Use timer for loop rate
  create_wall_timer(std::chrono::duration<double>(1.0 / SPIN_RATE), std::bind(&MicTest::process, this));
}

MicTest::~MicTest()
{
}

void MicTest::setDemoEnable()
{
  wait_time_ = -1;
  test_status_ = AnnounceRecording;
  enable_ = true;

  RCLCPP_INFO(this->get_logger(), "Start Mic test Demo");
}

void MicTest::setDemoDisable()
{
  finishTimer();

  test_status_ = Ready;
  enable_ = false;
}

void MicTest::process()
{
  // check status
  // timer
  if (wait_time_ > 0)
  {
    rclcpp::Duration dur = rclcpp::Clock().now() - start_time_;

    // check timer
    if (dur.seconds() >= wait_time_)
    {
      finishTimer();
    }
  }
  else if (wait_time_ == -1.0)
  {
    // handle test process
    switch (test_status_)
    {
      case Ready:
        // do nothing
        break;

      case AnnounceRecording:
        announceTest();
        test_status_ = MicRecording;
        break;

      case MicRecording:
        recordSound();
        test_status_ = PlayingSound;
        break;

      case PlayingSound:
        playTestSound(recording_file_name_);
        test_status_ = DeleteTempFile;
        break;

      case DeleteTempFile:
        deleteSoundFile(recording_file_name_);
        test_status_ = Ready;
        break;

      default:
        break;
    }
  }

}

void MicTest::buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    // restart mic test
    if (test_status_ != Ready)
      return;

    test_status_ = AnnounceRecording;
  }
  else if(msg->data == "user")
  {
    is_wait_ = true;
  }
}

void MicTest::announceTest()
{
  // play mic test sound
  playSound(default_mp3_path_ + "Announce mic test.mp3");

  std::this_thread::sleep_for(std::chrono::milliseconds(3400));
}

void MicTest::recordSound(int recording_time)
{
  RCLCPP_INFO(rclcpp::get_logger("mic_test"), "Start to record");

  playSound(default_mp3_path_ + "Start recording.mp3");

  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  if (record_pid_ != -1)
    kill(record_pid_, SIGKILL);

  record_pid_ = fork();

  switch (record_pid_)
  {
    case -1:
      fprintf(stderr, "Fork Failed!! \n");
      RCLCPP_WARN(rclcpp::get_logger("mic_test"), "Fork Failed!! \n");
      break;

    case 0:
    {
      std::stringstream ss;
      ss << "-d " << recording_time;
      execl("/usr/bin/arecord", "arecord", "-Dplughw:1,0", "-fS16_LE", "-c1", "-r22050", "-twav", ss.str().c_str(),
            recording_file_name_.c_str(), (char*) 0);
      break;
    }

    default:
      break;
  }

  startTimer(recording_time);
}

void MicTest::recordSound()
{
  recordSound(5);
}

void MicTest::playTestSound(const std::string &path)
{
  RCLCPP_INFO(rclcpp::get_logger("mic_test"), "Start to play recording sound");

  playSound(default_mp3_path_ + "Start playing.mp3");

  std::this_thread::sleep_for(std::chrono::milliseconds(1300));

  if (play_pid_ != -1)
    kill(play_pid_, SIGKILL);

  play_pid_ = fork();

  switch (play_pid_)
  {
    case -1:
      fprintf(stderr, "Fork Failed!! \n");
      RCLCPP_WARN(rclcpp::get_logger("mic_test"), "Fork Failed!! \n");
      break;

    case 0:
      execl("/usr/bin/aplay", "aplay", path.c_str(), (char*) 0);
      break;

    default:
      break;
  }

  startTimer(5);
}

void MicTest::playSound(const std::string &path)
{
  std_msgs::msg::String sound_msg;
  sound_msg.data = path;

  play_sound_pub_->publish(sound_msg);
}

void MicTest::deleteSoundFile(const std::string &file_path)
{
  remove(file_path.c_str());
  RCLCPP_INFO(rclcpp::get_logger("mic_test"), "Delete temporary file");
}

void MicTest::startTimer(double wait_time)
{
  start_time_ = rclcpp::Clock().now();
  wait_time_ = wait_time;
}

void MicTest::finishTimer()
{
  wait_time_ = -1;
}

} /* namespace robotis_op */
