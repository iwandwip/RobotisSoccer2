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

#ifndef BALL_TRACKING_H_
#define BALL_TRACKING_H_

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "op3_ball_detector_msgs/msg/circle_set_stamped.hpp"
#include "op3_walking_module_msgs/msg/walking_param.hpp"
#include "op3_walking_module_msgs/srv/get_walking_param.hpp"

namespace robotis_op
{

// head tracking for looking the ball
class BallTracker //: public rclcpp::Node
{
public:
  enum TrackingStatus
  {
    NotFound = -1,
    Waiting = 0,
    Found = 1,
  };

  BallTracker();
  ~BallTracker();

  int processTracking();

  void startTracking();
  void stopTracking();

  void setUsingHeadScan(bool use_scan);
  void goInit();

  double getPanOfBall()
  {
    // left (+) ~ right (-)
    return current_ball_pan_;
  }
  double getTiltOfBall()
  {
    // top (+) ~ bottom (-)
    return current_ball_tilt_;
  }
  double getBallSize()
  {
    return current_ball_bottom_;
  }

  rclcpp::Node::SharedPtr node_;
  void setNode(rclcpp::Node::SharedPtr node);

protected:
  const double FOV_WIDTH;
  const double FOV_HEIGHT;
  const int NOT_FOUND_THRESHOLD;
  const int WAITING_THRESHOLD;
  const bool DEBUG_PRINT;

  void ballPositionCallback(const op3_ball_detector_msgs::msg::CircleSetStamped::SharedPtr msg);
  // void ballTrackerCommandCallback(const std_msgs::msg::String::SharedPtr msg);
  void publishHeadJoint(double pan, double tilt);
  void scanBall();

  //image publisher/subscriber
  // rclcpp::Publisher<robotis_controller_msgs::msg::JointCtrlModule>::SharedPtr module_control_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_joint_offset_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_joint_pub_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr head_scan_pub_;

  //  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;

  // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motion_index_pub_;

  rclcpp::Subscription<op3_ball_detector_msgs::msg::CircleSetStamped>::SharedPtr ball_position_sub_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ball_tracking_command_sub_;

  // (x, y) is the center position of the ball in image coordinates
  // z is the ball radius
  geometry_msgs::msg::Point ball_position_;

  int tracking_status_;
  bool use_head_scan_;
  int count_not_found_;
  bool on_tracking_;
  double current_ball_pan_, current_ball_tilt_;
  double current_ball_bottom_;
  double x_error_sum_, y_error_sum_;
  rclcpp::Time prev_time_;
  double p_gain_, d_gain_, i_gain_;
};
}

#endif /* BALL_TRACKING_H_ */
