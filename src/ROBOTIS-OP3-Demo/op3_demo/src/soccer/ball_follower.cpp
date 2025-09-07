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

#include "op3_demo/ball_follower.h"

namespace robotis_op
{

BallFollower::BallFollower()
  : // Node("ball_follower"),
    FOV_WIDTH(35.2 * M_PI / 180),
    FOV_HEIGHT(21.6 * M_PI / 180),
    count_not_found_(0),
    count_to_kick_(0),
    on_tracking_(false),
    approach_ball_position_(NotFound),
    kick_motion_index_(83),
    CAMERA_HEIGHT(0.46),
    NOT_FOUND_THRESHOLD(50),
    MAX_FB_STEP(40.0 * 0.001),
    MAX_RL_TURN(15.0 * M_PI / 180),
    IN_PLACE_FB_STEP(-3.0 * 0.001),
    MIN_FB_STEP(5.0 * 0.001),
    MIN_RL_TURN(5.0 * M_PI / 180),
    UNIT_FB_STEP(1.0 * 0.001),
    UNIT_RL_TURN(0.5 * M_PI / 180),
    SPOT_FB_OFFSET(0.0 * 0.001),
    SPOT_RL_OFFSET(0.0 * 0.001),
    SPOT_ANGLE_OFFSET(0.0),
    hip_pitch_offset_(7.0),
    current_pan_(-10),
    current_tilt_(-10),
    current_x_move_(0.005),
    current_r_angle_(0),
    curr_period_time_(0.6),
    accum_period_time_(0.0),
    DEBUG_PRINT(false)
{
  // current_joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
  //     "/robotis/goal_joint_states", 10, std::bind(&BallFollower::currentJointStatesCallback, this, std::placeholders::_1));

  // set_walking_command_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/walking/command", 10);
  // set_walking_param_pub_ = this->create_publisher<op3_walking_module_msgs::msg::WalkingParam>("/robotis/walking/set_params", 10);
  // get_walking_param_client_ = this->create_client<op3_walking_module_msgs::srv::GetWalkingParam>("/robotis/walking/get_params");

  prev_time_ = rclcpp::Clock().now();
}

BallFollower::~BallFollower()
{

}

void BallFollower::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  if (node_ != nullptr)
  {
    current_joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/robotis/goal_joint_states", 10, std::bind(&BallFollower::currentJointStatesCallback, this, std::placeholders::_1));
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Node is not set");
  }
}

void BallFollower::startFollowing()
{
  on_tracking_ = true;
  RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Start Ball following");

  setWalkingCommand("start");

  bool result = getWalkingParam();
  if (result == true)
  {
    hip_pitch_offset_ = current_walking_param_.hip_pitch_offset;
    curr_period_time_ = current_walking_param_.period_time;
  }
  else
  {
    hip_pitch_offset_ = 7.0 * M_PI / 180;
    curr_period_time_ = 0.6;
  }
}

void BallFollower::stopFollowing()
{
  on_tracking_ = false;
  //  approach_ball_position_ = NotFound;
  count_to_kick_ = 0;
//  accum_ball_position_ = 0;
  RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Stop Ball following");

  setWalkingCommand("stop");
}

void BallFollower::currentJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  double pan, tilt;
  int get_count = 0;

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    if (msg->name[ix] == "head_pan")
    {
      pan = msg->position[ix];
      get_count += 1;
    }
    else if (msg->name[ix] == "head_tilt")
    {
      tilt = msg->position[ix];
      get_count += 1;
    }

    if (get_count == 2)
      break;
  }

  // check variation
  current_pan_ = pan;
  current_tilt_ = tilt;
}

void BallFollower::calcFootstep(double target_distance, double target_angle, double delta_time,
                                double& fb_move, double& rl_angle)
{
  // clac fb
  double next_movement = current_x_move_;
  if (target_distance < 0)
    target_distance = 0.0;

  double fb_goal = fmin(target_distance * 0.1, MAX_FB_STEP);
  accum_period_time_ += delta_time;
  if (accum_period_time_ > (curr_period_time_  / 4))
  {
    accum_period_time_ = 0.0;
    if ((target_distance * 0.1 / 2) < current_x_move_)
      next_movement -= UNIT_FB_STEP;
    else
      next_movement += UNIT_FB_STEP;
  }
  fb_goal = fmin(next_movement, fb_goal);
  fb_move = fmax(fb_goal, MIN_FB_STEP);
  if (DEBUG_PRINT)
  {
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "distance to ball : %6.4f, fb : %6.4f, delta : %6.6f", target_distance, fb_move, delta_time);
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "==============================================");
  }

  // calc rl angle
  double rl_goal = 0.0;
  if (fabs(target_angle) * 180 / M_PI > 5.0)
  {
    double rl_offset = fabs(target_angle) * 0.2;
    rl_goal = fmin(rl_offset, MAX_RL_TURN);
    rl_goal = fmax(rl_goal, MIN_RL_TURN);
    rl_angle = fmin(fabs(current_r_angle_) + UNIT_RL_TURN, rl_goal);

    if (target_angle < 0)
      rl_angle *= (-1);
  }
}

// x_angle : ball position (pan), y_angle : ball position (tilt), ball_size : angle of ball radius
bool BallFollower::processFollowing(double x_angle, double y_angle, double ball_size)
{
  rclcpp::Time curr_time = rclcpp::Clock().now();
  rclcpp::Duration dur = curr_time - prev_time_;
  double delta_time = dur.seconds();
  prev_time_ = curr_time;

  count_not_found_ = 0;
//  int ball_position_sum = 0;

  // check of getting head joints angle
  if (current_tilt_ == -10 && current_pan_ == -10)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Failed to get current angle of head joints.");
    setWalkingCommand("stop");

    on_tracking_ = false;
    approach_ball_position_ = NotFound;
    return false;
  }

  if (DEBUG_PRINT)
  {
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "   ============== Head | Ball ==============   ");
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "== Head Pan : %f | Ball X : %f", (current_pan_ * 180 / M_PI), (x_angle * 180 / M_PI));
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "== Head Tilt : %f | Ball Y : %f", (current_tilt_ * 180 / M_PI), (y_angle * 180 / M_PI));
  }

  approach_ball_position_ = OutOfRange;

  double distance_to_ball = CAMERA_HEIGHT * tan(M_PI * 0.5 + current_tilt_ - hip_pitch_offset_ - ball_size);

  double ball_y_angle = (current_tilt_ + y_angle) * 180 / M_PI;
  double ball_x_angle = (current_pan_ + x_angle) * 180 / M_PI;

  if (distance_to_ball < 0)
    distance_to_ball *= (-1);

  //double distance_to_kick = 0.25;
  double distance_to_kick = 0.22;

  // check whether ball is correct position.
  if ((distance_to_ball < distance_to_kick) && (fabs(ball_x_angle) < 25.0))
  {
    count_to_kick_ += 1;

    if (DEBUG_PRINT)
    {
      RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "head pan : %f | ball pan : %f", (current_pan_ * 180 / M_PI), (x_angle * 180 / M_PI));
      RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "head tilt : %f | ball tilt : %f", (current_tilt_ * 180 / M_PI), (y_angle * 180 / M_PI));
      RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "foot to kick : %f", ball_x_angle);
    }

    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "In range [%d | %f]", count_to_kick_, ball_x_angle);

    // ball queue
//    if(ball_position_queue_.size() >= 5)
//      ball_position_queue_.erase(ball_position_queue_.begin());

//    ball_position_queue_.push_back((ball_x_angle > 0) ? 1 : -1);


    if (count_to_kick_ > 20)
    {
      setWalkingCommand("stop");
      on_tracking_ = false;

      // check direction of the ball
//      accum_ball_position_ = std::accumulate(ball_position_queue_.begin(), ball_position_queue_.end(), 0);

//      if (accum_ball_position_ > 0)
      if (ball_x_angle > 0)
      {
        if (DEBUG_PRINT)
          RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Ready to kick : left");  // left
        approach_ball_position_ = OnLeft;
      }
      else
      {
        if (DEBUG_PRINT)
          RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Ready to kick : right");  // right
        approach_ball_position_ = OnRight;
      }

      return true;
    }
    else if (count_to_kick_ > 15)
    {
      //      if (ball_x_angle > 0)
      //        accum_ball_position_ += 1;
      //      else
      //        accum_ball_position_ -= 1;

      // send message
      setWalkingParam(IN_PLACE_FB_STEP, 0, 0);
      return false;
    }
  }
  else
  {
    count_to_kick_ = 0;
//    accum_ball_position_ = NotFound;
  }

  double fb_move = 0.0, rl_angle = 0.0;
  double distance_to_walk = distance_to_ball - distance_to_kick;

  calcFootstep(distance_to_walk, current_pan_, delta_time, fb_move, rl_angle);

  // send message
  setWalkingParam(fb_move, 0, rl_angle);

  // for debug
  //RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "distance to ball : %6.4f, fb : %6.4f, delta : %6.6f", distance_to_ball, fb_move, delta_time);

  return false;
}

void BallFollower::decideBallPositin(double x_angle, double y_angle)
{
  // check of getting head joints angle
  if (current_tilt_ == -10 && current_pan_ == -10)
  {
    approach_ball_position_ = NotFound;
    return;
  }

  double ball_x_angle = current_pan_ + x_angle;

  if (ball_x_angle > 0)
    approach_ball_position_ = OnLeft;
  else
    approach_ball_position_ = OnRight;
}

void BallFollower::waitFollowing()
{
  count_not_found_++;

  if (count_not_found_ > NOT_FOUND_THRESHOLD * 0.5)
    setWalkingParam(0.0, 0.0, 0.0);
}

void BallFollower::setWalkingCommand(const std::string &command)
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Node is not set, cannot set walking command");
    return;
  }

  // get param
  if (command == "start")
  {
    getWalkingParam();
    setWalkingParam(IN_PLACE_FB_STEP, 0, 0, true);
  }

  auto set_walking_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/robotis/walking/command", 10);
  std_msgs::msg::String _command_msg;
  _command_msg.data = command;
  set_walking_command_pub_->publish(_command_msg);

  if (DEBUG_PRINT)
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Send Walking command : %s", command.c_str());
}

void BallFollower::setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance)
{
  current_walking_param_.balance_enable = balance;
  current_walking_param_.x_move_amplitude = x_move + SPOT_FB_OFFSET;
  current_walking_param_.y_move_amplitude = y_move + SPOT_RL_OFFSET;
  current_walking_param_.angle_move_amplitude = rotation_angle + SPOT_ANGLE_OFFSET;

  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Node is not set, cannot set walking parameters");
    return;
  }
  auto set_walking_param_pub_ = node_->create_publisher<op3_walking_module_msgs::msg::WalkingParam>("/robotis/walking/set_params", 10);
  set_walking_param_pub_->publish(current_walking_param_);

  current_x_move_ = x_move;
  current_r_angle_ = rotation_angle;
}

bool BallFollower::getWalkingParam()
{
  auto temp_node = rclcpp::Node::make_shared("ballfollower_get_walking_param");
  auto get_walking_param_client_ = temp_node->create_client<op3_walking_module_msgs::srv::GetWalkingParam>("/robotis/walking/get_params");
  auto request = std::make_shared<op3_walking_module_msgs::srv::GetWalkingParam::Request>();

  if (!get_walking_param_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "BallFollower::getWalkingParam - Service not available");
    return false;
  }

  auto future = get_walking_param_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(temp_node, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = future.get();
    if (result)
    {
      current_walking_param_ = result->parameters;

      if (DEBUG_PRINT)
        RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Get walking parameters");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Fail to get walking parameters.");
    }
  }

  return true;
}

}

