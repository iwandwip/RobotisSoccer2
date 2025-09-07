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

/* Author: Kayman, SCH */

#include <stdio.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "op3_tuning_module/tuning_module.h"

namespace robotis_op
{

TuningModule::TuningModule()
  : Node("op3_tuning_module"),
    control_cycle_msec_(0),
    has_goal_joints_(false),
    ini_pose_only_(false),
    get_tuning_data_(false)
{
  enable_ = false;
  module_name_ = "tuning_module";
  control_mode_ = robotis_framework::PositionControl;

  tuning_module_state_ = new TuningModuleState();
  joint_state_ = new TuneJointState();
}

TuningModule::~TuningModule()
{
  if (queue_thread_.joinable())
    queue_thread_.join();
}

void TuningModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = std::thread(&TuningModule::queueThread, this);

  // init result, joint_id_table
  for (auto& it : robot->dxls_)
  {
    std::string joint_name = it.first;
    robotis_framework::Dynamixel* dxl_info = it.second;

    joint_name_to_id_[joint_name] = dxl_info->id_;
    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;

    //Initialize RobotOffsetData
    robot_tuning_data_[joint_name] = new JointOffsetData(0, 0);
    robot_torque_enable_data_[joint_name] = true;
  }

  /* publish topics */
  status_msg_pub_ = this->create_publisher<robotis_controller_msgs::msg::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/enable_ctrl_module", 1);
  sync_write_pub_ = this->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 1);

  tune_pose_path_ = ament_index_cpp::get_package_share_directory("op3_tuning_module") + "/data/tune_pose.yaml";
  offset_path_ = ament_index_cpp::get_package_share_directory("op3_manager") + "/config/offset.yaml";
  init_file_path_ = ament_index_cpp::get_package_share_directory("op3_manager") + "/config/dxl_init_OP3.yaml";

  this->declare_parameter<std::string>("offset_file_path", offset_path_);
  this->declare_parameter<std::string>("init_file_path", init_file_path_);

  this->get_parameter<std::string>("offset_file_path", offset_path_);
  this->get_parameter<std::string>("init_file_path", init_file_path_);

  parseOffsetData(offset_path_);
  parseInitPoseData(tune_pose_path_);
  parseDxlInit(init_file_path_);
}

void TuningModule::moveToInitPose()
{
  if(enable_ == false)
  {
    // set tuning_module
    callServiceSettingModule(module_name_);

    // wait for changing the module to base_module and getting the goal position
    while (enable_ == false || has_goal_joints_ == false)
      std::this_thread::sleep_for(std::chrono::milliseconds(8));
  }

  if (tra_gene_thread_.joinable())
    tra_gene_thread_.join();

  // get target pose from yaml file
  parseInitPoseData(tune_pose_path_);

  // generate a trajectory
  tra_gene_thread_ = std::thread(&TuningModule::targetPoseTrajGenerateProc, this);
}

void TuningModule::moveToTunePose(const std::string &pose_name)
{
  if(enable_ == false)
  {
    RCLCPP_ERROR(this->get_logger(), "op3_tuning_module is not enable!!");
    return;
  }

  if (tra_gene_thread_.joinable())
    tra_gene_thread_.join();

  // get target pose from yaml file
  parseTunePoseData(tune_pose_path_, pose_name);

  // generate a trajectory
  tra_gene_thread_ = std::thread(&TuningModule::targetPoseTrajGenerateProc, this);
}

bool TuningModule::parseOffsetData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Fail to load offset yaml file.");
    return false;
  }

  //Get Offset Data and Init_Pose for Offset Tuning
  YAML::Node offset_data_node = doc["offset"];

  //Initialize Offset Data in RobotOffsetData
  for (YAML::const_iterator node_it = offset_data_node.begin(); node_it != offset_data_node.end(); node_it++)
  {
    std::string joint_name = node_it->first.as<std::string>();
    double offset = node_it->second.as<double>();

    auto robot_tuning_data_it = robot_tuning_data_.find(joint_name);
    if (robot_tuning_data_it != robot_tuning_data_.end())
      robot_tuning_data_[joint_name]->joint_offset_rad_ = offset;
  }

  return true;
}

bool TuningModule::parseInitPoseData(const std::string &path)
{
  RCLCPP_INFO(this->get_logger(), "parse pose for moving init pose");

  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Fail to load yaml file.: %s", e.what());
    return false;
  }

  YAML::Node init_pose_node;
  init_pose_node = doc["init_pose"];

  if(init_pose_node.IsNull())
  {
    RCLCPP_ERROR(this->get_logger(), "Fail to parse init pose");
    return false;
  }

  // parse movement time
  double move_time;
  move_time = init_pose_node["move_time"].as<double>();

  tuning_module_state_->mov_time_ = move_time;

  // no via pose
  tuning_module_state_->via_num_ = 0;

  // parse target pose
  YAML::Node tar_pose_node = init_pose_node["target_pose"];
  for (YAML::iterator yaml_it = tar_pose_node.begin(); yaml_it != tar_pose_node.end(); ++yaml_it)
  {
    std::string joint_name;
    double value;

    try {
      joint_name = yaml_it->first.as<std::string>();
      value = yaml_it->second.as<double>();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Fail to parse target_pose: %s", e.what());
      return false;
    }

    int id = joint_name_to_id_[joint_name];

    tuning_module_state_->joint_ini_pose_.coeffRef(id, 0) = value * DEGREE2RADIAN;

    // store target value
    auto robot_tuning_data_it = robot_tuning_data_.find(joint_name);
    if (robot_tuning_data_it != robot_tuning_data_.end())
      robot_tuning_data_[joint_name]->goal_position_ = value * DEGREE2RADIAN;
  }

  tuning_module_state_->all_time_steps_ = int(tuning_module_state_->mov_time_ / tuning_module_state_->smp_time_) + 1;
  tuning_module_state_->calc_joint_tra_.resize(tuning_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  return true;
}

bool TuningModule::parseTunePoseData(const std::string &path, const std::string &pose_name)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Fail to load yaml file.");
    return false;
  }

  // get tune pose name
  YAML::Node tune_pose_node;
  tune_pose_node = doc[pose_name];

  if(tune_pose_node.IsNull())
    return false;

  // check via pose
  std::vector<double> move_time;
  std::vector<std::string> target_pose_name;
  move_time = tune_pose_node["move_time"].as< std::vector<double> >();
  target_pose_name = tune_pose_node["target_pose"].as< std::vector<std::string> >();

  if(move_time.size() != target_pose_name.size() || move_time.size() == 0)
    return false;

  // parse movement time
  double total_move_time = std::accumulate(move_time.begin(),move_time.end(),0.0);
  tuning_module_state_->mov_time_ = total_move_time;

  // parse via-point number
  int via_num = move_time.size() - 1;
  tuning_module_state_->via_num_ = via_num;

  // parse via-point time
  tuning_module_state_->via_time_.resize(via_num, 1);
  // parse via-point pose
  tuning_module_state_->joint_via_pose_.resize(via_num, MAX_JOINT_ID + 1);
  tuning_module_state_->joint_via_dpose_.resize(via_num, MAX_JOINT_ID + 1);
  tuning_module_state_->joint_via_ddpose_.resize(via_num, MAX_JOINT_ID + 1);

  tuning_module_state_->joint_via_pose_.fill(0.0);
  tuning_module_state_->joint_via_dpose_.fill(0.0);
  tuning_module_state_->joint_via_ddpose_.fill(0.0);

  YAML::Node pose_data_node = doc["pose_data"];
  double sum_via_time = 0;

  for (int num = 0; num < via_num; num++)
  {
    // set via time
    sum_via_time += move_time[num];
    tuning_module_state_->via_time_.coeffRef(num, 0) = sum_via_time;

    // set via pose
    YAML::Node via_pose_node = pose_data_node[target_pose_name[num]];

    if(via_pose_node.IsNull())
      continue;

    RCLCPP_WARN_STREAM(this->get_logger(), "via : " << num);
    for (YAML::iterator yaml_it = via_pose_node.begin(); yaml_it != via_pose_node.end(); ++yaml_it)
    {
      std::string joint_name;
      double value;

      joint_name = yaml_it->first.as<std::string>();
      value = yaml_it->second.as<double>();
      int id = joint_name_to_id_[joint_name];

      tuning_module_state_->joint_via_pose_.coeffRef(num, id) = value * DEGREE2RADIAN;
      RCLCPP_INFO_STREAM(this->get_logger(), "joint : " << joint_name << ", value : " << value);
    }
  }

  // parse target pose
  YAML::Node tar_pose_node = pose_data_node[target_pose_name[via_num]];
  if(tar_pose_node.IsNull())
    return false;

  for (YAML::iterator yaml_it = tar_pose_node.begin(); yaml_it != tar_pose_node.end(); ++yaml_it)
  {
    std::string joint_name;
    double value;

    joint_name = yaml_it->first.as<std::string>();
    value = yaml_it->second.as<double>();
    int id = joint_name_to_id_[joint_name];

    tuning_module_state_->joint_ini_pose_.coeffRef(id, 0) = value * DEGREE2RADIAN;
    RCLCPP_INFO_STREAM(this->get_logger(), "joint : " << joint_name << ", value : " << value);
  }

  tuning_module_state_->all_time_steps_ = int(tuning_module_state_->mov_time_ / tuning_module_state_->smp_time_) + 1;
  tuning_module_state_->calc_joint_tra_.resize(tuning_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  RCLCPP_INFO_STREAM(this->get_logger(), "tune pose - via_num : " << via_num << ", move_time : " << total_move_time);

  return true;
}

void TuningModule::queueThread()
{
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(this->get_node_base_interface());

  /* subscribe topics */
  auto ini_pose_msg_sub = this->create_subscription<std_msgs::msg::String>("/robotis/tuning_module/tuning_pose", 5, std::bind(&TuningModule::tunePoseMsgCallback, this, std::placeholders::_1));

  joint_offset_data_sub_ = this->create_subscription<op3_tuning_module_msgs::msg::JointOffsetData>("/robotis/tuning_module/joint_offset_data", 10, std::bind(&TuningModule::jointOffsetDataCallback, this, std::placeholders::_1));
  joint_gain_data_sub_ = this->create_subscription<op3_tuning_module_msgs::msg::JointOffsetData>("/robotis/tuning_module/joint_gain_data", 10, std::bind(&TuningModule::jointGainDataCallback, this, std::placeholders::_1));
  joint_torque_enable_sub_ = this->create_subscription<op3_tuning_module_msgs::msg::JointTorqueOnOffArray>("/robotis/tuning_module/torque_enable", 10, std::bind(&TuningModule::jointTorqueOnOffCallback, this, std::placeholders::_1));
  command_sub_ = this->create_subscription<std_msgs::msg::String>("/robotis/tuning_module/command", 5, std::bind(&TuningModule::commandCallback, this, std::placeholders::_1));
  offset_data_server_ = this->create_service<op3_tuning_module_msgs::srv::GetPresentJointOffsetData>("robotis/tuning_module/get_present_joint_offset_data", std::bind(&TuningModule::getPresentJointOffsetDataServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  set_module_client_ = this->create_client<robotis_controller_msgs::srv::SetModule>("/robotis/set_present_ctrl_modules");
  enable_offset_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robotis/enable_offset", 1);
  load_offset_client_ = this->create_client<robotis_controller_msgs::srv::LoadOffset>("/robotis/load_offset");

  rclcpp::Rate rate(1000.0 / control_cycle_msec_);
  while (rclcpp::ok())
  {
    executor.spin_some();
    rate.sleep();
  }
}

void TuningModule::tunePoseMsgCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if(tuning_module_state_->is_generating_ == true)
  {
    RCLCPP_ERROR(this->get_logger(), "Previous pose is generating now.");
    return;
  }

  if (tuning_module_state_->is_moving_ == true)
  {
    RCLCPP_INFO(this->get_logger(), "previous task is alive");
    return;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Go to " << msg->data);

  if (msg->data == "ini_pose")
  {
    moveToInitPose();
  }
  else if (msg->data == "tune_pose_01")
  {
    moveToTunePose(msg->data);
  }
  else if (msg->data == "tune_pose_02")
  {
    moveToTunePose(msg->data);
  }
  else if (msg->data == "tune_pose_03")
  {
    moveToTunePose(msg->data);
  }
  else if (msg->data == "tune_pose_04")
  {
    moveToTunePose(msg->data);
  }

  return;
}

void TuningModule::targetPoseTrajGenerateProc()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = tuning_module_state_->joint_ini_pose_.coeff(id, 0);

    Eigen::MatrixXd tra;

    if (tuning_module_state_->via_num_ == 0)
    {
      tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                  tuning_module_state_->smp_time_, tuning_module_state_->mov_time_);
    }
    else
    {
      Eigen::MatrixXd via_value = tuning_module_state_->joint_via_pose_.col(id);
      Eigen::MatrixXd d_via_value = tuning_module_state_->joint_via_dpose_.col(id);
      Eigen::MatrixXd dd_via_value = tuning_module_state_->joint_via_ddpose_.col(id);

      tra = robotis_framework::calcMinimumJerkTraWithViaPoints(tuning_module_state_->via_num_, ini_value, 0.0, 0.0,
                                                               via_value, d_via_value, dd_via_value, tar_value, 0.0,
                                                               0.0, tuning_module_state_->smp_time_,
                                                               tuning_module_state_->via_time_,
                                                               tuning_module_state_->mov_time_);
    }

    tuning_module_state_->calc_joint_tra_.block(0, id, tuning_module_state_->all_time_steps_, 1) = tra;
  }

  tuning_module_state_->is_moving_ = true;
  tuning_module_state_->cnt_ = 0;
  RCLCPP_INFO(this->get_logger(), "[start] send trajectory");
}

void TuningModule::poseGenerateProc(Eigen::MatrixXd joint_angle_pose)
{
  callServiceSettingModule(module_name_);

  while (enable_ == false || has_goal_joints_ == false)
    std::this_thread::sleep_for(std::chrono::milliseconds(8));

  tuning_module_state_->mov_time_ = 5.0;
  tuning_module_state_->all_time_steps_ = int(tuning_module_state_->mov_time_ / tuning_module_state_->smp_time_) + 1;

  tuning_module_state_->calc_joint_tra_.resize(tuning_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  tuning_module_state_->joint_pose_ = joint_angle_pose;

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = tuning_module_state_->joint_pose_.coeff(id, 0);

    RCLCPP_INFO_STREAM(this->get_logger(), "[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                tuning_module_state_->smp_time_,
                                                                tuning_module_state_->mov_time_);

    tuning_module_state_->calc_joint_tra_.block(0, id, tuning_module_state_->all_time_steps_, 1) = tra;
  }

  tuning_module_state_->is_moving_ = true;
  tuning_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  RCLCPP_INFO(this->get_logger(), "[start] send trajectory");
}

void TuningModule::poseGenerateProc(std::map<std::string, double>& joint_angle_pose)
{
  callServiceSettingModule(module_name_);

  while (enable_ == false || has_goal_joints_ == false)
    std::this_thread::sleep_for(std::chrono::milliseconds(8));

  Eigen::MatrixXd target_pose = Eigen::MatrixXd::Zero(MAX_JOINT_ID + 1, 1);

  for (auto& joint_angle_it : joint_angle_pose)
  {
    std::string joint_name = joint_angle_it.first;
    double joint_angle_rad = joint_angle_it.second;

    auto joint_name_to_id_it = joint_name_to_id_.find(joint_name);
    if (joint_name_to_id_it != joint_name_to_id_.end())
    {
      target_pose.coeffRef(joint_name_to_id_it->second, 0) = joint_angle_rad;
    }
  }

  tuning_module_state_->joint_pose_ = target_pose;

  tuning_module_state_->mov_time_ = 5.0;
  tuning_module_state_->all_time_steps_ = int(tuning_module_state_->mov_time_ / tuning_module_state_->smp_time_) + 1;

  tuning_module_state_->calc_joint_tra_.resize(tuning_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = tuning_module_state_->joint_pose_.coeff(id, 0);

    RCLCPP_INFO_STREAM(this->get_logger(), "[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                tuning_module_state_->smp_time_,
                                                                tuning_module_state_->mov_time_);

    tuning_module_state_->calc_joint_tra_.block(0, id, tuning_module_state_->all_time_steps_, 1) = tra;
  }

  tuning_module_state_->is_moving_ = true;
  tuning_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  RCLCPP_INFO(this->get_logger(), "[start] send trajectory");
}

bool TuningModule::isRunning()
{
  return tuning_module_state_->is_moving_;
}

void TuningModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                           std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  // get dxl data
  for (auto& state_iter : result_)
  {
    std::string joint_name = state_iter.first;

    robotis_framework::Dynamixel *dxl = nullptr;
    auto dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    // applied offset value
    double offset_value = robot_tuning_data_[joint_name]->joint_offset_rad_;
    if (robot_torque_enable_data_[joint_name] == false)
      offset_value = 0.0;

    double joint_pres_position = dxl->dxl_state_->present_position_ - offset_value;
    double joint_goal_position = dxl->dxl_state_->goal_position_ - offset_value;
    int p_gain = dxl->dxl_state_->position_p_gain_;
    int i_gain = dxl->dxl_state_->position_i_gain_;
    int d_gain = dxl->dxl_state_->position_d_gain_;

    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_pres_position;
    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].p_gain_ = p_gain;
    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].i_gain_ = i_gain;
    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].d_gain_ = d_gain;

    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].p_gain_ = p_gain;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].i_gain_ = i_gain;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].d_gain_ = d_gain;
  }

  has_goal_joints_ = true;

  // check data to set
  // if moving
  if (tuning_module_state_->is_moving_ == true)
  {
    if (tuning_module_state_->cnt_ == 1)
      publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Start Init Pose");

    for (int id = 1; id <= MAX_JOINT_ID; id++)
      joint_state_->goal_joint_state_[id].position_ = tuning_module_state_->calc_joint_tra_(tuning_module_state_->cnt_, id);

    tuning_module_state_->cnt_++;
  }
  // if has tuning data
  else if (get_tuning_data_ == true)
  {
    // update tuning data
    // check whether lock is possible
    if (data_mutex_.try_lock() == true)
    {
      // get joint name
      std::string joint_name;
      tuning_data_.joint_name_.getValue(joint_name);

      auto joint_name_to_id_it = joint_name_to_id_.find(joint_name);
      if (joint_name_to_id_it != joint_name_to_id_.end())
      {
        // get tuning data
        int joint_id = joint_name_to_id_it->second;
        tuning_data_.position_.getValue(joint_state_->goal_joint_state_[joint_id].position_);

        tuning_data_.p_gain_.getValue(joint_state_->goal_joint_state_[joint_id].p_gain_);
        tuning_data_.i_gain_.getValue(joint_state_->goal_joint_state_[joint_id].i_gain_);
        tuning_data_.d_gain_.getValue(joint_state_->goal_joint_state_[joint_id].d_gain_);
      }

      // turn off the flag
      get_tuning_data_ = false;
      data_mutex_.unlock();
    }
  }

  // set goal state to result
  for (auto& state_iter : result_)
  {
    std::string joint_name = state_iter.first;

    if (robot_torque_enable_data_[joint_name] == false)
    {
      robot_tuning_data_[joint_name]->joint_offset_rad_ =
          joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ - robot_tuning_data_[joint_name]->goal_position_;
      joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = robot_tuning_data_[joint_name]->goal_position_;
    }

    // set goal position & store value to robot_tuning_data_
    double offset_value = robot_tuning_data_[joint_name]->joint_offset_rad_;

    result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ + offset_value;
    robot_tuning_data_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;

    // set pid gain
    int p_gain = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].p_gain_;
    int i_gain = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].i_gain_;
    int d_gain = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].d_gain_;

    if (p_gain != NONE_GAIN)
    {
      result_[joint_name]->position_p_gain_ = p_gain;
      robot_tuning_data_[joint_name]->p_gain_ = p_gain;
    }
    if (i_gain != NONE_GAIN)
    {
      result_[joint_name]->position_i_gain_ = i_gain;
      robot_tuning_data_[joint_name]->i_gain_ = i_gain;
    }
    if (d_gain != NONE_GAIN)
    {
      result_[joint_name]->position_d_gain_ = d_gain;
      robot_tuning_data_[joint_name]->d_gain_ = d_gain;
    }
  }

  /*---------- initialize count number ----------*/
  if ((tuning_module_state_->cnt_ >= tuning_module_state_->all_time_steps_) && (tuning_module_state_->is_moving_ == true))
  {
    RCLCPP_INFO(this->get_logger(), "[end] send trajectory");

    publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Finish Init Pose");

    tuning_module_state_->is_moving_ = false;
    tuning_module_state_->cnt_ = 0;

    // set all joints -> none
    if (ini_pose_only_ == true)
    {
      setCtrlModule("none");
      ini_pose_only_ = false;
    }
  }
}

void TuningModule::stop()
{
  return;
}

void TuningModule::onModuleEnable()
{
  RCLCPP_INFO(this->get_logger(), "Tuning module is enabled");

  // load offset file
  parseOffsetData(offset_path_);

  // turn off offset function
  turnOnOffOffset(false);
}

void TuningModule::onModuleDisable()
{
  tuning_data_.clearData();
  has_goal_joints_ = false;

  // turn on offset function
  turnOnOffOffset(true);
}

void TuningModule::setCtrlModule(std::string module)
{
  std_msgs::msg::String control_msg;
  control_msg.data = module_name_;

  set_ctrl_module_pub_->publish(control_msg);
}

void TuningModule::callServiceSettingModule(const std::string &module_name)
{
  auto request = std::make_shared<robotis_controller_msgs::srv::SetModule::Request>();
  request->module_name = module_name;

  if (!set_module_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect server");
    return;
  }

  auto future = set_module_client_->async_send_request(request, 
    [this](rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedFuture result)
    {
      if (result.get()->result == false)
        RCLCPP_ERROR(this->get_logger(), "[TuningModule::callServiceSettingModule] Failed to set module");
    });

  return;
}

void TuningModule::publishStatusMsg(unsigned int type, std::string msg)
{
  auto status_msg = robotis_controller_msgs::msg::StatusMsg();
  status_msg.header.stamp = rclcpp::Clock().now();
  status_msg.type = type;
  status_msg.module_name = "Tune";
  status_msg.status_msg = msg;

  status_msg_pub_->publish(status_msg);
}

// command for tuner
void TuningModule::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "save_offset")
  {
    // save current offset to yaml file
    saveOffsetToYaml(offset_path_);

    // send a command to robotis_controller to load the new offset file
    loadOffsetToController(offset_path_);
  }
  else if(msg->data == "save_gain")
  {
    // save gain to dxl init file
    saveDxlInit(init_file_path_);
  }
  else
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Invalid Command : " << msg->data);
  }
}

void TuningModule::jointOffsetDataCallback(const op3_tuning_module_msgs::msg::JointOffsetData::SharedPtr msg)
{
  if (this->enable_ == false)
  {
    RCLCPP_ERROR(this->get_logger(), "Tuning module is not enable");
    return;
  }

  if (tuning_module_state_->is_moving_ == true)
  {
    RCLCPP_ERROR(this->get_logger(), "Robot is moving, joint offset data will not be applied.");
    return;
  }

  //goal position
  RCLCPP_INFO_STREAM(
        this->get_logger(), msg->joint_name << " " << msg->goal_value << " " << msg->offset_value << " " << msg->p_gain <<" " << msg->i_gain <<" " << msg->d_gain);

  auto map_it = robot_tuning_data_.find(msg->joint_name);
  if (map_it == robot_tuning_data_.end())
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid Joint Name");
    return;
  }

  if (robot_torque_enable_data_[msg->joint_name] == false)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), msg->joint_name << "is turned off the torque");
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  tuning_data_.joint_name_.setValue(msg->joint_name);
  tuning_data_.position_.setValue(msg->goal_value);

  // store tuning data
  robot_tuning_data_[msg->joint_name]->goal_position_ = msg->goal_value;
  robot_tuning_data_[msg->joint_name]->joint_offset_rad_ = msg->offset_value;

  // flag on to update joint data
  get_tuning_data_ = true;
}

void TuningModule::jointGainDataCallback(const op3_tuning_module_msgs::msg::JointOffsetData::SharedPtr msg)
{
  if (this->enable_ == false)
  {
    RCLCPP_ERROR(this->get_logger(), "Tuning module is not enable");
    return;
  }

  if (tuning_module_state_->is_moving_ == true)
  {
    RCLCPP_ERROR(this->get_logger(), "Robot is moving, joint gain data will not be applied.");
    return;
  }

  auto map_it = robot_tuning_data_.find(msg->joint_name);
  if (map_it == robot_tuning_data_.end())
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid Joint Name");
    return;
  }

  if (robot_torque_enable_data_[msg->joint_name] == false)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), msg->joint_name << "is turned off the torque");
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);

  tuning_data_.joint_name_.setValue(msg->joint_name);
  tuning_data_.p_gain_.setValue(msg->p_gain);
  tuning_data_.i_gain_.setValue(msg->i_gain);
  tuning_data_.d_gain_.setValue(msg->d_gain);

  robot_tuning_data_[msg->joint_name]->p_gain_ = msg->p_gain;
  robot_tuning_data_[msg->joint_name]->i_gain_ = msg->i_gain;
  robot_tuning_data_[msg->joint_name]->d_gain_ = msg->d_gain;

  // flag on to update joint data
  get_tuning_data_ = true;
}

void TuningModule::jointTorqueOnOffCallback(const op3_tuning_module_msgs::msg::JointTorqueOnOffArray::SharedPtr msg)
{  
  auto syncwrite_msg = robotis_controller_msgs::msg::SyncWriteItem();
  syncwrite_msg.item_name = "torque_enable";

  for (unsigned int i = 0; i < msg->torque_enable_data.size(); i++)
  {
    std::string joint_name = msg->torque_enable_data[i].joint_name;
    bool torque_enable = msg->torque_enable_data[i].torque_enable;
    RCLCPP_INFO_STREAM(this->get_logger(), i <<" " << joint_name << torque_enable);

    auto map_it = robot_tuning_data_.find(joint_name);
    if (map_it == robot_tuning_data_.end())
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid Joint Name");
      continue;
    }
    else
    {
      int torque_value = torque_enable ? 1 : 0;
      syncwrite_msg.joint_name.push_back(joint_name);
      syncwrite_msg.value.push_back(torque_value);

      robot_torque_enable_data_[joint_name] = torque_enable;
    }
  }

  if(syncwrite_msg.joint_name.size() != 0)
    sync_write_pub_->publish(syncwrite_msg);
}

bool TuningModule::getPresentJointOffsetDataServiceCallback(
    const std::shared_ptr<op3_tuning_module_msgs::srv::GetPresentJointOffsetData::Request> req,
    std::shared_ptr<op3_tuning_module_msgs::srv::GetPresentJointOffsetData::Response> res)
{

  RCLCPP_INFO(this->get_logger(), "GetPresentJointOffsetDataService Called");

  for (auto map_it = robot_tuning_data_.begin(); map_it != robot_tuning_data_.end(); map_it++)
  {
    std::string joint_name = map_it->first;
    JointOffsetData* joint_data = map_it->second;

    op3_tuning_module_msgs::msg::JointOffsetPositionData joint_offset_pos;

    // get present joint value
    //...
    double present_value = joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_;

    // set message for respond
    joint_offset_pos.joint_name = joint_name;
    joint_offset_pos.goal_value = joint_data->goal_position_;
    joint_offset_pos.offset_value = joint_data->joint_offset_rad_;
    joint_offset_pos.present_value = present_value; // present value of this joint
    joint_offset_pos.p_gain = joint_data->p_gain_;
    joint_offset_pos.i_gain = joint_data->i_gain_;
    joint_offset_pos.d_gain = joint_data->d_gain_;

    res->present_data_array.push_back(joint_offset_pos);
  }
  return true;
}

bool TuningModule::turnOnOffOffset(bool turn_on)
{
  RCLCPP_WARN(this->get_logger(), "Try to turn on/off offset in robotis_controller");

  auto enable_offset_msg = std_msgs::msg::Bool();
  enable_offset_msg.data = turn_on;

  enable_offset_pub_->publish(enable_offset_msg);

  return true;
}

bool TuningModule::loadOffsetToController(const std::string &path)
{
  auto request = std::make_shared<robotis_controller_msgs::srv::LoadOffset::Request>();
  request->file_path = path;

  auto future = load_offset_client_->async_send_request(request,
    [this](rclcpp::Client<robotis_controller_msgs::srv::LoadOffset>::SharedFuture result)
    {
      auto request_result = result.get();
      if (request_result) 
      {
        if (request_result->result == true)
          RCLCPP_INFO(this->get_logger(), "succeed to let robotis_controller load the offset");
        else
          RCLCPP_ERROR(this->get_logger(), "Failed to let robotis_controller load the offset");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Service server is not responded for turning on/off offset");
      }
    });

  return true; // 비동기 요청 시작 후 바로 true 반환
}

void TuningModule::saveOffsetToYaml(const std::string &path)
{
  YAML::Emitter yaml_out;
  std::map<std::string, double> offset;
  for (auto map_it = robot_tuning_data_.begin(); map_it != robot_tuning_data_.end(); map_it++)
  {
    std::string joint_name = map_it->first;
    JointOffsetData* joint_data = map_it->second;

    offset[joint_name] = joint_data->joint_offset_rad_;  // edit one of the nodes
  }

  yaml_out << YAML::BeginMap;
  yaml_out << YAML::Key << "offset" << YAML::Value << offset;
  yaml_out << YAML::EndMap;
  std::ofstream fout(path.c_str());
  fout << yaml_out.c_str();  // dump it back into the file
  return;
}

void TuningModule::parseDxlInit(const std::string &path)
{
  RCLCPP_WARN(this->get_logger(), "Get the init gain from Dxl init file");
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Fail to load dxl init yaml file.");
    return;
  }

  for (auto map_it = robot_tuning_data_.begin(); map_it != robot_tuning_data_.end(); map_it++)
  {
    std::string joint_name = map_it->first;
    JointOffsetData* joint_data = map_it->second;

    YAML::Node joint_node = doc[joint_name];

    if(joint_node.IsNull())
      continue;

    if(joint_node["position_p_gain"].IsDefined())
      joint_data->p_gain_ = joint_node["position_p_gain"].as<int>();
    else
      joint_data->p_gain_ = NONE_GAIN;
    if(joint_node["position_i_gain"].IsDefined())
      joint_data->i_gain_ = joint_node["position_i_gain"].as<int>();
    else
      joint_data->i_gain_ = NONE_GAIN;
    if(joint_node["position_d_gain"].IsDefined())
      joint_data->d_gain_ = joint_node["position_d_gain"].as<int>();
    else
      joint_data->d_gain_ = NONE_GAIN;
  }
}

void TuningModule::saveDxlInit(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Fail to load dxl init yaml file.");
    return;
  }

  YAML::Emitter yaml_out;
  yaml_out << YAML::BeginMap;

  for (auto map_it = robot_tuning_data_.begin(); map_it != robot_tuning_data_.end(); map_it++)
  {
    std::string joint_name = map_it->first;
    JointOffsetData* joint_data = map_it->second;

    YAML::Node joint_node = doc[joint_name];

    if(joint_node.IsNull())
      continue;

    std::map<std::string, int> dxl_init_param;

    for (YAML::const_iterator node_it = joint_node.begin(); node_it != joint_node.end(); node_it++)
    {
      std::string init_param = node_it->first.as<std::string>();
      int init_value = node_it->second.as<int>();
      dxl_init_param[init_param] = init_value;
    }

    dxl_init_param["position_p_gain"] = joint_data->p_gain_;
    dxl_init_param["position_i_gain"] = joint_data->i_gain_;
    dxl_init_param["position_d_gain"] = joint_data->d_gain_;

    yaml_out << YAML::Key << joint_name << YAML::Value << dxl_init_param;
  }

  yaml_out << YAML::EndMap;
  std::ofstream fout(path.c_str());
  fout << yaml_out.c_str();  // dump it back into the file

  return;
}

}
