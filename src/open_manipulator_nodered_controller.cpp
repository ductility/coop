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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "open_manipulator_nodered_controller/open_manipulator_nodered_controller.h"
#include <unistd.h>

OpenManipulatorNoderedController::OpenManipulatorNoderedController()
    :node_handle_(""), 
     priv_node_handle_("~")
{
  //전역변수 초기화
  joint_name_.push_back("joint1");
  joint_name_.push_back("joint2");
  joint_name_.push_back("joint3");
  joint_name_.push_back("joint4");
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_gripper_angle_.resize(1);
  present_kinematic_position_.resize(3);
  input_kinematic_position_.resize(3);
  input_kinematic_orientation_.resize(4);
  input_gripper_angle_.resize(1);
  start_joint_stamp_.resize(NUM_OF_JOINT);
  end_joint_stamp_.resize(NUM_OF_JOINT);

  initClient();
  initSubscriber();

  ROS_INFO("OpenManipulator initialization");
}

OpenManipulatorNoderedController::~OpenManipulatorNoderedController()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
}

void OpenManipulatorNoderedController::initClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_task_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path");
  goal_task_space_path_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
  set_actuator_state_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetActuatorState>("set_actuator_state");
}
void OpenManipulatorNoderedController::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorNoderedController::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorNoderedController::kinematicsPoseCallback, this);
  input_kinematics_pose_sub_ = node_handle_.subscribe("input_kinematics_pose", 10, &OpenManipulatorNoderedController::kinematicsPoseInputCallback, this);
  input_gripper_sub_ = node_handle_.subscribe("input_gripper_angle", 10, &OpenManipulatorNoderedController::gripperInputCallback, this);
  input_actuator_sub_ = node_handle_.subscribe("input_actuator_state", 10, &OpenManipulatorNoderedController::ActuatorStateCallback, this);
  position_stamp_sub_ = node_handle_.subscribe("joint_angle_stamp", 10, &OpenManipulatorNoderedController::positionStampCallback, this);//position stamp 찍기
  hand_guide_move_point_sub_ = node_handle_.subscribe("hand_guide_move_point", 10, &OpenManipulatorNoderedController::handGuideMovePointCallback, this);
  hand_guide_move_path_sub_ = node_handle_.subscribe("hand_guide_move_path", 10, &OpenManipulatorNoderedController::handGuideMovePathCallback, this);
}

void OpenManipulatorNoderedController::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  std::vector<double> temp_gripper_angle;
  temp_angle.resize(NUM_OF_JOINT);
  temp_gripper_angle.resize(1);
  for(std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;

  if(!msg->name.at(4).compare("gripper")) temp_gripper_angle.at(0) = (msg->position.at(4));//gripper angle 값을 가져옴
  present_gripper_angle_ = temp_gripper_angle;
}

void OpenManipulatorNoderedController::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

void OpenManipulatorNoderedController::kinematicsPoseInputCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  std::vector<double> temp_orientation;

  temp_position.push_back(msg->position.x);
  temp_position.push_back(msg->position.y);
  temp_position.push_back(msg->position.z);

  temp_orientation.push_back(msg->orientation.w);
  temp_orientation.push_back(msg->orientation.x);
  temp_orientation.push_back(msg->orientation.y);
  temp_orientation.push_back(msg->orientation.z);

  input_kinematic_position_ = temp_position;
  input_kinematic_orientation_ = temp_orientation;
  ROS_INFO("Move by set task space path\n");
  setMode('1');
}

void OpenManipulatorNoderedController::gripperInputCallback(const std_msgs::Float64::ConstPtr &msg)
{
  std::vector<double> joint_angle;
  joint_angle.push_back(msg->data);
  setToolControl(joint_angle);
  ROS_INFO("gripper on/off \n");
}

void OpenManipulatorNoderedController::ActuatorStateCallback(const std_msgs::Bool::ConstPtr &msg)
{
  setActuatorState(msg->data);
  if(msg->data){
    ROS_INFO("Actuator Enabled\n");
  }
  else{
    ROS_INFO("Actuator Disabled\n");
  }
}

void OpenManipulatorNoderedController::positionStampCallback(const std_msgs::Bool::ConstPtr &msg)
{
  stamp = msg->data;
  if(stamp){
    record_buffer_.clear();
    start_joint_stamp_ = getPresentJointAngle();
    ROS_INFO("Start Joint Angle Stamp Recorded\n");
  }
  else{
    ROS_INFO("End Joint Angle Stamp Recorded \n");
  }
}

void OpenManipulatorNoderedController::handGuideMovePointCallback(const std_msgs::Bool::ConstPtr &msg)
{
  setJointSpacePath(joint_name_, start_joint_stamp_, PATH_TIME);
  usleep(1000000);
  setJointSpacePath(joint_name_, end_joint_stamp_, PATH_TIME);
  ROS_INFO("Move by recorded Point\n");
}

void OpenManipulatorNoderedController::handGuideMovePathCallback(const std_msgs::Bool::ConstPtr &msg)
{
  playstamp = msg->data;
  buffer_index_ = 0;
  ROS_INFO("Move by recorded Path\n");
}

bool OpenManipulatorNoderedController::setActuatorState(bool actuator_state)
{
  open_manipulator_msgs::SetActuatorState srv;
  srv.request.set_actuator_state = actuator_state;

  if(set_actuator_state_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

std::vector<double> OpenManipulatorNoderedController::getPresentJointAngle()
{
  return present_joint_angle_;
}
std::vector<double> OpenManipulatorNoderedController::getPresentGripperAngle()
{
  return present_gripper_angle_;
}
std::vector<double> OpenManipulatorNoderedController::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

bool OpenManipulatorNoderedController::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorNoderedController::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorNoderedController::setTaskSpacePathPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.end_effector_name = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if(goal_task_space_path_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorNoderedController::setTaskSpacePath(std::vector<double> kinematics_pose, std::vector<double> kinematics_orientation, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.end_effector_name = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kinematics_orientation.at(0);
  srv.request.kinematics_pose.pose.orientation.x = kinematics_orientation.at(1);
  srv.request.kinematics_pose.pose.orientation.y = kinematics_orientation.at(2);
  srv.request.kinematics_pose.pose.orientation.z = kinematics_orientation.at(3);

  srv.request.path_time = path_time;

  if(goal_task_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorNoderedController::printBufferSize()
{
  printf("\n---------------------------\n");
  printf("Start Recording Trajectory\n");
  printf("Buffer Size : %d\n", (int)(record_buffer_.size()));
}

void OpenManipulatorNoderedController::setMode(char ch)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);
  std::vector<double> goalOrientation;  goalOrientation.resize(4);

  if(ch == '1')
  {
    printf("MODE1 : \tby Position + Orientation\n");

    goalPose.at(0) = input_kinematic_position_.at(0);
    goalPose.at(1) = input_kinematic_position_.at(1);
    goalPose.at(2) = input_kinematic_position_.at(2);

    goalOrientation.at(0) = input_kinematic_orientation_.at(0);
    goalOrientation.at(1) = input_kinematic_orientation_.at(1);
    goalOrientation.at(2) = input_kinematic_orientation_.at(2);
    goalOrientation.at(3) = input_kinematic_orientation_.at(3);

    setTaskSpacePath(goalPose, goalOrientation, PATH_TIME);
  }
  else if(ch == '2')
  {
    printf("MODE2 : 6 \tby Position Only\n");

    goalPose.at(0) = input_kinematic_position_.at(0);
    goalPose.at(1) = input_kinematic_position_.at(1);
    goalPose.at(2) = input_kinematic_position_.at(2);

    setTaskSpacePathPositionOnly(goalPose, PATH_TIME);
  }
}

void OpenManipulatorNoderedController::publishCallback(const ros::TimerEvent&)
{
  if(stamp){
    WaypointBuffer temp;
    temp.joint_angle = getPresentJointAngle();
    temp.tool_position = getPresentGripperAngle().at(0);
    record_buffer_.push_back(temp);
    end_joint_stamp_ = temp.joint_angle;

    static int count = 0;
    if(!(count % 5))
    {
      printBufferSize();
    }
    count ++;
  }

  if(playstamp && record_buffer_.size() > buffer_index_)
  {
    std::vector<double> joint_angle;
    joint_angle = record_buffer_.at(buffer_index_).joint_angle;
    std::vector<double> gripper_angle;
    gripper_angle.push_back(record_buffer_.at(buffer_index_).tool_position);

    setJointSpacePath(joint_name_ , joint_angle, LOOP_RATE * 10);
    setToolControl(gripper_angle);
    printf("\n%d",(int)(buffer_index_));
    printf("\nrecorded Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
      record_buffer_.at(buffer_index_).joint_angle.at(0),
      record_buffer_.at(buffer_index_).joint_angle.at(1),
      record_buffer_.at(buffer_index_).joint_angle.at(2),
      record_buffer_.at(buffer_index_).joint_angle.at(3));
    buffer_index_ ++;
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_modified_teleop");
  ros::NodeHandle node_handle("");

  OpenManipulatorNoderedController openManipulatorNoderedController;

  ROS_INFO("OpenManipulator modified teleop launched");

  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(LOOP_RATE), &OpenManipulatorNoderedController::publishCallback, &openManipulatorNoderedController);
  
  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}