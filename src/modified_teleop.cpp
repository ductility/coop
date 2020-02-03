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

#include "modified_teleop/modified_teleop.h"
#include <unistd.h>

OpenManipulatorTeleop::OpenManipulatorTeleop()
    :node_handle_(""),
     priv_node_handle_("~")
{
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);
  input_kinematic_position_.resize(3, 0.0);
  input_kinematic_orientation_.resize(4);
  input_gripper_angle_.resize(1);

  initClient();
  initSubscriber();

  ROS_INFO("OpenManipulator initialization");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
}

void OpenManipulatorTeleop::initClient()
{
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  goal_task_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path");
  goal_task_space_path_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");

  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
}
void OpenManipulatorTeleop::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback, this);
  input_kinematics_pose_sub_ = node_handle_.subscribe("input_kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseInput, this);
  input_gripper_sub_ = node_handle_.subscribe("input_gripper_angle", 10, &OpenManipulatorTeleop::gripperInput, this);
}

void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for(std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;

}

void OpenManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

void OpenManipulatorTeleop::kinematicsPoseInput(const geometry_msgs::Pose::ConstPtr &msg)
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
  ROS_INFO("run\n");
  setGoal('6');
}

void OpenManipulatorTeleop::gripperInput(const std_msgs::Float64::ConstPtr &msg)
{
  std::vector<double> joint_angle;
  joint_angle.push_back(msg->data);
  setToolControl(joint_angle);
  ROS_INFO("gripper on/of \n");
}

std::vector<double> OpenManipulatorTeleop::getPresentJointAngle()
{
  return present_joint_angle_;
}
std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

//inputkinematic position return
std::vector<double> OpenManipulatorTeleop::getInputKinematicsPose()
{
  return input_kinematic_position_;
}

bool OpenManipulatorTeleop::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
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

bool OpenManipulatorTeleop::setToolControl(std::vector<double> joint_angle)
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

bool OpenManipulatorTeleop::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if(goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

//add task space path ===> position only
bool OpenManipulatorTeleop::setTaskSpacePathPositionOnly(std::vector<double> kinematics_pose, double path_time)
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

//add task space path ===> position + orientation
bool OpenManipulatorTeleop::setTaskSpacePath(std::vector<double> kinematics_pose, std::vector<double> kinematics_orientation, double path_time)
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

void OpenManipulatorTeleop::printText()
{
  printf("\n");
  printf("---------------------------\n");

  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));
  printf("---------------------------\n");
}

void OpenManipulatorTeleop::setGoal(char ch)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);

  if(ch == 'g' || ch == 'G')
  {
    printf("input : g \topen gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
  }
  else if(ch == 'f' || ch == 'F')
  {
    printf("input : f \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    setToolControl(joint_angle);
  }

  else if(ch == '2')
  {
    printf("input : 2 \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if(ch == '1')
  {
    printf("input : 1 \tinit pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if(ch == '3')
  {
    printf("input : 3 \tprint orientation\n");

    printf("input ori X: %.3lf Y: %.3lf Z: %.3lf\n",
    input_kinematic_orientation_.at(0),
    input_kinematic_orientation_.at(1),
    input_kinematic_orientation_.at(2));

    // std::vector<std::string> joint_name;
    // std::vector<double> joint_angle;
    // double path_time = 2.0;
    // joint_name.push_back("joint1"); joint_angle.push_back(1.0);
    // joint_name.push_back("joint2"); joint_angle.push_back(1.0);
    // joint_name.push_back("joint3"); joint_angle.push_back(1.0);
    // joint_name.push_back("joint4"); joint_angle.push_back(1.0);
    // setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if(ch == '4')
  {
    printf("input : 4 \tnew pose by position modify\n");
    //입력받아서하기
    // input_kinematic_position_.at(0) = 0.1025;
    // input_kinematic_position_.at(1) = 0.1060;
    // input_kinematic_position_.at(2) = 0.2380;

    goalPose.at(0) = input_kinematic_position_.at(0) - getPresentKinematicsPose().at(0);
    goalPose.at(1) = input_kinematic_position_.at(1) - getPresentKinematicsPose().at(1);
    goalPose.at(2) = input_kinematic_position_.at(2) - getPresentKinematicsPose().at(2);

    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == '5')
  {
    printf("input : 5 \tpose + orientation\n");

    std::vector<double> goalOrientation;
    goalOrientation.resize(4);

    goalPose.at(0) = input_kinematic_position_.at(0);
    goalPose.at(1) = input_kinematic_position_.at(1);
    goalPose.at(2) = input_kinematic_position_.at(2);

    goalOrientation.at(0) = input_kinematic_orientation_.at(0);
    goalOrientation.at(1) = input_kinematic_orientation_.at(1);
    goalOrientation.at(2) = input_kinematic_orientation_.at(2);
    goalOrientation.at(3) = input_kinematic_orientation_.at(3);

    setTaskSpacePath(goalPose, goalOrientation, PATH_TIME);
  }
  else if(ch == '6')
  {
    printf("input : 6 \tpose + orientation\n");

    // std::vector<double> goalOrientation;
    // goalOrientation.resize(4);

    goalPose.at(0) = input_kinematic_position_.at(0);
    goalPose.at(1) = input_kinematic_position_.at(1);
    goalPose.at(2) = input_kinematic_position_.at(2);

    // goalOrientation.at(0) = input_kinematic_orientation_.at(0);
    // goalOrientation.at(1) = input_kinematic_orientation_.at(1);
    // goalOrientation.at(2) = input_kinematic_orientation_.at(2);
    // goalOrientation.at(3) = input_kinematic_orientation_.at(3);

    setTaskSpacePathPositionOnly(goalPose, PATH_TIME);
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_modified_teleop");

  OpenManipulatorTeleop openManipulatorTeleop;

  ROS_INFO("OpenManipulator modified teleop launched");

  ros::spin();

  return 0;
}
