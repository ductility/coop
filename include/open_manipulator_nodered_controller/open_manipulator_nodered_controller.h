#ifndef OPEN_MANIPULATOR_NODERED_CONTROLLER_H
#define OPEN_MANIPULATOR_NODERED_CONTROLLER_H
//OPEN_MANIPULATOR_TELEOP에 OPEN_MANIPULATOR_CONTROL_GUI, OPEN_MANIPULATOR_MASTER_SLAVE 를 참고하여 만들었음

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <termios.h>
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetActuatorState.h" //Actuator 토크 제어를 위해 추가

#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#define NUM_OF_JOINT 4
// #define DELTA 0.01
// #define JOINT_DELTA 0.05
#define PATH_TIME 2
#define LOOP_RATE 0.1

typedef struct _WaypointBuffer
{
  std::vector<double> joint_angle;
  double tool_position;
} WaypointBuffer;

class OpenManipulatorNoderedController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  //Client
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_task_space_path_client_;
  ros::ServiceClient goal_task_space_path_position_only_client_;
  ros::ServiceClient goal_tool_control_client_;
  ros::ServiceClient set_actuator_state_client_;

  //Subscriber 
  ros::Subscriber joint_states_sub_;
  ros::Subscriber kinematics_pose_sub_;
  ros::Subscriber input_kinematics_pose_sub_;
  ros::Subscriber input_gripper_sub_;
  ros::Subscriber input_actuator_sub_;
  ros::Subscriber position_stamp_sub_;
  ros::Subscriber hand_guide_move_point_sub_;
  ros::Subscriber hand_guide_move_path_sub_;

  //전역변수 선언
  std::vector<std::string> joint_name_;
  std::vector<double> present_joint_angle_;
  std::vector<double> present_gripper_angle_;
  std::vector<double> present_kinematic_position_;
  std::vector<double> input_kinematic_position_;
  std::vector<double> input_kinematic_orientation_;
  std::vector<double> input_gripper_angle_;
  std::vector<double> start_joint_stamp_;
  std::vector<double> end_joint_stamp_;
  std::vector<WaypointBuffer> record_buffer_;
  int buffer_index_ = 0;

 public:

  OpenManipulatorNoderedController();
  ~OpenManipulatorNoderedController();

  void initClient();
  void initSubscriber();

  //Callback함수 선언, node-red 컨트롤러에서 발행된 토픽을 받으면 실행
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void kinematicsPoseInputCallback(const geometry_msgs::Pose::ConstPtr &msg);
  void gripperInputCallback(const std_msgs::Float64::ConstPtr &msg);
  void ActuatorStateCallback(const std_msgs::Bool::ConstPtr &msg);
  void positionStampCallback(const std_msgs::Bool::ConstPtr &msg);
  void handGuideMovePointCallback(const std_msgs::Bool::ConstPtr &msg);
  void handGuideMovePathCallback(const std_msgs::Bool::ConstPtr &msg);

  //Manipulator의 실시간 정보 가져오기
  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentGripperAngle();
  std::vector<double> getPresentKinematicsPose();

  //Actuator ON/OFF
  bool setActuatorState(bool actuator_state);

  //handguide 레코딩과 실행을 위한 bool 선언
  bool stamp = false;// true : 레코드 루프돌리기, false : 루프탈출.
  bool playstamp = false;// true : 레코드 실행하기

  //Manipulator를 특정 각도 또는 위치로 이동시키기 위한 함수 선언. 함수 내에 service call 하는 내용 포함.
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathPositionOnly(std::vector<double> kinematics_pose, double path_time);
  bool setTaskSpacePath(std::vector<double> kinematics_pose, std::vector<double> kinematics_orientation, double path_time);
  bool setToolControl(std::vector<double> joint_angle);

  //기타 함수
  void publishCallback(const ros::TimerEvent&);//특정 시간간격으로 루프를 돌게 함
  void printBufferSize();
  void setMode(char ch);//task space path를 이용한 이동에서 position만 고려할 것인지, orientation도 같이 고려할 것인지 선택
};

#endif //OPEN_MANIPULATOR_NODERED_CONTROLLER_H
