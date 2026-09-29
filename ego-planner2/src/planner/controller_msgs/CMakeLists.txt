cmake_minimum_required(VERSION 2.8.3)
project(controller_msgs)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs sensor_msgs message_generation std_msgs mavros_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  FlatTarget.msg
  YawTarget.msg
  control.msg
  cmd.msg
  DroneState.msg
  TargetPose.msg
  PoseStamped.msg
  MinTraj.msg
  Trajectory.msg
  SwarmMatrix.msg
  PlanningGoal.msg
  TargetBox.msg
  TargetBoxes.msg
)

add_service_files(DIRECTORY srv
  FILES
  planning.srv
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy geometry_msgs sensor_msgs message_runtime std_msgs)

