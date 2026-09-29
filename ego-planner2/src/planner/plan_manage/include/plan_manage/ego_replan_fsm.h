#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <controller_msgs/MinTraj.h>
#include <controller_msgs/cmd.h>
#include <quadrotor_msgs/PositionCommand.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {
  public:
    EGOReplanFSM() {}
    ~EGOReplanFSM() {}

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };
    /* Anomaly Detection Parameters */
    Eigen::Vector3d last_local_target_pos_;
    double last_target_change_time_;
    int replan_fail_count_;
    static constexpr double TARGET_STUCK_THRESH = 0.3;  // Threshold for target movement below which it's considered "stuck"
    double TARGET_STUCK_TIME;                           // Default time threshold (seconds) for being considered stuck before reinitialization
    static constexpr int MAX_REPLAN_FAIL_COUNT = 10;    // Threshold for maximum optimization failure count
    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    bool watch_waypoint_;
    double no_replan_thresh_, replan_thresh_;
    std::vector<std::vector<double>> waypoints_;
    int waypoint_num_, wpt_id_, waypoint_dim_;
    double planning_horizen_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool enable_ground_height_measurement_;
    bool flag_escape_emergency_;
    bool need_hover_stop_;
    bool mondify_final_goal_;
    bool enable_stuck_detect_; // Whether to enable stuck detection

    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};
    double last_yaw_err = -1.0;
    bool flag_get_new_ring_info_ = false;
    ros::Time last_ring_info_time_ = ros::Time(0);
    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d final_goal_;                             // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_, odom_euler_;     // odometry state
    Eigen::VectorXd ringVec_; // drill ring vector
    Eigen::VectorXd ringVecCam_; // in camera frame
    Eigen::Matrix3d rotM_;
      /* functions */
    std::vector<Eigen::Vector3d> wps_;
    std::vector<Eigen::Vector3d> watch_dir_;
    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, cmd_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_, ring_sub_;
    ros::Publisher pos_cmd_pub_, gimbal_pub_, poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_,
    shutter_pub_, pause_pub_, ground_height_pub_, traj_server_pause_pub_;

    /* state machine functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void printFSMExecState();
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    void ringCallback(const std_msgs::Float64MultiArray &msg);
    /* safety */
    void checkCollisionCallback(const ros::TimerEvent &e);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    /* local planning */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /* global trajectory */
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void readGivenWpsAndPlan();
    bool planWatchTraj(const Eigen::Vector3d& target, const Eigen::Vector3d& watch_target, const Eigen::Vector3d& nextTarget);
    bool planNextWaypoint(const Eigen::Vector3d next_wp, bool flag_2replan);
    bool mondifyInCollisionFinalGoal();
    void finishProcess();

    /* input-output */
    void mandatoryStopCallback(const std_msgs::Empty &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void RecvBroadcastMINCOTrajCallback(const controller_msgs::MinTrajConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, controller_msgs::MinTraj &MINCO_msg);
    void cmdCallback(const controller_msgs::cmdConstPtr &msg);

    /* ground height measurement */
    bool measureGroundHeight(double &height);
    Eigen::Vector3d projectPointToLineSegment(const Eigen::Vector3d& a,
                                              const Eigen::Vector3d& b,
                                              const Eigen::Vector3d& p);
  };

} // namespace ego_planner

#endif
