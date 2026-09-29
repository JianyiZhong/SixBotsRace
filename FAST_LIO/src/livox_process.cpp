#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Points;
typedef pcl::PointCloud<PointType>::Ptr PointsPtr;

visualization_msgs::Marker poseMarker;
visualization_msgs::Marker trajMarker;

ros::Publisher pubPointCloud;
ros::Publisher pubOdom;

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
  PointsPtr pcl_cloud(new Points);
  int plsize = msg->point_num;
  for(uint i=1; i<plsize; i++) {
    PointType p;
    p.x = msg->points[i].x;
    p.y = msg->points[i].y;
    p.z = msg->points[i].z;
    pcl_cloud->points.push_back(p);
  }

  sensor_msgs::PointCloud2 points_ros;
  pcl::toROSMsg(*pcl_cloud, points_ros);
  points_ros.header.stamp = ros::Time::now();
  points_ros.header.frame_id = "world";
  pubPointCloud.publish(points_ros);
}

void traj_cbk(const visualization_msgs::Marker::ConstPtr &msg) {
  visualization_msgs::Marker trajMarker;
  trajMarker = *msg;
}

void odom_cbk(const nav_msgs::Odometry::ConstPtr &msg) {

  poseMarker.header.stamp = ros::Time::now();
  poseMarker.header.frame_id = "world";
  poseMarker.lifetime = ros::Duration(0.1);
  poseMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
  poseMarker.action = visualization_msgs::Marker::ADD;
  
  if(trajMarker.points.size()>1){
    poseMarker.pose.position.x = (trajMarker.points[0].x + msg->pose.pose.position.x)/2.0;
    poseMarker.pose.position.y = (trajMarker.points[0].y + msg->pose.pose.position.y)/2.0;
    poseMarker.pose.position.z = (trajMarker.points[0].z + msg->pose.pose.position.z)/2.0;
  }
  else{
    poseMarker.pose.position.x = msg->pose.pose.position.x;
    poseMarker.pose.position.y = msg->pose.pose.position.y;
    poseMarker.pose.position.z = msg->pose.pose.position.z;
  }

  poseMarker.pose.orientation.w = 1;
  poseMarker.pose.orientation.x = 0;
  poseMarker.pose.orientation.y = 0;
  poseMarker.pose.orientation.z = 0;
  poseMarker.scale.x = 0.8;
  poseMarker.scale.y = 0.8;
  poseMarker.scale.z = 0.8;
  poseMarker.color.a = 1.0;
  poseMarker.color.r = 0.0;
  poseMarker.color.g = 1.0;
  poseMarker.color.b = 0.0;
  poseMarker.mesh_resource = "package://odom_visualization/meshes/fake_drone.dae";
  pubOdom.publish(poseMarker);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_process");
  ros::NodeHandle nh("~");
  pubPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/lidar_points", 100000);
  pubOdom = nh.advertise<visualization_msgs::Marker>("/Odom", 100000);

  ros::Subscriber livox_sub_ = nh.subscribe("/livox/lidar", 20000, livox_pcl_cbk);
  ros::Subscriber traj_sub_ = nh.subscribe("/drone_0_ego_planner_node/optimal_list", 20000, traj_cbk);
  ros::Subscriber odom_sub_ = nh.subscribe("/Odometry", 20000, odom_cbk);

  // ros::Timer odom_timer_ = nh.createTimer(ros::Duration(0.05), markerCallback);

  ros::spin();
  
  return 0;
}