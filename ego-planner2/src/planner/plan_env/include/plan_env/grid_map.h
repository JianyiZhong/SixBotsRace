#ifndef _GRID_MAP_
#define _GRID_MAP_
#include <sophus/se3.h>  // 确保这里包含了 Sophus 的头文件
#include <sophus/so3.h>
#include <mlmap.h>

class GridMap : public mlmap 
{
public:
  GridMap() {}
  ~GridMap() {}

  void initMap(ros::NodeHandle &nh) {
    init_map(nh);
  };
  inline int getOccupancy(Eigen::Vector3d pos) {
      return mlmap::getInflateOccupancy(pos) == OCCUPIED;
  };
  inline int getInflateOccupancy(Eigen::Vector3d pos) {
      return mlmap::getInflateOccupancy(pos) == OCCUPIED;
  };
  inline double getResolution() {
      return mlmap::local_map->map_dxyz_obv_sub;
  };
  bool getOdomDepthTimeout() {
    // ROS_INFO("[GRID_MAP] last_sensor_time: %.3f s", mlmap::last_sensor_time.toSec());
    return (ros::Time::now() - mlmap::last_sensor_time).toSec() > 0.5;
  }
  void setDepthIntrinsic(double fx, double fy, double cx, double cy)
  {
      mlmap::cx_ = cx;
      mlmap::cy_ = cy;
      mlmap::fx_ = fx;
      mlmap::fy_ = fy;
  };

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
};

#endif
