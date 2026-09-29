#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace ego_planner
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node = nh;

    goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    global_list_pub = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    init_list_pub = nh.advertise<visualization_msgs::Marker>("init_list", 2);
    optimal_list_pub = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
    failed_list_pub = nh.advertise<visualization_msgs::Marker>("failed_list", 2);
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);
    ring_pub = nh.advertise<visualization_msgs::Marker>("ring", 2);
    // intermediate_pt0_pub = nh.advertise<visualization_msgs::Marker>("pt0_dur_opt", 10);
    // intermediate_grad0_pub = nh.advertise<visualization_msgs::MarkerArray>("grad0_dur_opt", 10);
    // intermediate_pt1_pub = nh.advertise<visualization_msgs::Marker>("pt1_dur_opt", 10);
    // intermediate_grad1_pub = nh.advertise<visualization_msgs::MarkerArray>("grad1_dur_opt", 10);
    // intermediate_grad_smoo_pub = nh.advertise<visualization_msgs::MarkerArray>("smoo_grad_dur_opt", 10);
    // intermediate_grad_dist_pub = nh.advertise<visualization_msgs::MarkerArray>("dist_grad_dur_opt", 10);
    // intermediate_grad_feas_pub = nh.advertise<visualization_msgs::MarkerArray>("feas_grad_dur_opt", 10);
    // intermediate_grad_swarm_pub = nh.advertise<visualization_msgs::MarkerArray>("swarm_grad_dur_opt", 10);
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "odom";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      if (show_sphere) sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    if (show_sphere) pub.publish(sphere);
    pub.publish(line_strip);
  }

  // real ids used: {id, id+1}
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "odom";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "odom";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "odom";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub.publish(sphere);
  }

  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (global_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    static int last_nums = 0;

    for ( int id=0; id<last_nums; id++ )
    {
      Eigen::Vector4d color(0, 0, 0, 0);
      vector<Eigen::Vector3d> blank;
      displayMarkerList(init_list_pub, blank, scale, color, id, false);
      ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for ( int id=0; id<(int)init_trajs.size(); id++ )
    {
      Eigen::Vector4d color(0, 0, 1, 0.7);
      displayMarkerList(init_list_pub, init_trajs[id], scale, color, id, false);
      ros::Duration(0.001).sleep();
      last_nums++;
    }

  }

  void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(init_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayMultiOptimalPathList(vector<vector<Eigen::Vector3d>> optimal_trajs, const double scale) // zxzxzx
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    static int last_nums = 0;

    for ( int id=0; id<last_nums; id++ )
    {
      Eigen::Vector4d color(0, 0, 0, 0);
      vector<Eigen::Vector3d> blank;
      displayMarkerList(optimal_list_pub, blank, scale, color, id + 10, false);
      ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for ( int id=0; id<(int)optimal_trajs.size(); id++ )
    {
      Eigen::Vector4d color(1, 0, 0, 0.7);
      displayMarkerList(optimal_list_pub, optimal_trajs[id], scale, color, id + 10, false);
      ros::Duration(0.001).sleep();
      last_nums++;
    }

  }

  void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(1, 1, 1, 1);
    displayMarkerList(optimal_list_pub, list, 0.15, color, id);
  }

  void PlanningVisualization::displayFailedList(Eigen::MatrixXd failed_pts, int id)
  {

    if (failed_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < failed_pts.cols(); i++)
    {
      Eigen::Vector3d pt = failed_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(0.3, 0, 0, 1);
    displayMarkerList(failed_list_pub, list, 0.15, color, id);
  }
  
  void PlanningVisualization::displayRing(Eigen::VectorXd ringVec)
  {
    if (ringVec.size() < 6)
    {
      ROS_ERROR("displayRing: ringVec must have at least 6 elements (center[3], normal[3])");
      return;
    }

    Eigen::Vector3d center = ringVec.segment<3>(0);
    Eigen::Vector3d normal = ringVec.segment<3>(3);
    if (normal.norm() < 1e-8)
    {
      ROS_ERROR("displayRing: normal vector is zero");
      return;
    }
    normal.normalize();

    // build an orthonormal basis (u,v) perpendicular to normal
    Eigen::Vector3d arb(1.0, 0.0, 0.0);
    if (std::abs(normal.dot(arb)) > 0.9)
      arb = Eigen::Vector3d(0.0, 1.0, 0.0);

    Eigen::Vector3d u = normal.cross(arb).normalized();
    Eigen::Vector3d v = normal.cross(u); // already normalized if normal and u are unit

    // parameters for the ring
    const double radius = 0.5;    // adjust radius as needed
    const int segments = 64;      // more segments -> smoother circle
    const double two_pi = 2.0 * M_PI;

    // Ring marker
    visualization_msgs::Marker ring_marker;
    ring_marker.header.frame_id = "odom";
    ring_marker.header.stamp = ros::Time::now();
    ring_marker.ns = "ring";
    ring_marker.id = 2000;
    ring_marker.type = visualization_msgs::Marker::LINE_STRIP;
    ring_marker.action = visualization_msgs::Marker::ADD;
    ring_marker.pose.orientation.w = 1.0;
    ring_marker.scale.x = 0.02; // line width
    ring_marker.color.r = 0.2;
    ring_marker.color.g = 1.0;
    ring_marker.color.b = 0.0;
    ring_marker.color.a = 1.0;

    ring_marker.points.reserve(segments + 1);
    for (int i = 0; i <= segments; ++i)
    {
      double theta = two_pi * double(i) / double(segments);
      Eigen::Vector3d p = center + radius * (std::cos(theta) * u + std::sin(theta) * v);
      geometry_msgs::Point pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
      ring_marker.points.push_back(pt);
    }

    // Center marker (sphere)
    visualization_msgs::Marker center_marker;
    center_marker.header.frame_id = "odom";
    center_marker.header.stamp = ros::Time::now();
    center_marker.ns = "ring_center";
    center_marker.id = 2001;
    center_marker.type = visualization_msgs::Marker::SPHERE;
    center_marker.action = visualization_msgs::Marker::ADD;
    center_marker.pose.orientation.w = 1.0;
    center_marker.pose.position.x = center.x();
    center_marker.pose.position.y = center.y();
    center_marker.pose.position.z = center.z();
    center_marker.scale.x = 0.12;
    center_marker.scale.y = 0.12;
    center_marker.scale.z = 0.12;
    center_marker.color.r = 0.0;
    center_marker.color.g = 0.5;
    center_marker.color.b = 1.0;
    center_marker.color.a = 1.0;

    // Normal marker (arrow)
    visualization_msgs::Marker normal_marker;
    normal_marker.header.frame_id = "odom";
    normal_marker.header.stamp = ros::Time::now();
    normal_marker.ns = "ring_normal";
    normal_marker.id = 2002;
    normal_marker.type = visualization_msgs::Marker::ARROW;
    normal_marker.action = visualization_msgs::Marker::ADD;
    normal_marker.pose.orientation.w = 1.0;
    // scale.x is the shaft diameter, and scale.y is the head diameter. If scale.z is not zero, it specifies the head length.
    normal_marker.scale.x = 0.05; // length of the arrow relative to the two points (unused if points provided, but keep sane)
    normal_marker.scale.y = 0.3;
    normal_marker.scale.z = 0.4;
    normal_marker.color.r = 1.0;
    normal_marker.color.g = 0.0;
    normal_marker.color.b = 0.0;
    normal_marker.color.a = 1.0;

    geometry_msgs::Point p_start, p_end;
    p_start.x = center.x();
    p_start.y = center.y();
    p_start.z = center.z();
    const double normal_display_len = 0.7; // length of the displayed normal
    Eigen::Vector3d end = center + normal_display_len * normal;
    p_end.x = end.x();
    p_end.y = end.y();
    p_end.z = end.z();

    normal_marker.points.clear();
    normal_marker.points.push_back(p_start);
    normal_marker.points.push_back(p_end);

    // publish markers (same topic can carry multiple markers distinguished by id)
    ring_pub.publish(ring_marker);
    ring_pub.publish(center_marker);
    ring_pub.publish(normal_marker);
  }
  
  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {

    if (a_star_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    for (auto block : a_star_paths)
    {
      list.clear();
      for (auto pt : block)
      {
        list.push_back(pt);
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      displayMarkerList(a_star_list_pub, list, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
      i++;
    }
  }

  void PlanningVisualization::displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;
    // clear
    pub.publish(array);

    generateArrowDisplayArray(array, list, scale, color, id);

    pub.publish(array);
  }

  void PlanningVisualization::displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color)
  {
    std::vector<Eigen::Vector3d> pts_;
    pts_.reserve(pts.cols());
    for ( int i=0; i<pts.cols(); i++ )
    {
      pts_.emplace_back(pts.col(i));
    }

    if ( !type.compare("0") )
    {
      displayMarkerList(intermediate_pt0_pub, pts_, 0.1, color, id);
    }
    else if ( !type.compare("1") )
    {
      displayMarkerList(intermediate_pt1_pub, pts_, 0.1, color, id);
    }
  }

  void PlanningVisualization::displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color)
  {
    if ( pts.cols() != grad.cols() )
    {
      ROS_ERROR("pts.cols() != grad.cols()");
      return;
    }
    std::vector<Eigen::Vector3d> arrow_;
    arrow_.reserve(pts.cols()*2);
    if ( !type.compare("swarm") )
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(grad.col(i));
      }
    }
    else
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(pts.col(i)+grad.col(i));
      }
    }
    

    if ( !type.compare("grad0") )
    {
      displayArrowList(intermediate_grad0_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("grad1") )
    {
      displayArrowList(intermediate_grad1_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("dist") )
    {
      displayArrowList(intermediate_grad_dist_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("smoo") )
    {
      displayArrowList(intermediate_grad_smoo_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("feas") )
    {
      displayArrowList(intermediate_grad_feas_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("swarm") )
    {
      displayArrowList(intermediate_grad_swarm_pub, arrow_, 0.02, color, id);
    }
    
  }

  // PlanningVisualization::
} // namespace ego_planner
