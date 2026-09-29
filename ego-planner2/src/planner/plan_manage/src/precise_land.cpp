#include <ros/ros.h>
#include <mavros_msgs/LandingTarget.h>
#include <mavros_msgs/SetMode.h>
// 新增：Odometry 消息头文件
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

class PrecisionLanding {
private:
    ros::NodeHandle nh;

    // 1. 发布器：仅发布精准降落目标
    ros::Publisher target_pub, target_pub_raw;

    // 2. 订阅器：Odometry 里程计（无人机位姿）
    ros::Subscriber odom_sub;
    // 存储无人机位姿（兼容原有坐标转换逻辑）
    geometry_msgs::Pose current_pose;

    // 3. 服务客户端：仅切换飞控模式
    ros::ServiceClient set_mode_client;

    // TF坐标变换
    tf::TransformListener tf_listener;

    // PX4 精准降落模式
    const std::string MODE_PRECISION_LAND = "AUTO.PRECLAND";

public:
    PrecisionLanding() {
        // 初始化降落目标发布器
        target_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/landing_target/pose", 10);

        target_pub_raw = nh.advertise<mavros_msgs::LandingTarget>("/mavros/landing_target/raw", 10);
        // ===================== 关键修改：订阅 Odometry 里程计 =====================
        // 默认话题：/mavros/local_position/odom （MAVROS 标准本地里程计话题）
        // 可根据你的实际话题名修改！
        odom_sub = nh.subscribe("/gt_iris_base_link_imu", 10, &PrecisionLanding::odomCallback, this);

        // 初始化模式切换服务
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        // 等待MAVROS服务启动
        ROS_INFO("wait for MAVROS service connection...");
        ros::service::waitForService("/mavros/set_mode");
        ROS_INFO("service connected, ready to switch to precision landing mode");
    }

    // ===================== 关键修改：Odometry 回调函数 =====================
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 从 Odometry 消息中提取 无人机位姿（位置+姿态）
        current_pose = msg->pose.pose;
    }

    // 切换到精准降落模式
    bool setPrecisionLandMode() {
        mavros_msgs::SetMode srv;
        srv.request.base_mode = 0;
        srv.request.custom_mode = MODE_PRECISION_LAND;

        if (set_mode_client.call(srv) && srv.response.mode_sent) {
            ROS_INFO("✅ successfully switched to (AUTO.PRECLAND)");
            return true;
        } else {
            ROS_ERROR("❌ failed to switch to precision landing mode");
            return false;
        }
    }

    void publishLandingTarget() {
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.stamp = ros::Time::now();
        target_pose.header.frame_id = "odom";  // 必须为 map / local_ned

        // --------------------------
        // 降落目标坐标（NED坐标系）
        // 示例：无人机正下方为降落点（x=0,y=0）
        // 你可以替换为视觉检测的真实坐标
        // --------------------------
        // 地面上做匀速直线运动（以第一次调用为起点）
        static ros::Time start_time = ros::Time::now();
        double t = (ros::Time::now() - start_time).toSec();

        // 线速度（m/s） — 可按需修改
        const double vx = 0.0; // 北向速度（x）
        const double vy = 0.0; // 东向速度（y）

        // 初始位置（t=0）
        const double x0 = 0.0;
        const double y0 = -1.0;

        target_pose.pose.position.x = x0 + vx * t;    // 北
        target_pose.pose.position.y = y0 + vy * t;    // 东
        target_pose.pose.position.z = 0.2;            // 地面（根据需要微调）

        // 默认姿态（水平降落）
        target_pose.pose.orientation.w = 1.0;
        target_pose.pose.orientation.x = 0.0;
        target_pose.pose.orientation.y = 0.0;
        target_pose.pose.orientation.z = 0.0;

        // 发布目标
        target_pub.publish(target_pose);
    }


    // // 发布精准降落目标（核心：坐标转换 + 消息发送）
     void publishLandingTarget_raw() {
         mavros_msgs::LandingTarget msg;
         msg.header.stamp = ros::Time::now();
         // PX4强制要求：NED坐标系
         msg.frame = 1;
         // 视觉 fiducial 标记（ArUco/IR信标）
//         uint8 LIGHT_BEACON = 0             # Landing target signaled by light beacon (ex: IR-LOCK)
//uint8 RADIO_BEACON = 1             # Landing target signaled by radio beacon (ex: ILS, NDB)
//uint8 VISION_FIDUCIAL = 2          # Landing target represented by a fiducial marker (ex: ARTag)
//uint8 VISION_OTHER = 3 
         msg.type = mavros_msgs::LandingTarget::LIGHT_BEACON;

         // ============== 坐标转换：相机坐标系 → 机体坐标系 → NED坐标系 ==============
         // 1. 【替换为你的真实视觉检测结果】相机坐标系下的目标坐标
//         tf::Vector3 target_cam(2.0, 0.0, 2.0);  // 示例：相机正下方2m
//
//         // 2. 相机 → 机体坐标系（根据你的相机实际安装位置修改）
//         tf::Transform cam_to_body;
//         cam_to_body.setOrigin(tf::Vector3(0.0, 0.0, -0.1));  // 相机在机体下方0.1m
//         cam_to_body.setRotation(tf::Quaternion(0, 0, 0, 1));
//         tf::Vector3 target_body = cam_to_body * target_cam;
//
//         // 3. 机体 → NED坐标系（从 Odometry 获取无人机姿态）
//         tf::Quaternion q(
//             current_pose.orientation.x,
//             current_pose.orientation.y,
//             current_pose.orientation.z,
//             current_pose.orientation.w
//         );
//         tf::Vector3 target_ned = tf::quatRotate(q, target_body);

         // 填充目标数据
         // msg.pose.position.x = target_ned.x();
         // msg.pose.position.y = target_ned.y();
         // msg.pose.position.z = target_ned.z();
         msg.angle[0] = 10;
         msg.angle[1] = 12;
         msg.distance = 2.0;
         msg.pose.position.x = 2.0;
         msg.pose.position.y = -1.0;
         msg.pose.position.z = 0.0;
         msg.size = {0.5, 0.5};  // 降落目标尺寸（米）

         // 发布目标
         target_pub_raw.publish(msg);
     }

    // 主运行逻辑
    void run() {
        // 步骤1：直接切换到精准降落模式
        if (!setPrecisionLandMode()) {
            ROS_ERROR("exiting due to mode switch failure");
            return;
        }

        // 步骤2：循环发布降落目标（20Hz，PX4要求持续发送）
        ros::Rate rate(20);
        ROS_INFO("========== Begin to publish landing target ==========");
        while (ros::ok()) {
            publishLandingTarget();
            publishLandingTarget_raw();
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "precision_landing_odom");
    PrecisionLanding pl;
    pl.run();
    return 0;
}