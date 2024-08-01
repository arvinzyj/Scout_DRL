#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

ros::Publisher odom_pub;
tf::Transform initial_transform;
bool initial_transform_set = false;

void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    if (msg->detections.empty())
    {
        ROS_WARN("No detections in the current message.");
        return;
    }

    // 假设我们只处理第一个检测到的AprilTag
    const auto &detection = msg->detections[0];
    const auto &cam_pose = detection.pose.pose.pose;

    // 创建一个里程计消息
    nav_msgs::Odometry odom;

    // 设置里程计消息的header
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "tag_odom";

    // 将相机到tag的位姿转换为tf::Transform
    tf::Transform tag_to_cam;
    tf::poseMsgToTF(cam_pose, tag_to_cam);

    tf::Transform cam_to_car;
    cam_to_car.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    cam_to_car.setRotation(tf::Quaternion(0.5, -0.5, 0.5, -0.5));

    tf::Transform tag_to_car;
    tag_to_car = cam_to_car * tag_to_cam;

    if (!initial_transform_set)
    {
        // 在第一帧时记录下相机到tag的变换矩阵
        initial_transform = tag_to_car;
        initial_transform_set = true;
    }

    // 计算基座到tag的变换
    tf::Transform base_to_tag = initial_transform * tag_to_car.inverse();

    // 将基座到tag的变换转换为里程计消息中的位姿
    tf::poseTFToMsg(base_to_tag, odom.pose.pose);

    // 设置里程计的其他信息
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    // 发布里程计消息
    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_to_odom");
    ros::NodeHandle nh;

    // 订阅/tag_detections话题
    ros::Subscriber tag_sub = nh.subscribe("/tag_detections", 10, tagDetectionsCallback);

    // 发布/odom话题
    odom_pub = nh.advertise<nav_msgs::Odometry>("/tag_odom", 10);

    ros::spin();
    return 0;
}
