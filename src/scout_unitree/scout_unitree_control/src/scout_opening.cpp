#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <scout_unitree_control/TrajectoryArray.h>
#include <scout_unitree_control/TrajectoryPoint.h>
#include "scout_tracking.h"

// 全局发布器
ros::Publisher cmd_vel_pub;
bool first_send_;
ros::Time start_time_;
ros::Time current_time_;
std::string trajectory_type_;

void computeControl()
{

    current_time_ = ros::Time::now();
    if (first_send_)
    {
        start_time_ = current_time_;
        first_send_ = false;
    }

    double t = (current_time_ - start_time_).toSec();

    std::cout << "current_time: " << current_time_.toSec() << std::endl;
    std::cout << "start_time: " << start_time_.toSec() << std::endl;
    std::cout << "t = " << t << std::endl;
    double v_d = 0.0, w_d = 0.0;

    if (trajectory_type_ == "straight")
    {
        v_d = 0.15 * std::pow(t, 2) - 0.03 * std::pow(t, 3) + 0.0015 * std::pow(t, 4);
    }
    else if (trajectory_type_ == "circle")
    {

        // Fill in the velocity
        v_d = 0.2 * M_PI * 1.0;
        w_d = 0.2 * M_PI;
    }
    else if (trajectory_type_ == "curve")
    {
        double x = 0.05 * std::pow(t, 3) - 0.0075 * std::pow(t, 4) + 0.0003 * std::pow(t, 5);
        double dx = 0.15 * std::pow(t, 2) - 0.03 * std::pow(t, 3) + 0.0015 * std::pow(t, 4);
        double ddx = 0.3 * t - 0.09 * std::pow(t, 2) + 0.006 * std::pow(t, 3);
        double y = 0.4 * std::pow(x, 3) - 0.12 * std::pow(x, 4) + 0.0096 * std::pow(x, 5);
        double dy_dx = 1.2 * std::pow(x, 2) - 0.48 * std::pow(x, 3) + 0.048 * std::pow(x, 4);
        double ddy_dx = 2.4 * x - 1.44 * std::pow(x, 2) + 0.192 * std::pow(x, 3);
        double dy = dy_dx * dx;
        double ddy = ddy_dx * dx * dx + dy_dx * ddx;
        double yaw = std::atan(dy_dx);

        // Fill in the velocity
        v_d = std::sqrt(dx * dx + dy * dy);
        w_d = (dx * ddy - dy * ddx) / (dx * dx + dy * dy);
    }
    else
    {
        ROS_WARN("Unknown trajectory type: %s", trajectory_type_.c_str());
    }

    if (t > 12.0)
    {
        v_d = 0.0;
        w_d = 0.0;
    }
    geometry_msgs::Twist cmd;
    cmd.linear.x = v_d;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = w_d;
    std::cout << "v = " << cmd.linear.x << std::endl;
    std::cout << "w = " << cmd.angular.z << std::endl;

    cmd_vel_pub.publish(cmd);
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "scout_opening");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    // 创建 /cmd_vel 话题的发布器
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    nh.param<std::string>("/trajectory_generate/trajectory_type", trajectory_type_, "straight");

    first_send_ = true;

    while (ros::Time::now().toSec() == 0)
    {
        ROS_INFO("Waiting for ROS time to be initialized...");
        ros::Duration(0.1).sleep();
    }

    start_time_ = ros::Time::now();
    std::cout << "start_time: " << start_time_.toSec() << std::endl;
    while (ros::ok())
    {
        computeControl();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}