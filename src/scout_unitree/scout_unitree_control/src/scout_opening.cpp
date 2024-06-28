#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <scout_unitree_control/TrajectoryArray.h>
#include <scout_unitree_control/TrajectoryPoint.h>

// 全局发布器
ros::Publisher cmd_vel_pub;
bool first_send_;
std::vector<geometry_msgs::Pose> trajectory_;
std::vector<geometry_msgs::Twist> velocities_;
std::vector<double> time_stamps_;
ros::Time start_time_;
ros::Time current_time_;

void computeControl()
{

    current_time_ = ros::Time::now();
    if (first_send_ && current_time_.toSec() > 20.0)
    {
        start_time_ = current_time_;
        first_send_ = false;
    }

    double t = (current_time_ - start_time_).toSec();
    if (t > 10.1)
    {
        t = 0.0;
    }

    std::cout << "current_time: " << current_time_.toSec() << std::endl;
    std::cout << "start_time: " << start_time_.toSec() << std::endl;
    std::cout << "t = " << t << std::endl;
    double v_d = 0.0, w_d = 0.0;
    v_d = 0.03 * std::pow(t, 2) - 0.006 * std::pow(t, 3) + 0.0003 * std::pow(t, 4);
    // geometry_msgs::Pose desire_pose;
    // getDesire(t, v_d, w_d, desire_pose);

    geometry_msgs::Twist cmd;
    cmd.linear.x = v_d;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;
    // const auto &goal_pose = trajectory_[current_goal_index_];
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

    // 订阅速度消息话题
    // ros::Subscriber velocity_sub = nh.subscribe("/trajectory", 10, goalCallback);

    // 创建 /cmd_vel 话题的发布器
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    first_send_ = true;
    start_time_ = ros::Time::now();
    while (ros::ok())
    {
        computeControl();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}