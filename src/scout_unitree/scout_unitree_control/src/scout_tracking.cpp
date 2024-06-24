#include "scout_tracking.h"
#include <tf/tf.h>
#include <cmath>
#include <Eigen/Cholesky>

ScoutTracking::ScoutTracking(ros::NodeHandle &nh) : nh_(nh), linear_k1_(1.0), angular_k2_(4.0), angular_k3_(1.0), current_goal_index_(0)
{
    odom_sub_ = nh_.subscribe("/odom", 10, &ScoutTracking::odomCallback, this);
    goal_sub_ = nh_.subscribe("/trajectory", 10, &ScoutTracking::goalCallback, this); // 订阅轨迹话题
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    nh_.param("linear_k1", linear_k1_, linear_k1_);
    nh_.param("angular_k2", angular_k2_, angular_k2_);
    nh_.param("angular_k3", angular_k3_, angular_k3_);
}

void ScoutTracking::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose_ = msg->pose.pose;
    computeControl();
}

void ScoutTracking::goalCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    trajectory_.clear();
    time_stamps_.clear();
    trajectory_.reserve(msg->poses.size());
    time_stamps_.reserve(msg->poses.size());

    double t = 0.0;
    for (const auto &pose : msg->poses)
    {
        trajectory_.emplace_back(pose);
        time_stamps_.push_back(t);
        t += 1.0;
    }
    current_goal_index_ = 0;
}

void ScoutTracking::getDesire(double t, double &v_d, double &w_d)
{
    if (trajectory_.empty())
        return;

    size_t idx = 0;
    while (idx < time_stamps_.size() && t > time_stamps_[idx])
    {
        idx++;
    }

    if (idx == 0 || idx >= trajectory_.size())
    {
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

    const auto &prev_pose = trajectory_[idx - 1];
    const auto &next_pose = trajectory_[idx];
    double prev_t = time_stamps_[idx - 1];
    double next_t = time_stamps_[idx];

    double dt = next_t - prev_t;
    double dx = next_pose.position.x - prev_pose.position.x;
    double dy = next_pose.position.y - prev_pose.position.y;
    double dphi = tf::getYaw(next_pose.orientation) - tf::getYaw(prev_pose.orientation);

    v_d = std::sqrt(dx * dx + dy * dy) / dt;
    w_d = dphi / dt;
}

Eigen::Vector3d ScoutTracking::getError(const geometry_msgs::Pose &current_pose, const geometry_msgs::Pose &desire_pose)
{
    double x_c = current_pose.position.x;
    double y_c = current_pose.position.y;
    double phi_c = tf::getYaw(current_pose.orientation);

    double x_d = desire_pose.position.x;
    double y_d = desire_pose.position.y;
    double phi_d = tf::getYaw(desire_pose.orientation);

    // get error
    double phi_e = phi_c - phi_d;
    double x_e = (x_c - x_d) * cos(phi_d) + (y_c - y_d) * sin(phi_d);
    double y_e = -(x_c - x_d) * sin(phi_d) + (y_c - y_d) * cos(phi_d);

    Eigen::Vector3d q_e = {phi_e, x_e, y_e};

    return q_e;
}

void ScoutTracking::computeControl()
{
    if (trajectory_.empty() || current_goal_index_ >= trajectory_.size())
        return;

    geometry_msgs::Twist cmd;
    const auto &goal_pose = trajectory_[current_goal_index_];
    Eigen::Vector3d q_e = getError(current_pose_, goal_pose);

    double v_d = 0.0, w_d = 0.0;
    getDesire(ros::Time::now().toSec(), v_d, w_d);

    cmd.linear.x = (v_d - linear_k1_ * abs(v_d) * (q_e(1)) + (q_e(2) * tan(q_e(0)))) / cos(q_e(0));
    cmd.angular.z = w_d - (angular_k2_ * v_d * q_e(2) + angular_k3_ + abs(v_d) * tan(q_e(0))) * cos(q_e(0) * cos(q_e(0)));

    double distance = std::sqrt(q_e(1) * q_e(1) + q_e(2) * q_e(2));
    if (distance < 0.1)
    {
        current_goal_index_++;
    }

    cmd_pub_.publish(cmd);
}