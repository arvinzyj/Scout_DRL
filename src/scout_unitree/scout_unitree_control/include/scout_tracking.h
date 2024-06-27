#ifndef SCOUT_TRACKING_H
#define SCOUT_TRACKING_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <scout_unitree_control/TrajectoryArray.h>
#include <scout_unitree_control/TrajectoryPoint.h>
#include <tf/tf.h>
#include <cmath>
#include <Eigen/Cholesky>

class ScoutTracking
{
public:
    ScoutTracking(ros::NodeHandle &nh);
    void computeControl();
    bool first_send_;

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void goalCallback(const scout_unitree_control::TrajectoryArray::ConstPtr &msg);
    void getDesire(double t, double &v_d, double &w_d, geometry_msgs::Pose &desire_pose);
    Eigen::Vector3d getError(const geometry_msgs::Pose &current_pose, const geometry_msgs::Pose &goal_pose);

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher cmd_pub_;

    geometry_msgs::Pose current_pose_;
    std::vector<geometry_msgs::Pose> trajectory_;
    std::vector<geometry_msgs::Twist> velocities_;
    std::vector<double> time_stamps_;

    double linear_k1_;
    double angular_k2_;
    double angular_k3_;
    int current_goal_index_;
    ros::Time start_time_;
};

#endif