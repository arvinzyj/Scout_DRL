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

typedef Eigen::Vector3d VectorX;
typedef Eigen::Vector2d VectorU;

class ScoutTracking
{
public:
    ScoutTracking(ros::NodeHandle &nh);
    void computeControl();
    bool first_send_;

    VectorX diff(const VectorX &state, const VectorU &input) const;
    void step(VectorX &state, const VectorU &input, const double dt) const;
    void getDesire(double t, double &v_d, double &w_d, geometry_msgs::Pose &desire_pose);

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void goalCallback(const scout_unitree_control::TrajectoryArray::ConstPtr &msg);
    VectorX compensateDelay(const VectorX &x0);
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

    std::deque<VectorU> historyInput_;
    int history_length_;
    double delay_;
    double dt_;
    std::string odom_topic_;
};

inline VectorX ScoutTracking::diff(const VectorX &state, const VectorU &input) const
{
    VectorX ds;
    double phi = state(0);
    double x = state(1);
    double y = state(2);

    double v = input(0);
    double w = input(1);

    ds(0) = w;
    ds(1) = v * cos(phi);
    ds(2) = v * sin(phi);
    return ds;
}

inline void ScoutTracking::step(VectorX &state, const VectorU &input, const double dt) const
{
    // RungeKutta
    VectorX k1 = diff(state, input);
    VectorX k2 = diff(state + k1 * dt / 2, input);
    VectorX k3 = diff(state + k2 * dt / 2, input);
    VectorX k4 = diff(state + k3 * dt, input);
    state = state + (k1 + 2 * k2 + 2 * k3 + k4) * dt / 6;
}

#endif