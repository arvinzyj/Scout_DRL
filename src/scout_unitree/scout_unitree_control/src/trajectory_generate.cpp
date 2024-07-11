#include <ros/ros.h>
#include <scout_unitree_control/TrajectoryPoint.h>
#include <scout_unitree_control/TrajectoryArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh;

    ros::Publisher traj_pub = nh.advertise<scout_unitree_control::TrajectoryArray>("/trajectory", 10);

    ros::Rate rate(1); // 1 Hz

    while (ros::ok())
    {
        scout_unitree_control::TrajectoryArray traj_points;

        // straight
        // for (int i = 0; i < 1001; ++i)
        // {
        //     scout_unitree_control::TrajectoryPoint traj_point;

        //     double t = i / 100.0;

        //     // Fill in the position
        //     traj_point.pose.position.x = 0.01 * std::pow(t, 3) - 0.0015 * std::pow(t, 4) + 0.00006 * std::pow(t, 5);
        //     traj_point.pose.position.y = 0.0;
        //     traj_point.pose.position.z = 0.0;
        //     traj_point.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

        //     // Fill in the velocity
        //     traj_point.velocity.linear.x = 0.03 * std::pow(t, 2) - 0.006 * std::pow(t, 3) + 0.0003 * std::pow(t, 4);
        //     traj_point.velocity.linear.y = 0.0;
        //     traj_point.velocity.linear.z = 0.0;
        //     traj_point.velocity.angular.x = 0.0;
        //     traj_point.velocity.angular.y = 0.0;
        //     traj_point.velocity.angular.z = 0.0;

        //     traj_points.points.push_back(traj_point);
        // }

        // circle
        // for (int i = 0; i < 1001; ++i)
        // {
        //     scout_unitree_control::TrajectoryPoint traj_point;

        //     double t = i / 100.0;
        //     double radius = 5.0;

        //     // Fill in the position
        //     traj_point.pose.position.x = radius * std::sin(0.2 * M_PI * t);
        //     traj_point.pose.position.y = -radius * std::cos(0.2 * M_PI * t) + radius;
        //     traj_point.pose.position.z = 0.0;
        //     traj_point.pose.orientation = tf::createQuaternionMsgFromYaw(0.2 * M_PI * t);

        //     // Fill in the velocity
        //     traj_point.velocity.linear.x = 0.2 * M_PI * radius;
        //     traj_point.velocity.linear.y = 0.0;
        //     traj_point.velocity.linear.z = 0.0;
        //     traj_point.velocity.angular.x = 0.0;
        //     traj_point.velocity.angular.y = 0.0;
        //     traj_point.velocity.angular.z = 0.2 * M_PI;

        //     traj_points.points.push_back(traj_point);
        // }

        // curve
        for (int i = 0; i < 1001; ++i)
        {
            scout_unitree_control::TrajectoryPoint traj_point;

            double t = i / 100.0;

            double x = 0.05 * std::pow(t, 3) - 0.0075 * std::pow(t, 4) + 0.0003 * std::pow(t, 5);
            double dx = 0.15 * std::pow(t, 2) - 0.03 * std::pow(t, 3) + 0.0015 * std::pow(t, 4);
            double ddx = 0.3 * t - 0.09 * std::pow(t, 2) + 0.006 * std::pow(t, 3);
            double y = 0.4 * std::pow(x, 3) - 0.12 * std::pow(x, 4) + 0.0096 * std::pow(x, 5);
            double dy_dx = 1.2 * std::pow(x, 2) - 0.48 * std::pow(x, 3) + 0.048 * std::pow(x, 4);
            double ddy_dx = 2.4 * x - 1.44 * std::pow(x, 2) + 0.192 * std::pow(x, 3);
            double dy = dy_dx * dx;
            double ddy = ddy_dx * dx * dx + dy_dx * ddx;
            double yaw = std::atan(dy_dx);

            // Fill in the position
            traj_point.pose.position.x = x;
            traj_point.pose.position.y = y;
            traj_point.pose.position.z = 0.0;
            traj_point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

            // Fill in the velocity
            traj_point.velocity.linear.x = std::sqrt(dx * dx + dy * dy);
            traj_point.velocity.linear.y = 0.0;
            traj_point.velocity.linear.z = 0.0;
            traj_point.velocity.angular.x = 0.0;
            traj_point.velocity.angular.y = 0.0;
            traj_point.velocity.angular.z = (dx * ddy - dy * ddx) / (dx * dx + dy * dy);

            traj_points.points.push_back(traj_point);
        }

        traj_pub.publish(traj_points);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}