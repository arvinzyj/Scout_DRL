#include <ros/ros.h>
#include <scout_unitree_control/TrajectoryPoint.h>
#include <scout_unitree_control/TrajectoryArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "trajectory_publisher");
//     ros::NodeHandle nh;

//     ros::Publisher traj_pub = nh.advertise<geometry_msgs::PoseArray>("/trajectory", 10);

//     ros::Rate rate(1); // 1 Hz

//     while (ros::ok())
//     {
//         geometry_msgs::PoseArray pose_array;
//         pose_array.header.stamp = ros::Time::now();
//         pose_array.header.frame_id = "map";

//         // Create a simple trajectory: a straight line with constant orientation
//         for (int i = 0; i < 1001; ++i)
//         {
//             double t = i / 1000.0;
//             geometry_msgs::Pose pose;
//             pose.position.x = 0.01 * std::pow(t, 3) - 0.0015 * std::pow(t, 4) + 0.00006 * std::pow(t, 5);
//             pose.position.y = 0.0;
//             pose.position.z = 0.0;

//             // Set the orientation to be constant (facing along the x-axis)
//             pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

//             pose_array.poses.push_back(pose);
//         }

//         traj_pub.publish(pose_array);

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh;

    ros::Publisher traj_pub = nh.advertise<scout_unitree_control::TrajectoryArray>("/trajectory", 10);

    ros::Rate rate(1); // 1 Hz

    while (ros::ok())
    {
        scout_unitree_control::TrajectoryArray traj_points;

        for (int i = 0; i < 1001; ++i)
        {
            scout_unitree_control::TrajectoryPoint traj_point;

            double t = i / 100.0;

            // Fill in the position
            traj_point.pose.position.x = 0.01 * std::pow(t, 3) - 0.0015 * std::pow(t, 4) + 0.00006 * std::pow(t, 5);
            traj_point.pose.position.y = 0.0;
            traj_point.pose.position.z = 0.0;
            traj_point.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

            // Fill in the velocity
            traj_point.velocity.linear.x = 0.03 * std::pow(t, 2) - 0.006 * std::pow(t, 3) + 0.0003 * std::pow(t, 4);
            traj_point.velocity.linear.y = 0.0;
            traj_point.velocity.linear.z = 0.0;
            traj_point.velocity.angular.x = 0.0;
            traj_point.velocity.angular.y = 0.0;
            traj_point.velocity.angular.z = 0.0;

            traj_points.points.push_back(traj_point);
        }
        std::cout << "length = " << traj_points.points.size() << std::endl;
        traj_pub.publish(traj_points);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}