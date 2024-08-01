#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

class DynamicTransformListener
{
public:
    DynamicTransformListener()
        : rate_(10.0) // 初始化ros::Rate
    {
        // 初始化ROS节点句柄
        nh_ = ros::NodeHandle();

        // 初始化tf2缓存区和监听器
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        // 发布里程计消息的ROS发布者
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/tag_odom", 10);

        listenAndPublish();
    }

private:
    ros::NodeHandle nh_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    ros::Publisher odom_pub_;
    geometry_msgs::Transform initial_transform_;
    ros::Rate rate_;
    bool initial_transform_set_ = false;

    void listenAndPublish()
    {
        while (ros::ok())
        {
            try
            {
                // 监听"tag0"和"camera_link"之间的变换
                geometry_msgs::TransformStamped trans = tf2_buffer_->lookupTransform("tag_0", "camera_link", ros::Time(0));

                if (!initial_transform_set_)
                {
                    initial_transform_ = trans.transform;
                    publishOdom(initial_transform_, true);
                    initial_transform_set_ = true;
                }
                else
                {
                    geometry_msgs::Transform relative_transform = getRelativeTransform(trans.transform, initial_transform_);
                    publishOdom(relative_transform, false);
                }
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

            rate_.sleep();
        }
    }

    geometry_msgs::Transform getRelativeTransform(const geometry_msgs::Transform &current, const geometry_msgs::Transform &initial)
    {
        geometry_msgs::Transform relative_transform;

        relative_transform.translation.x = current.translation.x - initial.translation.x;
        relative_transform.translation.y = current.translation.y - initial.translation.y;
        relative_transform.translation.z = current.translation.z - initial.translation.z;

        relative_transform.rotation = current.rotation; // 简化处理，假设旋转是相对的

        return relative_transform;
    }

    void publishOdom(const geometry_msgs::Transform &transform, bool is_initial)
    {
        nav_msgs::Odometry odom;

        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "tag_0";
        odom.child_frame_id = "camera_link";

        odom.pose.pose.position.x = transform.translation.x;
        odom.pose.pose.position.y = transform.translation.y;
        odom.pose.pose.position.z = transform.translation.z;

        odom.pose.pose.orientation = transform.rotation;

        odom_pub_.publish(odom);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_transform_listener");

    DynamicTransformListener dtl;

    return 0;
}
