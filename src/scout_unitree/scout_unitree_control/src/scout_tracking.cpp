#include "scout_tracking.h"

ScoutTracking::ScoutTracking(ros::NodeHandle &nh) : nh_(nh), linear_k1_(1.0), angular_k2_(1.0), angular_k3_(1.0), current_goal_index_(0), delay_(0.1), dt_(0.1)
{
    odom_sub_ = nh_.subscribe("/odom", 10, &ScoutTracking::odomCallback, this);
    // odom_sub_ = nh_.subscribe("/aft_mapped_to_init", 10, &ScoutTracking::odomCallback, this);
    goal_sub_ = nh_.subscribe("/trajectory", 10, &ScoutTracking::goalCallback, this); // 订阅轨迹话题
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    nh_.param("linear_k1", linear_k1_);
    nh_.param("angular_k2", angular_k2_);
    nh_.param("angular_k3", angular_k3_);
    nh_.param("delay_", delay_);
    nh_.param("dt_", dt_);

    history_length_ = std::ceil(delay_ / dt_);
    for (int i = 0; i < history_length_; ++i)
    {
        historyInput_.emplace_back(0, 0);
    }

    start_time_ = ros::Time::now();
}

void ScoutTracking::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose_ = msg->pose.pose;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(current_pose_.orientation, quat);
    quat.normalize();
    tf::quaternionTFToMsg(quat, current_pose_.orientation);
}

void ScoutTracking::goalCallback(const scout_unitree_control::TrajectoryArray::ConstPtr &msg)
{
    trajectory_.clear();
    velocities_.clear();
    time_stamps_.clear();

    double t = 0.0; // 假设时间从0开始
    for (const auto &point : msg->points)
    {
        trajectory_.push_back(point.pose);
        velocities_.push_back(point.velocity);
        time_stamps_.push_back(t); // 增加时间戳（假设固定间隔，这里每次加1秒，你可以根据实际情况调整）
        t += 0.01;
    }
    std::cout << "Received trajectory with " << trajectory_.size() << " points." << std::endl;
    current_goal_index_ = 0;
}

void ScoutTracking::getDesire(double t, double &v_d, double &w_d, geometry_msgs::Pose &desire_pose)
{
    if (trajectory_.empty())
        return;

    // 查找当前时间 t 对应的轨迹点索引
    size_t idx = 0;
    while (idx < time_stamps_.size() && t > time_stamps_[idx])
    {
        idx++;
    }

    // 如果 t 超出范围，返回最后一个点的速度
    if (idx == 0 || idx >= velocities_.size())
    {
        v_d = 0.0;
        w_d = 0.0;
        return;
    }

    v_d = velocities_[idx].linear.x;
    w_d = velocities_[idx].angular.z;
    desire_pose = trajectory_[idx];
}

VectorX ScoutTracking::compensateDelay(const VectorX &x0)
{
    VectorX x0_delay = x0;
    for (double t = delay_; t > 0; t -= dt_)
    {
        int i = std::ceil(t / dt_);
        VectorU input = historyInput_[history_length_ - i];
        step(x0_delay, input, dt_);
    }

    return x0_delay;
}

Eigen::Vector3d ScoutTracking::getError(const geometry_msgs::Pose &current_pose, const geometry_msgs::Pose &desire_pose)
{
    double phi_c = tf::getYaw(current_pose.orientation);
    double x_c = current_pose.position.x;
    double y_c = current_pose.position.y;
    VectorX x0;
    x0 << phi_c, x_c, y_c;
    VectorX x0_delay = this->compensateDelay(x0);

    double phi_d = tf::getYaw(desire_pose.orientation);
    double x_d = desire_pose.position.x;
    double y_d = desire_pose.position.y;

    // get error
    // double phi_e = phi_c - phi_d;
    // double x_e = (x_c - x_d) * cos(phi_d) + (y_c - y_d) * sin(phi_d);
    // double y_e = -(x_c - x_d) * sin(phi_d) + (y_c - y_d) * cos(phi_d);
    double phi_e = x0_delay(0) - phi_d;
    double x_e = (x0_delay(1) - x_d) * cos(phi_d) + (x0_delay(2) - y_d) * sin(phi_d);
    double y_e = -(x0_delay(1) - x_d) * sin(phi_d) + (x0_delay(2) - y_d) * cos(phi_d);

    Eigen::Vector3d q_e = {phi_e, x_e, y_e};

    return q_e;
}

void ScoutTracking::computeControl()
{
    if (trajectory_.empty() || current_goal_index_ >= trajectory_.size())
        return;

    ros::Time current_time = ros::Time::now();

    std::cout << first_send_ << std::endl;
    if (first_send_)
    {
        start_time_ = current_time;
        first_send_ = false;
    }

    std::cout << "current_time: " << current_time << std::endl;
    std::cout << "start_time: " << start_time_ << std::endl;
    double t = (current_time - start_time_).toSec();
    std::cout << "t = " << t << std::endl;
    double v_d = 0.0, w_d = 0.0;
    geometry_msgs::Pose desire_pose;
    desire_pose.position.x = 0.0;
    desire_pose.position.y = 0.0;
    desire_pose.position.z = 0.0;
    desire_pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    getDesire(t, v_d, w_d, desire_pose);

    geometry_msgs::Twist cmd;
    Eigen::Vector3d q_error = this->getError(current_pose_, desire_pose);

    cmd.linear.x = (v_d - linear_k1_ * abs(v_d) * (q_error(1)) + (q_error(2) * tan(q_error(0)))) / cos(q_error(0));
    cmd.angular.z = w_d - (angular_k2_ * v_d * q_error(2) + angular_k3_ * abs(v_d) * tan(q_error(0))) * cos(q_error(0) * cos(q_error(0)));
    VectorU predicInput = {cmd.linear.x, cmd.angular.z};
    historyInput_.pop_front();
    historyInput_.push_back(predicInput);
    std::cout << "v = " << cmd.linear.x << std::endl;
    std::cout << "w = " << cmd.angular.z << std::endl;

    double distance = std::sqrt(q_error(1) * q_error(1) + q_error(2) * q_error(2));
    if (distance < 0.1)
    {
        current_goal_index_++;
    }

    cmd_pub_.publish(cmd);
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "scout_tracking");
    ros::NodeHandle nh;

    ScoutTracking tracker(nh);
    ros::Rate rate(10); // 10 Hz

    tracker.first_send_ = true;
    while (ros::ok())
    {
        tracker.computeControl();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}