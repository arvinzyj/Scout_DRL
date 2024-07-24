#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <Eigen/Cholesky>
#include <Eigen/LU>

typedef Eigen::Matrix<double, 3, 3> Matrix33;
typedef Eigen::Matrix<double, 3, 2> Matrix32;
typedef Eigen::Matrix<double, 2, 3> Matrix23;
typedef Eigen::Matrix<double, 2, 2> Matrix22;

// lqr parameters
Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3);
Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);

Matrix33 sloveDare(const Matrix33 &A, const Matrix32 &B)
{
    // solve a discrete time_Algebratic Riccati Equation
    Matrix33 P = Q;
    Matrix33 P_next = Q;
    int max_iter = 150;
    double eps = 0.01;

    for (int i = 0; i < max_iter; i++)
    {
        P_next = Q + A.transpose() * P * A - A.transpose() * P * B * ((R + B.transpose() * P * B).inverse()) * B.transpose() * P * A;
        if ((P_next - P).norm() < eps)
        {
            P = P_next;
            break;
        }
        P = P_next;
    }

    return P_next;
}

void sloveLqr(const Matrix33 &A, const Matrix32 &B, const Eigen::Vector3d &x_e)
{
    Eigen::Matrix3d P = sloveDare(A, B);

    Matrix23 K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;

    Eigen::Vector2d u_e = -K * x_e;
    Eigen::Vector3d x_next_e = A * x_e + B * u_e;
    return;
}

int main(int argc, char *argv[])
{
    // 设置编码
    setlocale(LC_ALL, "");

    // 2.初始化 ROS 节点:命名(唯一)
    //  参数1和参数2 后期为节点传值会使用
    //  参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc, argv, "talker");
    // 3.实例化 ROS 句柄
    ros::NodeHandle nh; // 该类封装了 ROS 中的一些常用功能

    // 4.实例化 发布者 对象
    // 泛型: 发布的消息类型
    // 参数1: 要发布到的话题
    // 参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);

    // 5.组织被发布的数据，并编写逻辑发布数据
    // 数据(动态组织)
    std_msgs::String msg;
    // msg.data = "你好啊！！！";
    std::string msg_front = "Hello 你好！"; // 消息前缀
    int count = 0;                          // 消息计数器

    // 逻辑(一秒10次)
    ros::Rate r(1);

    // 节点不死
    while (ros::ok())
    {
        // 使用 stringstream 拼接字符串与编号
        std::stringstream ss;
        ss << msg_front << count;
        msg.data = ss.str();
        // 发布消息
        pub.publish(msg);
        // 加入调试，打印发送的消息
        ROS_INFO("发送的消息:%s", msg.data.c_str());

        // 根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        r.sleep();
        count++; // 循环结束前，让 count 自增
        // 暂无应用
        ros::spinOnce();
    }

    return 0;
}