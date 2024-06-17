#!/usr/bin/env python
import sys
sys.path.append("../z1_sdk/lib")
import unitree_arm_interface
import time
import numpy as np

import rospy
from geometry_msgs.msg import Twist



def scout_z1_run():
    #读取轨迹
    traj = np.loadtxt("../data/cp_2024-06-16-19-29-13.txt")
    traj_n = len(traj)
    traj_scout = traj[:,0:2]
    traj_z1 = traj[:,2:8]

    #初始化z1信息
    np.set_printoptions(precision=3, suppress=True)
    arm = unitree_arm_interface.ArmInterface(hasGripper=True)
    armModel = arm._ctrlComp.armModel
    arm.setFsmLowcmd()

    #初始化scout信息
    rospy.init_node('accelerate_robot_node', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)#控制频率
    twist = Twist()

    #使机器人到达第一个点
    lastPos = arm.lowstate.getQ()
    firstPos = traj[0,:]
    duration = 300
    for i in range(0, duration):
        arm.q = lastPos*(1-i/duration) + firstPos*(i/duration)# set position
        arm.qd = (firstPos-lastPos)/(duration*0.01) # set velocity
        arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
        arm.setArmCmd(arm.q, arm.qd, arm.tau)
        arm.sendRecv()# udp connection
        # print(arm.lowstate.getQ())
        time.sleep(0.01)

    count = 0
    while not rospy.is_shutdown():
        count += 1
        if count >= traj_n:
            break
        else:
            #编辑z1运动数据
            arm.q = traj[count,:]
            arm.qd = np.array([0.001,0.001,0.001,0.001,0.001,0.001])# set velocity
            arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque

            #编辑scout速度信息
            twist.linear.x = traj[count,0]
            twist.angular.z = traj[count,1]
            #发布scout话题
            cmd_vel_pub.publish(twist)

            #发送z1关节信息
            arm.setArmCmd(arm.q, arm.qd, arm.tau)
            arm.sendRecv()# udp connection

            rate.sleep()

    arm.loopOn()
    arm.loopOff()

    

if __name__ == '__main__':
    try:
        scout_z1_run()
    except rospy.ROSInterruptException:
        pass