问题3：
Cameras are not connected through mutual observations, please check the dataset. Maybe adjust the approx. sync. tolerance.
无法输出标定结果。
解决：
https://blog.csdn.net/xiaoxiaoyikesu/article/details/105646064
命令：rosrun kalibr kalibr_calibrate_cameras --target checkerboard.yaml --bag mult_cam_d435i_01.bag --models pinhole-radtan pinhole-radtan pinhole-radtan --topics /color /infra_left /infra_right --bag-from-to 10 100 --show-extraction --approx-sync 0.04

问题4：
/camera/imu 话题没有数据
解决：
可能是usb数据线的问题，没电的时候供电不足会出现这种情况。不清楚原因，第二天过来就好了。
https://blog.csdn.net/qq_35616298/article/details/116197526

多目相机标定配置：
修改前：
```shell
  <arg name="infra_width"         default="848"/>
  <arg name="infra_height"        default="480"/>
  <arg name="enable_infra"        default="false"/>
  <arg name="enable_infra1"       default="false"/>
  <arg name="enable_infra2"       default="false"/>
  <arg name="infra_rgb"           default="false"/>
```
修改后：
```shell
  <arg name="infra_width"         default="848"/>
  <arg name="infra_height"        default="480"/>
  <arg name="enable_infra"        default="true"/>
  <arg name="enable_infra1"       default="true"/>
  <arg name="enable_infra2"       default="true"/>
  <arg name="infra_rgb"           default="false"/>
```



imu_cam标定
rosrun kalibr kalibr_calibrate_imu_camera --target checkerboard.yaml --cam camd435i_02-camchain.yaml --imu imu.yaml --bag camer_imu_02.bag --timeoffset-padding 0.1 

cam_odom标定：
1.采集数据
是把标定板固定，然后在控制地板运动的同时采集相机和odom的数据，在这个过程中需要保持相机一直能够看到标定板。是的，尽可能看到标定板，不用一直保证看到。因为标定一开始会筛选数据，只处理能看到标定板的图像和相对应的odo数据。建议做一下时间同步，如果没有，那么尽量减少复杂运动，每次跑一个半圆，多跑几次即可

参考连接：
https://blog.csdn.net/G_C_H/article/details/136907206#t6
https://github.com/IntelRealSense/realsense-ros/issues/3026
https://github.com/IntelRealSense/realsense-ros/issues/2326#issuecomment-1107658481
https://blog.csdn.net/xiaoxiaoyikesu/article/details/105646064
