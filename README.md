<<<<<<< HEAD
# DRL_Path_Planning

This is a DRL(Deep Reinforcement Learning) platform built with Gazebo for the purpose of robot's adaptive path planning.

# Environment

## Software

    Ubuntu 16.04
    ROS Kinect
    Python 2.7.12
    tensorflow 1.12.0

# Document

```
src
   ├─ CMakeLists.txt
   ├─ multi_jackal	// 创建模型的功能包
   └─ tf_pkg	// 关于强化学习路径规划的功能包
      ├─ CMakeLists.txt
      ├─ package.xml
      └─ scripts
         ├─ 10_D3QN_PER_image_add_sensor_empty_world_30m.py	// 无障碍环境训练
         ├─ 10_D3QN_PER_image_add_sensor_empty_world_30m_test.py
         ├─ 10_D3QN_PER_image_add_sensor_obstacle_world_30m.py	// 静态障碍环境训练
         ├─ 10_D3QN_PER_image_add_sensor_obstacle_world_30m_test.py
         ├─ D3QN_PER_image_add_sensor_dynamic_10obstacle_world_30m_test.py	//动态障碍环境训练
         ├─ Models.py
         ├─ gazebo_env_D3QN_PER_image_add_sensor_empty_world_30m.py	// 无障碍环境搭建
         ├─ gazebo_env_D3QN_PER_image_add_sensor_empty_world_30m_test.py
         ├─ gazebo_env_D3QN_PER_image_add_sensor_obstacle_world_30m.py	// 静态障碍环境搭建
         ├─ gazebo_env_D3QN_PER_image_add_sensor_obstacle_world_30m_test.py
         ├─ gazebo_env_dynamic_obstacle_10jackal_test.py	// 动态环境搭建
```

# Installation

* 创建一个工作空间并命名为DRL_Path_Planning

  `makedir DRL_Path_Planning`

* git clone https://github.com/CoderWangcai/DRL_Path_Planning.git

* 编译

  `catkin_make`

每个python文件中都会有注释对应的模型文件和环境文件，上文对python文件功能已经作了阐述。以无障碍环境的训练为例：
打开`10_D3QN_PER_image_add_sensor_empty_world_30m.py`文件，其对应的文件如下：

```python
# 环境模型：gazebo_env_D3QN_PER_image_add_sensor_empty_world_30m.py
# launch文件：one_jackal_image_add_sensor.launch
# world文件：empty_sensor.world
```

下面是具体步骤：

* `source devel/setup.bash`
* `roslaunch multi_jackal_tutorials one_jackal_image_add_sensor.launch ` # 启动launch文件

​		需要注意的是启动前需要将l`launch`中的`world`标签更改为对应的`world`文件

* 另起一个命令行，执行`python 10_D3QN_PER_image_add_sensor_empty_world_30m.py `

# Error

## catkin_make时报错

**描述：**在编译时报错如下：

```shell
Could not find a package configuration file provided by
  "robot_localization" with any of the following names:

    robot_localizationConfig.cmake
    robot_localization-config.cmake
```

**原因**：缺少这个包

**解决**：安装就行了

```shell
sudo apt-get install ros-melodic-robot-localization
```

诸如此类的报错，解决方法都一样安装相关包就行了。

## 加载模型时报错

**描述**：加载训练好的模型时报错：

```shell
DataLossError (see above for traceback): Checksum does not match: stored 1214729159 vs. calculated on the restored bytes 4272926173
```

**原因**：

* 模型文件损坏
* tensorflow版本问题
* 储存介质出现问题

**解决**：

重新训练

## 报错

**描述**：训练时报错

```shell
ValueError: cannot copy sequence with size 724 to array axis with dimension 364
```

**原因**：

维度不匹配

**解决**：

寻找到sick_lms1xx.urdf.xacro文件，在里面将720改为360

文件路径为：`/opt/ros/melodic/share/lms1xx/urdf/sick_lms1xx.urdf.xacro`

命令行输入：

```shell
sudo gedit sick_lms1xx.urdf.xacro
```

## 训练时报错

**描述**：

```shell
UnknownError (see above for traceback): Failed to get convolution algorithm. This is probably because cuDNN failed to initialize, so try looking to see if a warning log message was printed above.
```

**原因**：
显存不足，将显存按需分配。

**解决**：

在开头添加这么一段

```python
import keras  
from keras.backend.tensorflow_backend import set_session
config = tf.ConfigProto()  
config.gpu_options.allow_growth = True 
sess = tf.Session(config=config)
set_session(sess)
keras.backend.clear_session()
```



>  ROS常用API官方链接：
>
> http://wiki.ros.org/APIs
=======
# Scout_DRL
>>>>>>> afc2d15c635bce5056307c29de11baae7b7111c6
