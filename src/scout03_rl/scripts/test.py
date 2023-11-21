# !/usr/bin/env python
# -*-coding:utf-8-*

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

from numpy import polyfit, poly1d

# 选择'ggplot'样式
plt.style.use("ggplot")

step = np.arange(0, 249565, 355)
reward = np.loadtxt('./saved_rewards/obstacle_world_reward.txt')

# 创建曲线图
plt.plot(step, reward, label="first train")

# 设置标题和坐标轴标签
plt.title("Reward")
plt.xlabel("step")
plt.ylabel("reward")

# 添加图例
plt.legend()

# 保存图片
plt.savefig('./saved_rewards/obstacle_world_reward.png', dpi=600)

# 显示曲线图
plt.show()