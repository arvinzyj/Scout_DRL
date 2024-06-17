import numpy as np
import matplotlib.pyplot as plt

def compute_arc_points(center, radius, start_angle, end_angle, num_points=100):
    angles = np.linspace(start_angle, end_angle, num_points)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    return x, y

def plot_path(x1, y1, theta_A, x2, y2, theta_B, R):
    # 计算圆心
    cx1 = x1 + R * np.cos(theta_A + np.pi/2)
    cy1 = y1 + R * np.sin(theta_A + np.pi/2)
    
    cx2 = x2 + R * np.cos(theta_B + np.pi/2)
    cy2 = y2 + R * np.sin(theta_B + np.pi/2)

    # 计算切点
    p1x = cx1 + R * np.cos(theta_A)
    p1y = cy1 + R * np.sin(theta_A)
    
    p2x = cx2 + R * np.cos(theta_B)
    p2y = cy2 + R * np.sin(theta_B)

    # 计算圆弧1
    arc1_x, arc1_y = compute_arc_points((cx1, cy1), R, theta_A - np.pi/2, theta_A, num_points=50)
    
    # 计算直线段
    line_x = np.linspace(p1x, p2x, 100)
    line_y = np.linspace(p1y, p2y, 100)

    # 计算圆弧2
    arc2_x, arc2_y = compute_arc_points((cx2, cy2), R, theta_B - np.pi, theta_B - np.pi/2, num_points=50)

    # 绘制轨迹
    plt.figure(figsize=(10, 10))
    plt.plot(arc1_x, arc1_y, label='Arc 1')
    plt.plot(line_x, line_y, label='Line')
    plt.plot(arc2_x, arc2_y, label='Arc 2')
    plt.scatter([x1, x2], [y1, y2], color='red') # A点和B点
    plt.scatter([p1x, p2x], [p1y, p2y], color='blue') # 切点
    plt.axis('equal')
    plt.legend()
    plt.show()

# 示例参数
x1, y1 = 0, 0
theta_A = np.pi / 4  # 45度
x2, y2 = 10, 10
theta_B = np.pi / 2  # 90度
R = 2

plot_path(x1, y1, theta_A, x2, y2, theta_B, R)
