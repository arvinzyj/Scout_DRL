# !/usr/bin/env python
# -*-coding:utf-8-*
'''
@File    :   BFS.py
@Time    :   2023/11/16 10:02:21
@Author  :   Arvin 
@Version :   1.0
@Contact :   arvin0211@163.com
@Desc    :   None
'''


import numpy as np
import random


class BFS:
    def __init__(self):
        self.start = [0.0, 0.0]     # 起点
        self.goal = [4.0, 4.0]      # 终点
        self.init_map = np.zeros([5, 5])        # 地图，0表示可以通过，-1表示有障碍
        
        self.actions = [[0, 1], [0, -1], [-1, 0], [1, 0]]

    def get_layer(self):
        self.layer_map = np.full_like(self.init_map, 0)
        layer = 1
        self.layer_map[self.start[0]][self.start[1]] = 1
        self.rows, self.cols = self.init_map.shape
        while True:
            layer += 1
            for i in range(self.rows):
                for j in range(self.cols):
                    if self.layer_map[i][j] == layer - 1:
                        for k in range(4):
                            action = self.actions[k]
                            next_x, next_y = i + action[0], j + action[1]
                            if 0 <= next_x < self.rows and 0 <= next_y < self.cols:
                                if self.init_map[next_x][next_y] == 0 and self.layer_map[next_x][next_y] == 0:
                                    self.layer_map[next_x][next_y] = layer

            if self.layer_map[self.goal[0]][self.goal[1]] != 0:
                break
        
        self.last_layer = layer

        return self.layer_map

    def get_path(self):
        self.path = []
        curr_point = self.goal
        self.path.insert(0, curr_point)
        layer = self.last_layer
        while True:
            k1 = random.randint(0, 3)
            next_x, next_y = curr_point[0] + self.actions[k1][0], curr_point[1] + self.actions[k1][1]
            next_point = [next_x, next_y]
            if 0 <= next_x < self.rows and 0 <= next_y < self.cols:
                while True:
                    k2 = k1
                    next_x, next_y = curr_point[0] + self.actions[k2][0], curr_point[1] + self.actions[k2][1]
                    next_point = [next_x, next_y]
                    if self.layer_map[next_x][next_y] == layer - 1:
                        curr_point = next_point
                        self.path.insert(0, curr_point)
                        layer -= 1
                    else:
                        break
            
            if len(self.path) == self.last_layer:
                break

        
        return self.path

class DFS:
    def __init__(self) -> None:
        self.start = [0.0, 0.0]
        self.goal = [4.0, 4.0]
        self.init_map = np.zeros([5, 5])
        self.actions = [[0, 1], [0, -1], [-1, 0], [1, 0]]

    def get_layer(self):
        pass

    def get_path(self):
        pass

if __name__ == '__main__':
    init_map = np.zeros([10, 10])
    init_map[1][1], init_map[1][2] = -1, -1
    init_map[5][4], init_map[5][5] = -1, -1
    start = [0, 0]
    goal = [9, 9]
    model = BFS()
    model.init_map, model.start, model.goal = init_map, start, goal
    layer_map = model.get_layer()
    path = model.get_path()
    print(layer_map)
    print(path)
