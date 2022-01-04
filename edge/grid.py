# !usr/bin/env python
# -*- coding:utf-8 -*-

"""
 Description  : 
 Version      : 1.0
 Author       : yurui
 Date         : 2022-01-02 21:52:00
 LastEditors  : yurui
 LastEditTime : 2022-01-04 14:10:31
 FilePath     : \\Lidar\\edge\\grid.py
 Copyright (C) 2022 yurui. All rights reserved.
"""


from matplotlib import pyplot as plt
import cv2
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math


def read_pcd(filepath, dim):
    lidar = []
    with open(filepath, "r") as f:
        line = f.readline().strip()
        while line:
            linestr = line.split(" ")
            if len(linestr) == dim and (not "nan" in linestr):
                linestr_convert = list(map(float, linestr))
                lidar.append(linestr_convert)
            line = f.readline().strip()
    return np.array(lidar)


def mean_and_var_of_grids(num_of_points, points_of_grids, dim=2):
    # 计算每个网格内点的某一维度的均值和方差
    # 没有点的网格其均值方差为0
    mean = np.zeros_like(num_of_points)  # 记录均值
    var = np.zeros_like(num_of_points)  # 记录方差
    grid_size_x, grid_size_y = num_of_points.shape

    for i in range(grid_size_x):
        for j in range(grid_size_y):
            if num_of_points[i, j] != 0:
                # mean[i,j] = np.sum(points_of_grids[i,j,:num_of_points[i,j],dim])/num_of_points[i,j]
                mean[i, j] = np.mean(
                    points_of_grids[i, j, : int(num_of_points[i, j]), dim]
                )
                var[i, j] = np.var(
                    points_of_grids[i, j, : int(num_of_points[i, j]), dim]
                )

    return mean, var


# 绘制3D柱状图
def plot3d(grid_size_x, grid_size_y, num_of_points):
    # 构造需要显示的值
    Y = np.arange(0, grid_size_x, step=1)  # X轴的坐标
    X = np.arange(0, grid_size_y, step=1)  # Y轴的坐标
    xx, yy = np.meshgrid(X, Y)  # 网格化坐标
    X, Y = xx.ravel(), yy.ravel()  # 矩阵扁平化
    bottom = np.zeros_like(Y)  # 设置柱状图的底端位值
    width = height = 1  # 每一个柱子的长和宽
    num_of_points = num_of_points.ravel()

    # 绘图设置
    fig = plt.figure()
    ax = fig.gca(projection="3d")  # 三维坐标轴
    ax.bar3d(X, Y, bottom, width, height, num_of_points, shade=True)
    # 坐标轴设置
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("num_of_points")
    plt.show()


# 绘制灰度图
def graymap(path, **kwargs):
    for k, v in kwargs.items():
        cv2.imshow(k, v)
        cv2.imwrite(path + k + ".png", v)
    cv2.waitKey(-1)
    cv2.destroyAllWindows()


# 方格特征平均化
def average_points(points_of_grids):
    ave_points = np.zeros(
        (points_of_grids.shape[0], points_of_grids.shape[1], points_of_grids.shape[3])
    )

    for i in range(points_of_grids.shape[0]):
        for j in range(points_of_grids.shape[1]):
            for z in range(points_of_grids.shape[3]):
                ave_points[i, j, z] = np.mean(points_of_grids[i, j, :, z])
    return ave_points


# 计算每个方格的坡度
def get_grad_y(points_of_grids):
    # y方向上的坡度
    grad_y = np.zeros((points_of_grids.shape[0], points_of_grids.shape[1]))
    ave_points = average_points(points_of_grids)
    for i in range(ave_points.shape[0]):
        for j in range(ave_points.shape[1]):
            if j != ave_points.shape[1] - 1:
                delta_zi = ave_points[i, j + 1, 2] - ave_points[i, j, 2] + 1e-9
                delta_yi = ave_points[i, j + 1, 1] - ave_points[i, j, 1] + 1e-9
                delta_xi = ave_points[i, j + 1, 0] - ave_points[i, j, 0] + 1e-9
            if j != 0:
                delta_xi_1 = ave_points[i, j, 0] - ave_points[i, j - 1, 0] + 1e-9
                delta_yi_1 = ave_points[i, j, 1] - ave_points[i, j - 1, 1] + 1e-9
                delta_zi_1 = ave_points[i, j, 2] - ave_points[i, j - 1, 2] + 1e-9

            if j == 0:
                grad_y[i, j] = abs(
                    math.atan(delta_zi / math.sqrt(delta_xi ** 2 + delta_yi ** 2))
                )
            elif j == ave_points.shape[1] - 1:
                grad_y[i, j] = abs(
                    math.atan(delta_zi_1 / math.sqrt(delta_xi_1 ** 2 + delta_yi_1 ** 2))
                )
            else:
                grad_y[i, j] = abs(
                    math.atan(delta_zi / math.sqrt(delta_xi ** 2 + delta_yi ** 2))
                    - math.atan(
                        delta_zi_1 / math.sqrt(delta_xi_1 ** 2 + delta_yi_1 ** 2)
                    )
                )
    return grad_y


def get_grad_x(points_of_grids):
    # x方向上的坡度
    grad_x = np.zeros((points_of_grids.shape[0], points_of_grids.shape[1]))
    ave_points = average_points(points_of_grids)
    for i in range(ave_points.shape[0]):
        for j in range(ave_points.shape[1]):
            if i != ave_points.shape[0] - 1:
                delta_zi = ave_points[i + 1, j, 2] - ave_points[i, j, 2] + 1e-9
                delta_yi = ave_points[i + 1, j, 1] - ave_points[i, j, 1] + 1e-9
                delta_xi = ave_points[i + 1, j, 0] - ave_points[i, j, 0] + 1e-9
            if i != 0:
                delta_xi_1 = ave_points[i, j, 0] - ave_points[i - 1, j, 0] + 1e-9
                delta_yi_1 = ave_points[i, j, 1] - ave_points[i - 1, j, 1] + 1e-9
                delta_zi_1 = ave_points[i, j, 2] - ave_points[i - 1, j, 2] + 1e-9

            if i == 0:
                grad_x[i, j] = abs(
                    math.atan(delta_zi / math.sqrt(delta_xi ** 2 + delta_yi ** 2))
                )
            elif i == ave_points.shape[0] - 1:
                grad_x[i, j] = abs(
                    math.atan(delta_zi_1 / math.sqrt(delta_xi_1 ** 2 + delta_yi_1 ** 2))
                )
            else:
                grad_x[i, j] = abs(
                    math.atan(delta_zi / math.sqrt(delta_xi ** 2 + delta_yi ** 2))
                    - math.atan(
                        delta_zi_1 / math.sqrt(delta_xi_1 ** 2 + delta_yi_1 ** 2)
                    )
                )
    return grad_x


if __name__ == "__main__":
    pcd_path = "/home/cyr/Documents/cppProject/LidarProjects/Lidar/edge/plane_cloud.pcd"
    point_data = read_pcd(pcd_path, 4)

    # todo------------------------------将点云网格化-------------------------------------
    xmin, xmax = [0, 60]  # x轴的范围
    ymin, ymax = [-20, 20]  # y轴的范围
    grid_x, grid_y = 0.2, 0.2  # 每个网格的大小
    max_num_of_point = 200  # 每个网格中点的最大数量
    num_of_points_features = 4  # 每个点的特征数

    grid_size_x, grid_size_y = int((xmax - xmin) / grid_x), int(
        (ymax - ymin) / grid_y
    )  # [120,80]

    num_of_points = np.zeros((grid_size_x, grid_size_y))  # [120,80] 用于记录每个网格中点的数量
    points_of_grids = np.zeros(
        (grid_size_x, grid_size_y, max_num_of_point, num_of_points_features)
    )  # 用于记录每个grid中点的坐标

    for point in point_data:
        p_x = int(np.floor((point[0] - xmin) / grid_x))
        p_y = int(np.floor((point[1] - ymin) / grid_y))

        if (p_x < 0 or p_x >= grid_size_x) or (p_y < 0 or p_y >= grid_size_y):
            continue
        else:
            points_of_grids[p_x, p_y, int(num_of_points[p_x, p_y])] = point

            num_of_points[p_x, p_y] += 1

    # todo-----------------------计算每个网格的不同的特征值---------------------------------
    # ?-----待解决：
    # ?----- 1.有点的网格和无点的网格之间的差值如何计算？
    #!--------------------------高程
    height_of_ground = -1.30  # 设定一个地面高度给无点的网格
    num_of_grids_contain_points = 0  # 记录含有点的网格的数量
    height_of_grids = np.zeros((grid_size_x, grid_size_y))
    for i in range(grid_size_x):
        for j in range(grid_size_y):
            num_point_in_grid = int(num_of_points[i, j])  # 该网格内点的数量
            if num_point_in_grid == 0:  # 如果点的数量为0,则跳过
                height_of_grids[i, j] = height_of_ground
                continue
            num_of_grids_contain_points += 1
            height_of_grids[i, j] = (
                np.sum(points_of_grids[i, j, :num_point_in_grid, 2]) / num_point_in_grid
            )  # 单元格内部的平均高度

    # ------------------------计算相邻网格之间的高程差，沿x轴之间的高程差较小，y轴之间的高程差较大的网格
    d_value_of_height_x = np.zeros((grid_size_x, grid_size_y))
    d_value_of_height_y = np.zeros((grid_size_x, grid_size_y))

    for i in range(grid_size_x):
        for j in range(grid_size_y):
            # 两个边界点的高程差值 0和grid_size_y-1
            if j != 0 and j != grid_size_y - 1:
                d_value_of_height_y[i, j] = max(
                    abs(height_of_grids[i, j - 1] - height_of_grids[i, j]),
                    abs(height_of_grids[i, j] - height_of_grids[i, j + 1]),
                )

            elif j == 0:
                d_value_of_height_y[i, j] = abs(
                    height_of_grids[i, j] - height_of_grids[i, j + 1]
                )
            elif j == grid_size_y - 1:
                d_value_of_height_y[i, j] = abs(
                    height_of_grids[i, j - 1] - height_of_grids[i, j]
                )

            else:
                raise KeyError("wrong index!")

    for i in range(grid_size_x):
        for j in range(grid_size_y):
            # 两个边界点的高程差值 0和grid_size_x-1
            if i != 0 and i != grid_size_x - 1:
                d_value_of_height_x[i, j] = max(
                    abs(height_of_grids[i - 1, j] - height_of_grids[i, j]),
                    abs(height_of_grids[i, j] - height_of_grids[i + 1, j]),
                )

            elif i == 0:
                d_value_of_height_x[i, j] = abs(
                    height_of_grids[i, j] - height_of_grids[i + 1, j]
                )
            elif i == grid_size_x - 1:
                d_value_of_height_x[i, j] = abs(
                    height_of_grids[i - 1, j] - height_of_grids[i, j]
                )

            else:
                raise KeyError("wrong index!")
    d_value_of_height = np.sqrt(d_value_of_height_x ** 2 + d_value_of_height_y**2)

    #!--------------------------计算每个网格高度的方差和均值
    mean, var = mean_and_var_of_grids(num_of_points, points_of_grids)
    #!--------------------------坡度
    grad_x = get_grad_x(points_of_grids) #[120,80]
    grad_y = get_grad_y(points_of_grids) #[120,80]
    grad = np.sqrt(grad_x**2 + grad_y**2)
    #!--------------------------光强度的变化
    # y方向上反射率的变化 （变化大）
    delta_intensity_y = np.zeros((grid_size_x, grid_size_y))
    for i in range(grid_size_x):
        for j in range(grid_size_y):
            if j != 0 and j != grid_size_y - 1:
                delta_intensity_y[i, j] = max(
                    abs(
                        np.mean(points_of_grids[i, j, :, -1])
                        - np.mean(points_of_grids[i, j + 1, :, -1])
                    ),
                    abs(
                        np.mean(points_of_grids[i, j, :, -1])
                        - np.mean(points_of_grids[i, j - 1, :, -1])
                    ),
                )
            elif j == 0:
                delta_intensity_y[i, j] = abs(
                    np.mean(points_of_grids[i, j, :, -1])
                    - np.mean(points_of_grids[i, j + 1, :, -1])
                )

            elif j == grid_size_y - 1:
                delta_intensity_y[i, j] = abs(
                    np.mean(points_of_grids[i, j, :, -1])
                    - np.mean(points_of_grids[i, j - 1, :, -1])
                )
            else:
                raise KeyError("wrong index!")
    # x方向上反射率的变化 （变化小 意义不大？）
    delta_intensity_x = np.zeros((grid_size_x, grid_size_y))
    for i in range(grid_size_x):
        for j in range(grid_size_y):
            if i != 0 and i != grid_size_x - 1:
                delta_intensity_x[i, j] = max(
                    abs(
                        np.mean(points_of_grids[i, j, :, -1])
                        - np.mean(points_of_grids[i + 1, j, :, -1])
                    ),
                    abs(
                        np.mean(points_of_grids[i, j, :, -1])
                        - np.mean(points_of_grids[i + 1, j, :, -1])
                    ),
                )
            elif i == 0:
                delta_intensity_x[i, j] = abs(
                    np.mean(points_of_grids[i, j, :, -1])
                    - np.mean(points_of_grids[i + 1, j, :, -1])
                )

            elif i == grid_size_x - 1:
                delta_intensity_x[i, j] = abs(
                    np.mean(points_of_grids[i, j, :, -1])
                    - np.mean(points_of_grids[i - 1, j, :, -1])
                )
            else:
                raise KeyError("wrong index!")
    delta_intensity = np.sqrt(delta_intensity_x**2 + delta_intensity_y**2)
    #!--------------------------点数量的变化

    #!--------------------------与邻近网格连续性

    #!各特征的结合方式！

    # 绘制每个方格的点数3D直方图
    # plot3d(grid_size_x, grid_size_y, num_of_points)
    # todo------------------------------绘制灰度图----------------------------------------
    path = "/home/cyr/Documents/cppProject/LidarProjects/Lidar/edge/graymap/"
    var *= 100
    graymap(
        path,
        num_points=num_of_points,
        delta_height_y=d_value_of_height,
        var100=var,
        delta_intensity = delta_intensity,
        grad = grad
    )
