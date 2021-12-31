'''
Description: 
Version: 2.0
Autor: Anyubin
Date: 2021-12-30 13:25:35
LastEditors: Anyubin
LastEditTime: 2021-12-31 10:02:14
'''
import numpy as np
from matplotlib import pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
import cv2

def read_pcd(filepath,dim):
    lidar = []
    with open(filepath,'r') as f:
        line = f.readline().strip()
        while line:
            linestr = line.split(" ")
            if len(linestr) == dim and (not 'nan' in linestr):
                linestr_convert = list(map(float, linestr))
                lidar.append(linestr_convert)
            line = f.readline().strip()
    return np.array(lidar)


def mean_and_var_of_grids(num_of_points,points_of_grids,dim=2):
    #计算每个网格内点的某一维度的均值和方差
    #没有点的网格其均值方差为0
    mean = np.zeros_like(num_of_points) #记录均值
    var = np.zeros_like(num_of_points) #记录方差
    grid_size_x,grid_size_y = num_of_points.shape

    for i in range(grid_size_x):
        for j in range(grid_size_y):
            if(num_of_points[i,j]!=0):
                # mean[i,j] = np.sum(points_of_grids[i,j,:num_of_points[i,j],dim])/num_of_points[i,j]
                mean[i,j] = np.mean(points_of_grids[i,j,:int(num_of_points[i,j]),dim])
                var[i,j] = np.var(points_of_grids[i,j,:int(num_of_points[i,j]),dim])


    return mean,var








if __name__ == '__main__':
    
        
    pcd_path = './plane_cloud.pcd'

    point_data = read_pcd(pcd_path,4)


    #todo------------------------------将点云网格化-------------------------------------
    xmin,xmax = [0,60]   #x轴的范围 
    ymin,ymax = [-20,20] #y轴的范围
    grid_x,grid_y = 0.5,0.5 #每个网格的大小
    max_num_of_point = 200 #每个网格中点的最大数量
    num_of_points_features = 4 #每个点的特征数

    grid_size_x,grid_size_y = int((xmax-xmin)/grid_x),int((ymax-ymin)/grid_y) #[120,80]

    num_of_points = np.zeros((grid_size_x,grid_size_y)) #[120,80] 用于记录每个网格中点的数量
    points_of_grids = np.zeros((grid_size_x,grid_size_y,max_num_of_point,num_of_points_features)) #用于记录每个grid中点的坐标


    for point in point_data:
        p_x = int(np.floor((point[0]-xmin)/grid_x)) 
        p_y =  int(np.floor((point[1]-ymin)/grid_y))
        
        if (p_x<0 or p_x>=grid_size_x) or (p_y<0 or p_y>=grid_size_y):
            continue
        else:
            points_of_grids[p_x,p_y, int(num_of_points[p_x,p_y])] = point

            num_of_points[p_x,p_y]+=1



    #todo-----------------------计算每个网格的不同的特征值---------------------------------
    #?-----待解决：
    #?----- 1.有点的网格和无点的网格之间的差值如何计算？
    #!--------------------------高程
    height_of_grround = -1.30 #设定一个地面高度给无点的网格
    num_of_grids_contain_points = 0 #记录含有点的网格的数量
    height_of_grids = np.zeros((grid_size_x,grid_size_y))
    for i in range(grid_size_x):
        for j in range(grid_size_y):
            num_point_in_grid = int(num_of_points[i,j]) #该网格内点的数量
            if num_point_in_grid==0: #如果点的数量为0,则跳过
                height_of_grids[i,j] = height_of_grround
                continue
            num_of_grids_contain_points +=1
            height_of_grids[i,j] = np.sum(points_of_grids[i,j,:num_point_in_grid,2])/num_point_in_grid

    #------------------------计算相邻网格之间的高程差，沿x轴之间的高程差较小，y轴之间的高程差较大的网格


    d_value_of_height_x = np.zeros((grid_size_x,grid_size_y))
    d_value_of_height_y = np.zeros((grid_size_x,grid_size_y))



    for i in range(grid_size_x):
        for j in range(grid_size_y):
            #两个边界点的高程差值 0和grid_size_y-1
            if j!=0 and j!=grid_size_y-1:
                d_value_of_height_y[i,j] = max( abs(height_of_grids[i,j-1]-height_of_grids[i,j]),abs(height_of_grids[i,j]-height_of_grids[i,j+1]) )
                
            elif j==0:
                d_value_of_height_y[i,j] = abs(height_of_grids[i,j]-height_of_grids[i,j+1])
            elif j==grid_size_y-1:
                d_value_of_height_y[i,j] = abs(height_of_grids[i,j-1]-height_of_grids[i,j])
            
            else:
                raise KeyError('wrong index!')



    for i in range(grid_size_x):
        for j in range(grid_size_y):
            #两个边界点的高程差值 0和grid_size_x-1
            if i!=0 and i!=grid_size_x-1:
                d_value_of_height_x[i,j] = max( abs(height_of_grids[i-1,j]-height_of_grids[i,j]),abs(height_of_grids[i,j]-height_of_grids[i+1,j]) )
                
            elif i==0:
                d_value_of_height_x[i,j] = abs(height_of_grids[i,j]-height_of_grids[i+1,j])
            elif i==grid_size_x-1:
                d_value_of_height_x[i,j] = abs(height_of_grids[i-1,j]-height_of_grids[i,j])
            
            else:
                raise KeyError('wrong index!')




    #------------------------先获得平均高度，然后计算每个网格和平均高度之间的差值，没有点的网格高度差为0

    # avg_height_of_grid = np.sum(height_of_grids)/num_of_grids_contain_points #计算所有网格的平均高度

    # # print(num_of_grids_contain_points)
    # # print(avg_height_of_grid)

    # #计算有点的网格与平均高度之间的差值
    # for i in range(grid_size_x):
    #     for j in range(grid_size_y):
    #         num_point_in_grid = int(num_of_points[i,j]) #该网格内点的数量
    #         if num_point_in_grid==0: #如果点的数量为0,则跳过
    #             continue
    #         height_of_grids[i,j] = abs(height_of_grids[i,j]-avg_height_of_grid)

    #!--------------------------计算每个网格高度的方差和均值
    mean,var = mean_and_var_of_grids(num_of_points,points_of_grids)

    #!--------------------------坡度


    #!--------------------------反射率的变化

    #!--------------------------点数量的变化

    #!--------------------------与邻近网格连续性


    #todo------------------------------绘制3d柱状图-------------------------------------
    # #构造需要显示的值
    # Y=np.arange(0, grid_size_x, step=1)#X轴的坐标
    # X=np.arange(0, grid_size_y, step=1)#Y轴的坐标
    # xx, yy=np.meshgrid(X, Y)#网格化坐标
    # X, Y=xx.ravel(), yy.ravel()#矩阵扁平化
    # bottom=np.zeros_like(Y)#设置柱状图的底端位值
    # width=height=1#每一个柱子的长和宽
    # num_of_points = num_of_points.ravel()

    # #绘图设置
    # fig=plt.figure()
    # ax=fig.gca(projection='3d')#三维坐标轴
    # ax.bar3d(X, Y, bottom, width, height, num_of_points, shade=True)#
    # #坐标轴设置
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('num_of_points')
    # plt.show()

    #todo------------------------------绘制灰度图----------------------------------------

    # cv2.imshow('num_of_pointsimage', num_of_points)
    # cv2.imwrite('/media/yhserver/DATA1/point_cloud/tool_pcd/num_of_points.jpg', num_of_points)
    # cv2.waitKey(-1)
    # cv2.destroyAllWindows()


    # cv2.imshow('height_of_gridsimage', height_of_grids)
    # cv2.imwrite('/media/yhserver/DATA1/point_cloud/tool_pcd/height_of_grids.jpg', height_of_grids)
    # cv2.waitKey(-1)
    # cv2.destroyAllWindows()


    # cv2.imshow('d_value_of_height_y', d_value_of_height_y)
    # cv2.imwrite('/media/yhserver/DATA1/point_cloud/tool_pcd/d_value_of_height_y.jpg', d_value_of_height_y)
    # cv2.waitKey(-1)
    # cv2.destroyAllWindows()

    # cv2.imshow('d_value_of_height_x', d_value_of_height_x)
    # cv2.imwrite('/media/yhserver/DATA1/point_cloud/tool_pcd/d_value_of_height_x.jpg', d_value_of_height_x)
    # cv2.waitKey(-1)
    # cv2.destroyAllWindows()


    cv2.imshow('var*100', var*100)
    cv2.imwrite('/media/yhserver/DATA1/point_cloud/tool_pcd/var_100.jpg', var)
    cv2.waitKey(-1)
    cv2.destroyAllWindows()
    