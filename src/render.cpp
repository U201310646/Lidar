/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-23 09:44:06
 * @LastEditTime: 2021-12-23 11:21:04
 * @FilePath: /Lidar/src/render.cpp
 */

#include "render.h"

void initCamera(pcl::visualization::PCLVisualizer::Ptr viewer, Eigen::VectorXf & posVector){
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(1.0);
    // pcl::visualization::Camera中获取的相机pos,view参数
    viewer->setCameraPosition(posVector(0), posVector(1), posVector(2), posVector(3), posVector(4), posVector(5));
}