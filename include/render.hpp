/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-23 09:44:11
 * @LastEditTime: 2021-12-27 16:19:28
 * @FilePath: /Lidar/include/render.hpp
 */

#pragma once
#include <eigen3/Eigen/Dense>
#include <pcl-1.11/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.11/pcl/point_cloud.h>
#include <vector>
#include <string>
#include "box.h"

struct Color
{
    Color(int r, int g, int b)
    {
        this->r = r;
        this->g = g;
        this->b = b;
    }
    int r;
    int g;
    int b;
};
void initCamera(pcl::visualization::PCLVisualizer::Ptr viewer, Eigen::VectorXf &posVector);
// 显示聚类
template <typename PointT>
void renderClusters(pcl::visualization::PCLVisualizer::Ptr viewer,  std::vector< typename pcl::PointCloud<PointT>::Ptr > &clusters, Color color, std::string name);
// 显示聚类边界框
template<typename PointT>
void renderClusterBox(pcl::visualization::PCLVisualizer::Ptr viewer, ProcessPointClouds<PointT> * pointProcessorI, std::vector< typename pcl::PointCloud<PointT>::Ptr > &clusters, Color color, std::string name);

// 显示点云
template <typename PointT>
void renderCloud(pcl::visualization::PCLVisualizer::Ptr viewer, typename pcl::PointCloud<PointT>::Ptr cloud, Color color, std::string name, int v=0);

//----------------------------------------------------------------------------------------------------------------------------------------
void initCamera(pcl::visualization::PCLVisualizer::Ptr viewer, Eigen::VectorXf &posVector)
{
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(1.0);
    // pcl::visualization::Camera中获取的相机pos,view参数
    viewer->setCameraPosition(posVector(0), posVector(1), posVector(2), posVector(3), posVector(4), posVector(5));
}

// 显示聚类
template <typename PointT> 
void renderClusters(pcl::visualization::PCLVisualizer::Ptr viewer, std::vector< typename pcl::PointCloud<PointT>::Ptr > &clusters, Color color, std::string name)
{
    int cluster_id = 0;
    for (auto cluster_i : clusters)
    {   
        pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cluster_i, color.r, color.g, color.b);
        viewer->addPointCloud(cluster_i, rgb, name + std::to_string(cluster_id));
        cluster_id++;
    }
}
// 显示聚类边框
template<typename PointT>
void renderClusterBox(pcl::visualization::PCLVisualizer::Ptr viewer, ProcessPointClouds<PointT> * pointProcessorI,  std::vector< typename pcl::PointCloud<PointT>::Ptr > &clusters, Color color, std::string name){
    int cluster_id = 0;
    for (auto cluster_i : clusters)
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cluster_i, color.r, color.g, color.b);
        Box box = pointProcessorI->boundingBox(cluster_i);
        box.getBox();
        // box.printBoxInfo();
        boxRender(viewer, cluster_id, box);
        cluster_id++;
    }
}

// 显示点云
template <typename PointT>
void renderCloud(pcl::visualization::PCLVisualizer::Ptr viewer, typename pcl::PointCloud<PointT>::Ptr cloud, Color color, std::string name, int v)
{
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud, color.r, color.g, color.b);
    viewer->addPointCloud(cloud, rgb, name, v);
}



