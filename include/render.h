/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-23 09:44:11
 * @LastEditTime: 2021-12-23 11:19:32
 * @FilePath: /Lidar/include/render.h
 */
#pragma once
#include <eigen3/Eigen/Dense>
#include <pcl-1.11/pcl/visualization/pcl_visualizer.h>

struct Color{
    Color(int r, int g, int b){
        this->r = r;
        this->g = g;
        this->b = b;   
    }
    int r;
    int g;
    int b;
};
void initCamera(pcl::visualization::PCLVisualizer::Ptr viewer, Eigen::VectorXf& posVector);

/* template<typename PointT>
void renderClusters(std::vector<pcl::PointCloud<PointT>::Ptr> clusters, Color color, std::string name){
    int cluster_id = 0;
    for (auto cluster_i : clusters){
        pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cluster_i, color.r, color.g, color.b); 
        viewer->addPointCloud(cluster_i, rgb, name+std::to_string(cluster_id));
        cluster_id++;
    }
} */