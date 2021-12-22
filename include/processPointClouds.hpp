/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-18 13:32:56
 * @LastEditTime: 2021-12-20 14:45:40
 * @FilePath: /Lidar/include/processPointClouds.hpp
 */
#pragma once
#include <iostream>
#include <pcl-1.11/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.11/pcl/io/pcd_io.h>
#include <pcl-1.11/pcl/point_cloud.h>
#include <pcl-1.11/pcl/point_types.h>
#include <pcl-1.11/pcl/filters/voxel_grid.h>
#include <pcl-1.11/pcl/filters/extract_indices.h>
#include <pcl-1.11/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.11/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.11/pcl/filters/crop_box.h>
#include <pcl-1.11/pcl/search/kdtree.h>
#include <pcl-1.11/pcl/segmentation/extract_clusters.h>
#include <boost/thread/thread.hpp>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

template<typename PointT>
using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;

template<typename PointT>
class ProcessPointClouds{
public:
    ProcessPointClouds(){};
    ~ProcessPointClouds(){};
    void numPoints(PtCdtr<PointT> cloud);
    PtCdtr<PointT> loadPcd(std::string file);
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    //todo 滤波，分割，聚类，拟合..
    //对ROI感兴趣区域进行focus，体素最近点滤波
    PtCdtr<PointT> cloud_filter(PtCdtr<PointT> cloud_in, float leaf_size, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
    // 平面分割
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_Plane(PtCdtr<PointT> cloud, int maxIterations, float distanceThreshold);
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_Indices(PtCdtr<PointT> cloud, pcl::PointIndices::Ptr ind);
    // 点云聚类
    std::vector<PtCdtr<PointT>> cluster(PtCdtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize);
    
};

//*********************************************************************************************
template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::loadPcd(std::string file){
    PtCdtr<PointT> cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) { //* load the file
        PCL_ERROR ("Couldn't read file \n");
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;
    }
    return cloud;
};
template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(PtCdtr<PointT> cloud){
    std::cout << cloud->points.size() << std::endl;
};

// pcd文件流，遍历指定目录下所有文件
template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath){
    boost::filesystem::directory_iterator iter(dataPath);
    boost::filesystem::directory_iterator iter_end;
    std::vector<boost::filesystem::path> paths(iter, iter_end);
    // 按名称顺序进程排列（时间顺序）
    sort(paths.begin(), paths.end());
    return paths;
};

// 根据索引进行分割，first为索引点， second为索引外点
template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>> ProcessPointClouds<PointT>::segment_Indices(PtCdtr<PointT> cloud, pcl::PointIndices::Ptr ind){
        pcl::ExtractIndices<PointT> extractor;
        extractor.setInputCloud(cloud);
        extractor.setIndices(ind);
        extractor.setNegative(false);
        PtCdtr<PointT> inModel_Cloud(new pcl::PointCloud<PointT>);
        PtCdtr<PointT> outModel_Cloud(new pcl::PointCloud<PointT>);
        extractor.filter(*inModel_Cloud);
        extractor.setNegative(true);
        extractor.filter(*outModel_Cloud);
        std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair(inModel_Cloud, outModel_Cloud);
        return segment_resultPair;
};

// 平面分割，返回pair， first在模型内点，second模型外点
template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>> ProcessPointClouds<PointT>::segment_Plane(PtCdtr<PointT> cloud, int maxIterations, float distanceThreshold){
    pcl::PointIndices::Ptr ind(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> segments;
    segments.setOptimizeCoefficients(true);
    segments.setMaxIterations(maxIterations);
    segments.setDistanceThreshold(distanceThreshold);
    segments.setModelType(pcl::SACMODEL_PLANE);
    segments.setMethodType(pcl::SAC_RANSAC);
    segments.setInputCloud(cloud);
    segments.segment(*ind, *coefficient);
    if (ind->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
    }
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = segment_Indices(cloud, ind);
    return segment_resultPair; 
};

// 点云预处理：ROI滤波，体素就近滤波
template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::cloud_filter(PtCdtr<PointT> cloud_in, float leaf_size, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){
    // ROI滤波
    PtCdtr<PointT> cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region;
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_in);
    region.filter(*cloudRegion);

    // 体素就近滤波
    PtCdtr<PointT> cloud_out(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloudRegion); 
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter (*cloud_out);

    // kdtree寻找最近点代替理论重心
    pcl::PointIndices::Ptr ind(new pcl::PointIndices);
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloudRegion);
    int K = 1;
    for(auto search_point: *cloud_out){

        std::vector<int> pointId_KNNSearch;
        std::vector<float> sqrDistance_KNNSearch;
        if (kdtree.nearestKSearch(search_point, K, pointId_KNNSearch, sqrDistance_KNNSearch)>0 ){
            ind->indices.push_back(pointId_KNNSearch[0]);
        }
    }
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = segment_Indices(cloudRegion, ind);
    return segment_resultPair.first;
};

// 欧拉聚类
template<typename PointT>
std::vector<PtCdtr<PointT>> ProcessPointClouds<PointT>::cluster(PtCdtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize){
    std::vector<PtCdtr<PointT>> clusters;
    typename pcl::search::KdTree<PointT>::Ptr kdtree;
    kdtree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_ind;
    pcl::EuclideanClusterExtraction<PointT> eucliCluster;
    eucliCluster.setClusterTolerance(clusterTolerance);
    eucliCluster.setMaxClusterSize(maxSize);
    eucliCluster.setMinClusterSize(minSize);
    eucliCluster.setSearchMethod(kdtree);
    eucliCluster.setInputCloud(cloud);
    eucliCluster.extract(cluster_ind);

    for(auto pointIndices_i:cluster_ind){
        pcl::PointIndices::Ptr pointIndicesPtr_i(new pcl::PointIndices(pointIndices_i));
        std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = segment_Indices(cloud ,pointIndicesPtr_i);  
        clusters.push_back(segment_resultPair.first);
    }
    return clusters;
    
};