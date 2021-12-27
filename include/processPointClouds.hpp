/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-18 13:32:56
 * @LastEditTime: 2021-12-24 15:34:06
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
#include <pcl-1.11/pcl/features/normal_3d.h>
#include <pcl-1.11/pcl/segmentation/region_growing.h>
#include <pcl-1.11/pcl/common/common.h>
#include <boost/thread/thread.hpp>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include "box.h"

template<typename PointT>
using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;

using PtCdNormtr = typename pcl::PointCloud<pcl::Normal>::Ptr;

template<typename PointT>
class ProcessPointClouds{
public:
    ProcessPointClouds(){};
    ~ProcessPointClouds(){};
    void numPoints(PtCdtr<PointT> cloud);
    PtCdtr<PointT> loadPcd(std::string file);
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    // 将聚类的PointIndices索引容器转为PointCloud容器
    std::vector<PtCdtr<PointT>> ind2cloud(PtCdtr<PointT> cloud, std::vector<pcl::PointIndices>& cluster_ind);
    // 两个点云索引vector中交集索引数量
    int intersectVertor(std::vector<int>& v1, std::vector<int>& v2);
    // 对ROI感兴趣区域进行focus
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roi_filter(PtCdtr<PointT> cloud_in, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, std::vector<int>& indices, std::vector<int>& roni_indices);
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roi_filter(PtCdtr<PointT> cloud_in, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
    // 体素就近点滤波
    PtCdtr<PointT> voxel_filter(PtCdtr<PointT> cloud_in, float leaf_size);
    // 平面分割
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_Plane(PtCdtr<PointT> cloud, int maxIterations, float distanceThreshold, pcl::ModelCoefficients::Ptr coefficient = std::make_shared<pcl::ModelCoefficients>());
    // 根据索引进行分割，first为索引内点， second为索引外点
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_Indices(PtCdtr<PointT> cloud, pcl::PointIndices::Ptr ind);
    // 欧拉点云聚类,得到聚类索引
    std::vector<pcl::PointIndices> cluster(PtCdtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize);
    // 删除聚类中聚类索引与其他索引集合交集数大于阈值的聚类（例如： 与非roi区域有一定交集的聚类会被整体删除）
    void clusterFilter(std::vector<pcl::PointIndices>& cluster_ind, std::vector<int>& indices, int insectThreshold);
    // 点云法向量
    pcl::PointCloud <pcl::Normal>::Ptr normalExtractor(PtCdtr<PointT> cloud, float radius);
    // 区域生长聚类
    // nearPoints临近点的个数，CurvatureThreshold曲率的阈值，SmoothnessThreshold法线差值阈值
    std::vector<pcl::PointIndices> regionGrown(PtCdtr<PointT> cloud, int nearPoints,int minSize, int maxSize, float CurvatureThreshold, float SmoothnessThreshold, PtCdNormtr normals);
    // 获取聚类边界框
    Box boundingBox(PtCdtr<PointT> cluster);
};  

//-------------------------------------------------------------------------------------------------------------------------
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

// 将聚类的PointIndices索引容器转为PointCloud容器
template<typename PointT>
std::vector<PtCdtr<PointT>> ProcessPointClouds<PointT>::ind2cloud(PtCdtr<PointT> cloud ,std::vector<pcl::PointIndices>& cluster_ind){
    std::vector<PtCdtr<PointT>> clusters;
    for(auto pointIndices_i:cluster_ind){
        pcl::PointIndices::Ptr pointIndicesPtr_i(new pcl::PointIndices(pointIndices_i));
        std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = segment_Indices(cloud, pointIndicesPtr_i);  
        clusters.push_back(segment_resultPair.first);
    }
    return clusters;
}

// 两个点云索引vector中交集索引数量
template<typename PointT>
int ProcessPointClouds<PointT>::intersectVertor(std::vector<int>& v1, std::vector<int>& v2){
    
    int insect_count = 0;
    // 对vector进行元素排序
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    std::vector<int> vector_insect;
    vector_insect.resize(std::min<int>(v1.size(), v2.size()));
    std::vector<int>::iterator itEnd = std::set_intersection(v1.begin(), v1.end(),v2.begin(),v2.end(),vector_insect.begin());
    for(auto it = vector_insect.begin(); it != itEnd; it++){
        insect_count++;
    };
    return insect_count;
}

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
std::pair<PtCdtr<PointT>, PtCdtr<PointT>> ProcessPointClouds<PointT>::segment_Plane(PtCdtr<PointT> cloud, int maxIterations, float distanceThreshold, pcl::ModelCoefficients::Ptr coefficient){
    pcl::PointIndices::Ptr ind(new pcl::PointIndices);
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


// ROI滤波: focus the region of interest. first为roi区域，second为roni区域
template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>> 
ProcessPointClouds<PointT>::roi_filter(PtCdtr<PointT> cloud_in, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, std::vector<int>& roi_indices, std::vector<int>& roni_indices){
    // ROI滤波
    PtCdtr<PointT> roi_cloud(new pcl::PointCloud<PointT>);
    PtCdtr<PointT> roni_cloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region;
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_in);
    region.setNegative(false);
    region.filter(*roi_cloud);
    region.filter(roi_indices);

    region.setNegative(true);
    region.filter(*roni_cloud);
    region.filter(roni_indices);

    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roiResultPair(roi_cloud, roni_cloud);
    return roiResultPair;
};
// ROI滤波重载
template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>> 
ProcessPointClouds<PointT>::roi_filter(PtCdtr<PointT> cloud_in, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){
    // ROI滤波
    PtCdtr<PointT> roi_cloud(new pcl::PointCloud<PointT>);
    PtCdtr<PointT> roni_cloud(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region;
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_in);
    region.setNegative(false);
    region.filter(*roi_cloud);
    region.setNegative(true);
    region.filter(*roni_cloud);

    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roiResultPair(roi_cloud, roni_cloud);
    return roiResultPair;
};

// 点云预处理: 体素就近滤波
template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::voxel_filter(PtCdtr<PointT> cloud_in, float leaf_size){

    // 体素就近滤波
    PtCdtr<PointT> cloud_out(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud_in); 
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter (*cloud_out);

    // kdtree寻找最近点代替理论重心
    pcl::PointIndices::Ptr ind(new pcl::PointIndices);
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud_in);
    int K = 1;
    for(auto search_point: *cloud_out){
        std::vector<int> pointId_KNNSearch;
        std::vector<float> sqrDistance_KNNSearch;
        if (kdtree.nearestKSearch(search_point, K, pointId_KNNSearch, sqrDistance_KNNSearch)>0 ){
            ind->indices.push_back(pointId_KNNSearch[0]);
        }
    }
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = segment_Indices(cloud_in, ind);
    return segment_resultPair.first;
};

// 欧拉聚类
template<typename PointT>
std::vector<pcl::PointIndices> ProcessPointClouds<PointT>::cluster(PtCdtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize){
    std::vector<pcl::PointIndices> cluster_ind;
    typename pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    kdtree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<PointT> eucliCluster;
    eucliCluster.setClusterTolerance(clusterTolerance);
    eucliCluster.setMaxClusterSize(maxSize);
    eucliCluster.setMinClusterSize(minSize);
    eucliCluster.setSearchMethod(kdtree);
    eucliCluster.setInputCloud(cloud);
    eucliCluster.extract(cluster_ind);
    return cluster_ind;
    
};
// 删除聚类中聚类索引与其他索引集合交集数大于阈值的聚类（例如： 与非roi区域有一定交集的聚类会被整体删除）
template<typename PointT>
void ProcessPointClouds<PointT>::clusterFilter(std::vector<pcl::PointIndices>& cluster_ind, std::vector<int>& indices, int insectThreshold){
    for (auto it = cluster_ind.begin(); it != cluster_ind.end(); it++)
    {
        int insect_count = intersectVertor(it->indices, indices);
        if (insect_count > insectThreshold)
        {
            it = cluster_ind.erase(it);
            it--;
        }
    }
}
// 点云法向量
template<typename PointT>
PtCdNormtr ProcessPointClouds<PointT>::normalExtractor(PtCdtr<PointT> cloud, float radius){
        typename pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
        PtCdNormtr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<PointT, pcl::Normal> n;
        n.setSearchMethod(kdtree);
        n.setInputCloud(cloud);
        // n.setKSearch(k);
        n.setRadiusSearch(radius);
        n.compute(*normals);

        return normals;
    }
// 区域生长聚类
template<typename PointT>
std::vector<pcl::PointIndices> ProcessPointClouds<PointT>::regionGrown(PtCdtr<PointT> cloud, int nearPoints,int minSize, int maxSize, float CurvatureThreshold, float SmoothnessThreshold, PtCdNormtr normals){
        typename pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
        std::vector<pcl::PointIndices> cluster_ind;
        pcl::RegionGrowing<PointT, pcl::Normal> reg;
        reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.setMinClusterSize(minSize);
        reg.setMaxClusterSize(maxSize);
        reg.setSearchMethod(kdtree);
        reg.setNumberOfNeighbours(nearPoints);
        reg.setCurvatureThreshold(CurvatureThreshold);
        reg.setSmoothnessThreshold(SmoothnessThreshold / 180.0 * M_PI);
        reg.extract(cluster_ind);
        return cluster_ind;
    }

template<typename PointT>
Box ProcessPointClouds<PointT>::boundingBox(PtCdtr<PointT> cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);
    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;
    return box;
}

