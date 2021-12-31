/*
 * @Description:
 * @Author: yurui
 * @Date: 2021-12-18 13:32:56
 * @LastEditTime: 2021-12-27 17:52:21
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
#include <pcl-1.11/pcl/filters/statistical_outlier_removal.h>
#include <pcl-1.11/pcl/filters/crop_box.h>
#include <pcl-1.11/pcl/search/kdtree.h>
#include <pcl-1.11/pcl/segmentation/extract_clusters.h>
#include <pcl-1.11/pcl/segmentation/sac_segmentation.h>
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

template <typename PointT>
using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;
using PtCdNormtr = typename pcl::PointCloud<pcl::Normal>::Ptr;

// 路沿或者栅栏拟合面的参数， 面内的点数， 面内点云的索引
struct verticalPlane
{
    pcl::ModelCoefficients::Ptr coefficient;
    int numPoints;
    std::vector<int> indices;
};

template <typename PointT>
class ProcessPointClouds
{
public:
    ProcessPointClouds(){};
    ~ProcessPointClouds(){};
    void numPoints(PtCdtr<PointT> cloud);
    PtCdtr<PointT> loadPcd(std::string file);
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    // 将聚类的PointIndices索引容器转为PointCloud容器
    std::vector<PtCdtr<PointT>> ind2cloud(PtCdtr<PointT> cloud, std::vector<pcl::PointIndices> &cluster_ind);
    // 两个点云索引vector中交集索引数量
    int intersectVertor(std::vector<int> &v1, std::vector<int> &v2);
    // 对ROI感兴趣区域进行focus
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roi_filter(PtCdtr<PointT> cloud_in, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, std::vector<int> &indices, std::vector<int> &roni_indices);
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roi_filter(PtCdtr<PointT> cloud_in, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
    // 体素就近点滤波
    PtCdtr<PointT> voxel_filter(PtCdtr<PointT> cloud_in, float leaf_size);
    // 统计滤波：滤除离群点
    PtCdtr<PointT> statistic_filter(PtCdtr<PointT> cloud_in, int nearPoints, int stdNum);
    // 平面分割
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_Plane(PtCdtr<PointT> cloud, int maxIterations, float distanceThreshold, pcl::ModelCoefficients::Ptr coefficient = std::make_shared<pcl::ModelCoefficients>());
    // 根据索引进行分割，first为索引内点， second为索引外点
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_Indices(PtCdtr<PointT> cloud, pcl::PointIndices::Ptr ind);
    // 欧拉点云聚类,得到聚类索引
    std::vector<pcl::PointIndices> cluster(PtCdtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize);
    // 删除聚类中聚类索引与其他索引集合交集数大于阈值的聚类（例如： 与非roi区域有一定交集的聚类会被整体删除）
    void clusterFilter(std::vector<pcl::PointIndices> &cluster_ind, std::vector<int> &indices, int insectThreshold);
    // 点云法向量
    pcl::PointCloud<pcl::Normal>::Ptr normalExtractor(PtCdtr<PointT> cloud, float radius);
    // 区域生长聚类
    // nearPoints临近点的个数，CurvatureThreshold曲率的阈值，SmoothnessThreshold法线差值阈值
    std::vector<pcl::PointIndices> regionGrown(PtCdtr<PointT> cloud, int nearPoints, int minSize, int maxSize, float CurvatureThreshold, float SmoothnessThreshold, PtCdNormtr normals);
    // 获取聚类边界框
    Box boundingBox(PtCdtr<PointT> cluster);
    // 检测垂直路面平面的点
    std::vector<verticalPlane> verticalPlane_detect(PtCdtr<PointT> cloud, pcl::ModelCoefficients::Ptr coefficient);
    // 直线拟合
    PtCdtr<PointT> lineSeg(PtCdtr<PointT> cloud, float distance=0.05, int maxIteration=40);
};


//-------------------------------------------------------------------------------------------------------------------------
template <typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::loadPcd(std::string file)
{
    PtCdtr<PointT> cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)
    { //* load the file
        PCL_ERROR("Couldn't read file \n");
        std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;
    }
    return cloud;
};
template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(PtCdtr<PointT> cloud)
{
    std::cout << cloud->points.size() << std::endl;
};

// pcd文件流，遍历指定目录下所有文件
template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    boost::filesystem::directory_iterator iter(dataPath);
    boost::filesystem::directory_iterator iter_end;
    std::vector<boost::filesystem::path> paths(iter, iter_end);
    // 按名称顺序进程排列（时间顺序）
    sort(paths.begin(), paths.end());
    return paths;
};

// 将聚类的PointIndices索引容器转为PointCloud容器
template <typename PointT>
std::vector<PtCdtr<PointT>> ProcessPointClouds<PointT>::ind2cloud(PtCdtr<PointT> cloud, std::vector<pcl::PointIndices> &cluster_ind)
{
    std::vector<PtCdtr<PointT>> clusters;
    for (auto pointIndices_i : cluster_ind)
    {
        pcl::PointIndices::Ptr pointIndicesPtr_i(new pcl::PointIndices(pointIndices_i));
        std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = segment_Indices(cloud, pointIndicesPtr_i);
        clusters.push_back(segment_resultPair.first);
    }
    return clusters;
}

// 两个点云索引vector中交集索引数量
template <typename PointT>
int ProcessPointClouds<PointT>::intersectVertor(std::vector<int> &v1, std::vector<int> &v2)
{

    int insect_count = 0;
    // 对vector进行元素排序
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    std::vector<int> vector_insect;
    vector_insect.resize(std::min<int>(v1.size(), v2.size()));
    std::vector<int>::iterator itEnd = std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), vector_insect.begin());
    for (auto it = vector_insect.begin(); it != itEnd; it++)
    {
        insect_count++;
    };
    return insect_count;
}

// 根据索引进行分割，first为索引点， second为索引外点
template <typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>> ProcessPointClouds<PointT>::segment_Indices(PtCdtr<PointT> cloud, pcl::PointIndices::Ptr ind)
{
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
template <typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>> ProcessPointClouds<PointT>::segment_Plane(PtCdtr<PointT> cloud, int maxIterations, float distanceThreshold, pcl::ModelCoefficients::Ptr coefficient)
{
    pcl::PointIndices::Ptr ind(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> segments;
    segments.setOptimizeCoefficients(true);
    segments.setMaxIterations(maxIterations);
    segments.setDistanceThreshold(distanceThreshold);
    segments.setModelType(pcl::SACMODEL_PLANE);
    segments.setMethodType(pcl::SAC_RANSAC);
    segments.setInputCloud(cloud);
    segments.segment(*ind, *coefficient);
    if (ind->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
    }
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = segment_Indices(cloud, ind);
    return segment_resultPair;
};

// ROI滤波: focus the region of interest. first为roi区域，second为roni区域
template <typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
ProcessPointClouds<PointT>::roi_filter(PtCdtr<PointT> cloud_in, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, std::vector<int> &indices, std::vector<int> &roni_indices)
{
    // ROI滤波
    PtCdtr<PointT> roi_cloud(new pcl::PointCloud<PointT>);
    PtCdtr<PointT> roni_cloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region;
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_in);
    region.setNegative(false);
    region.filter(*roi_cloud);
    region.filter(indices);

    region.setNegative(true);
    region.filter(*roni_cloud);
    region.filter(roni_indices);

    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roiResultPair(roi_cloud, roni_cloud);
    return roiResultPair;
};
// ROI滤波重载
template <typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
ProcessPointClouds<PointT>::roi_filter(PtCdtr<PointT> cloud_in, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
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
template <typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::voxel_filter(PtCdtr<PointT> cloud_in, float leaf_size)
{

    // 体素就近滤波
    PtCdtr<PointT> cloud_out(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud_in);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*cloud_out);

    // kdtree寻找最近点代替理论重心
    pcl::PointIndices::Ptr ind(new pcl::PointIndices);
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud_in);
    int K = 1;
    for (auto search_point : *cloud_out)
    {
        std::vector<int> pointId_KNNSearch;
        std::vector<float> sqrDistance_KNNSearch;
        if (kdtree.nearestKSearch(search_point, K, pointId_KNNSearch, sqrDistance_KNNSearch) > 0)
        {
            ind->indices.push_back(pointId_KNNSearch[0]);
        }
    }
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = segment_Indices(cloud_in, ind);
    return segment_resultPair.first;
};
//
template <typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::statistic_filter(PtCdtr<PointT> cloud_in, int nearPoints, int stdNum)
{
    PtCdtr<PointT> cloud_out(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> stat;
    stat.setInputCloud(cloud_in);
    stat.setMeanK(nearPoints);
    stat.setStddevMulThresh(stdNum);
    stat.filter(*cloud_out);
    return cloud_out;
}

// 欧拉聚类
template <typename PointT>
std::vector<pcl::PointIndices> ProcessPointClouds<PointT>::cluster(PtCdtr<PointT> cloud, float clusterTolerance, int minSize, int maxSize)
{
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
template <typename PointT>
void ProcessPointClouds<PointT>::clusterFilter(std::vector<pcl::PointIndices> &cluster_ind, std::vector<int> &indices, int insectThreshold)
{
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
template <typename PointT>
PtCdNormtr ProcessPointClouds<PointT>::normalExtractor(PtCdtr<PointT> cloud, float radius)
{
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
template <typename PointT>
std::vector<pcl::PointIndices> ProcessPointClouds<PointT>::regionGrown(PtCdtr<PointT> cloud, int nearPoints, int minSize, int maxSize, float CurvatureThreshold, float SmoothnessThreshold, PtCdNormtr normals)
{
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

template <typename PointT>
Box ProcessPointClouds<PointT>::boundingBox(PtCdtr<PointT> cluster)
{

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

// 垂直地面平面检测
// 输入需要检测的点云集，对应的索引，以及地面法向量参数
template <typename PointT>
std::vector<verticalPlane> ProcessPointClouds<PointT>::verticalPlane_detect(PtCdtr<PointT> cloud, pcl::ModelCoefficients::Ptr coefficient)
{

    //--------------------------------------------------------
    // 自定义参数
    float verticalThreshold = 10;         // 与路面法向量夹角（90±threshold）
    float distance2plane_Threshold = 0.5; // 点云到拟合平面的距离阈值
    // 自定义参数
    float parrallelThreshold = 20;   // 平面阈值距离内的点的法向量与面的法向量的夹角阈值，小于该阈值才做点数统计
    float curvatureThreshold = 0.05; // 同上，对于曲率进行阈值点数统计，这里没有使用
    // 自定义参数
    int numPointsThreshold = 200;   // 最终统计的点数阈值
    float planeAngleThreshold = 20; // 结果验证：如果和已经拟合的面面夹角小于阈值，且面面之间的距离小于阈值则可拟合的是同一个平面
    int distancePlane2Plane = 5;
    // 法向量估计参数
    float radius = 0.5;
    //--------------------------------------------------------
    // 存储最终拟合结果
    std::vector<verticalPlane> verticalPlanes;
    // regionCloud中每个点的法向量,其中存在很多的0向量和nan向量
    pcl::PointCloud<pcl::Normal>::Ptr normals = normalExtractor(cloud, radius);
    Eigen::Map<Eigen::Vector3f> plane_normal(&coefficient->values[0], 3);

    // 标记点数
    int marktimes = 0;
    // 标记点集合初始化
    std::vector<bool> processed(cloud->points.size(), false);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        // 处理0向量和nan向量
        if (std::isnan(normals->points[i].normal_x) || std::isnan(normals->points[i].normal_y) || std::isnan(normals->points[i].normal_z))
        {
            // 标记为坏点
            processed[i] = true;
            marktimes++;
            continue;
        }
        float normSquare = (std::pow(normals->points[i].normal_x, 2) + std::pow(normals->points[i].normal_y, 2) + std::pow(normals->points[i].normal_z, 2));
        if (normSquare < 0.1)
        {
            // 标记为坏点
            processed[i] = true;
            marktimes++;
            continue;
        }
    }

    // jump次数
    int jumptimes = 0;
    // 计时开始
    auto startTime = std::chrono::steady_clock::now();
    srand(time(NULL));
    // 遍历点云中的所有点
    for (int ind = 0; ind < cloud->points.size(); ind++)
    {
        struct verticalPlane vp;
        // 如果是坏点则跳过
        if (processed[ind])
        {
            // std::cout<< "jump" << std::endl;
            jumptimes++;
            continue;
        }
        Eigen::Vector3f point_normal;
        float pointCurvature;
        point_normal(0) = normals->points[ind].normal_x;
        point_normal(1) = normals->points[ind].normal_y;
        point_normal(2) = normals->points[ind].normal_z;
        pointCurvature = normals->points[ind].curvature;
        // 计算点法向量与面向量夹角(0~180),只处理法向量与地面垂直的点
        float angle = pcl::getAngle3D(plane_normal, point_normal, true);

        // std::cout <<"法向量和面的夹角: "<< angle <<std::endl;
        // 自定义参数
        // float verticalThreshold = 10; // 与路面法向量夹角（90±threshold）
        // float distance2plane_Threshold = 0.3; // 点云到拟合平面的距离阈值

        if (std::abs(angle - 90) < verticalThreshold)
        {
            pcl::PointXYZ point;
            point.x = cloud->points[ind].x;
            point.y = cloud->points[ind].y;
            point.z = cloud->points[ind].z;
            // 通过点的坐标和法向量确定平面, 存入struct verticalPlane结构体中
            float d = -(point_normal(0) * point.x + point_normal(1) * point.y + point_normal(2) * point.z);

            // std::cout << "normal_x: " << point_normal(0)<< "normal_y: " << point_normal(1)<< "normal_z: " << point_normal(2) << std::endl;

            pcl::ModelCoefficients::Ptr coefficientPlane(new pcl::ModelCoefficients);
            coefficientPlane->values.push_back(point_normal(0));
            coefficientPlane->values.push_back(point_normal(1));
            coefficientPlane->values.push_back(point_normal(2));
            coefficientPlane->values.push_back(d);
            vp.coefficient = coefficientPlane;

            int numPoints = 0;
            for (int indice = 0; indice < cloud->points.size(); indice++)
            {
                float distance = point_normal(0) * cloud->points[indice].x + point_normal(1) * cloud->points[indice].y + point_normal(2) * cloud->points[indice].z + d;
                // std::cout<< "distance: "<<std::abs(distance) << std::endl;
                // 自定义参数parrallelThreshold，curvatureThreshold

                if (!processed[indice] && std::abs(distance) < distance2plane_Threshold)
                {

                    Eigen::Vector3f point_normal_inner;
                    point_normal_inner(0) = normals->points[indice].normal_x;
                    point_normal_inner(1) = normals->points[indice].normal_y;
                    point_normal_inner(2) = normals->points[indice].normal_z;
                    //根据角度和距离进行统计
                    float angle_inner = pcl::getAngle3D(point_normal_inner, point_normal, true);
                    if (angle_inner < parrallelThreshold || (180 - angle_inner) < parrallelThreshold)
                    {
                        vp.indices.push_back(indice);
                        numPoints++;
                    }

                    // 根据曲率和距离进行统计
                    /* float curvatureDiff = std::abs(normals->points[indice].curvature-pointCurvature);
                    if (curvatureDiff < curvatureThreshold)
                    {
                        vp.indices.push_back(indice);
                        numPoints++;
                    } */
                }
            }
            // 面内点数阈值, 面向量夹角阈值,面之间的距离
            // 如果面面夹角和面面距离小于阈值则可能拟合的是同一个平面，此时比较numpoints，多的留下
            // 自定义参数numPointsThreshold，planeAngleThreshold，distancePlane2Plane

            // std::cout << "numPoints in plane: " << numPoints<<std::endl;
            if (numPoints > numPointsThreshold)
            {
                // 用于执行加速,可能降低精度: 效果挺差
                // for(auto i : vp.indices){
                //     processed[i] = true;
                // }
                vp.numPoints = numPoints;

                bool flag = true;
                for (auto it = verticalPlanes.begin(); it != verticalPlanes.end(); it++)
                {
                    Eigen::Vector3f fit_plane_normal;
                    fit_plane_normal << it->coefficient->values[0], it->coefficient->values[1], it->coefficient->values[2];
                    float plane_angle = pcl::getAngle3D(fit_plane_normal, point_normal, true);

                    if (plane_angle < planeAngleThreshold)
                    {
                        // 判断面面之间的距离，此时以δd来近似
                        float distanceP2P = std::abs(it->coefficient->values[3] - coefficientPlane->values[3]);
                        // std::cout << "plane2plane distance: " << distanceP2P <<std::endl;
                        if (std::abs(distanceP2P) < distancePlane2Plane && numPoints > it->numPoints)
                        {
                            verticalPlanes.insert(verticalPlanes.erase(it), vp);
                            // 标记已经添加过
                            flag = false;
                            break;
                        }
                        // 标记应该舍弃
                        flag = false;
                    }
                }
                if (flag)
                {
                    verticalPlanes.push_back(vp);
                }
            };
        }
    }
    // 计时结束
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "vertical plane segment took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::cout << "verticalPlanes size: " << verticalPlanes.size() << std::endl;
    // jumptimes marktimes
    // std::cout << "jumptimes: " << jumptimes << std::endl;
    // std::cout << "marktimes: " << marktimes << std::end
    return verticalPlanes;
}

// 直线拟合
template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::lineSeg(PtCdtr<PointT> cloud, float distance, int maxIteration){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers用来存储直线上点的索引
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);      
    seg.setModelType(pcl::SACMODEL_LINE);   
    seg.setMethodType(pcl::SAC_RANSAC);     
    seg.setDistanceThreshold(distance);        
    seg.setMaxIterations(maxIteration);            
    seg.setInputCloud(cloud);               
    seg.segment(*inliers, *coefficients); 
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = segment_Indices(cloud, inliers);
    return segment_resultPair.first;    
}



