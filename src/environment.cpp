/*
 * @Description:
 * @Author: yurui
 * @Date: 2021-12-17 19:11:52
 * @LastEditTime: 2021-12-23 11:27:08
 * @FilePath: /Lidar/src/environment.cpp
 */

#include <set>
#include <algorithm>
#include "processPointClouds.hpp"
#include "render.h"
#include "box.h"

typedef pcl::PointXYZI PointT;

int main()
{   
    //--------------------------------------------------------------
    // 自定义参数
    std::string joint_pcd_file = "/home/cyr/Documents/Data/RoboSense_data/sensordata/joint_cloud";
    // 相机位置参数
    Eigen::VectorXf posCamera(6);
    posCamera << 2.30212, -34.4712, 42.2699, -0.000115849, 0.786469, 0.617629;
    // 体素滤波
    float leaf_size = 0.2;
    // ROI滤波
    Eigen::Vector4f minPoint(-50, -30, -2, 1);
    Eigen::Vector4f maxPoint(50, 30, 4, 1);
    // 平面分割
    int maxIterations = 40;
    float distanceThreshold = 0.3;
    // 欧拉聚类
    std::string cluster_name = "cluster";
    int cluster_id = 0;
    float clusterTolerance = 0.5;
    int minsize = 20;
    int maxsize = 10000;
    // 聚类中在roni中的点数大于cluster_roni_points则删除整个聚类
    int cluster_roni_points = 0;
    // 法向量
    float radius = 0.05;
    // 区域生长聚类
    int nearPoints = 30;
    int minsize_rg = 10;
    int maxsize_rg = 1000;
    float SmoothnessThreshold = 30;
    float CurvatureThreshold = 0.02;
    //--------------------------------------------------------------
    // 加载拼接点云文件
    ProcessPointClouds<PointT> *pointProcessorI = new ProcessPointClouds<PointT>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(joint_pcd_file);
    std::vector<boost::filesystem::path>::iterator stream_iter = stream.begin();
    PtCdtr<PointT> inputCloudI = pointProcessorI->loadPcd((*stream_iter).string());
    // 点云预处理：ROI滤波,体素就近滤波
    std::cerr << "before filtering" << std::endl;
    pointProcessorI->numPoints(inputCloudI);
    PtCdtr<PointT> outputCloudI = pointProcessorI->voxel_filter(inputCloudI, leaf_size);
    std::cerr << "after filtering" << std::endl;
    pointProcessorI->numPoints(outputCloudI);
    // 点云平面分割
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = pointProcessorI->segment_Plane(outputCloudI, maxIterations, distanceThreshold);
    PtCdtr<PointT> inPlane_Cloud = segment_resultPair.first;
    PtCdtr<PointT> outPlane_Cloud = segment_resultPair.second;
    // 点云聚类
    // 聚类中的索引
    std::vector<pcl::PointIndices> cluster_ind = pointProcessorI->cluster(outPlane_Cloud, clusterTolerance, minsize, maxsize);
    std::cerr << "cluster size: " << cluster_ind.size() << endl;
    // roi的索引和非roi的索引
    std::vector<int> roi_indices, roni_indices;
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roiResultPair = pointProcessorI->roi_filter(outPlane_Cloud, minPoint, maxPoint, roi_indices, roni_indices);
    PtCdtr<PointT> regionCloud = roiResultPair.first;
    PtCdtr<PointT> not_regionCloud = roiResultPair.second;
    // 聚类中cluster_ind中如果存在某个索引对象中包含非roi中roni_indices的点，则舍弃整个索引对象
    pointProcessorI->clusterFilter(cluster_ind, roni_indices, cluster_roni_points);
    std::vector<PtCdtr<PointT>> clusters = pointProcessorI->ind2cloud(outPlane_Cloud, cluster_ind);
    std::cerr << "cluster size: " << clusters.size() << endl;
    //区域生长聚类
    pcl::PointCloud<pcl::Normal>::Ptr normals = pointProcessorI->normalExtractor(outPlane_Cloud, radius);
    std::vector<pcl::PointIndices> clusterInd_rg = pointProcessorI->regionGrown(outPlane_Cloud, nearPoints, minsize_rg, maxsize_rg, CurvatureThreshold, SmoothnessThreshold, normals);
    std::vector<PtCdtr<PointT>> clusters_rg = pointProcessorI->ind2cloud(outPlane_Cloud, clusterInd_rg);
    std::cerr << "cluster_rg size: " << clusters_rg.size() << endl;

    std::cerr << "starting enviroment" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer = std::make_shared<pcl::visualization::PCLVisualizer>("display");
    initCamera(viewer, posCamera);
    
    // 设置颜色显示
    // pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(inPlane_Cloud, 255, 0, 0);
    // viewer->addPointCloud<PointT>(inPlane_Cloud, rgb, "sample cloud");

    int v1(0), v2(0), v3(0);
    // viewer->createViewPort(0, 0, 0.5, 0.5, v1);
    // viewer->createViewPort(0, 0.5, 0.5, 1, v2);
    // viewer->createViewPort(0.5, 0, 1, 1, v3);

    // pcl::visualization::PointCloudColorHandlerGenericField<PointT> intensity_distribution(regionCloud, "intensity");
    // viewer->addPointCloud<PointT>(regionCloud, intensity_distribution, "regionCloud", v3);
    // viewer->addPointCloud<PointT>(inPlane_Cloud, intensity_distribution,"plane_cloud",v3);

    // 根据强度自动调节颜色
    for (auto cluster_i : clusters)
    {
        pcl::visualization::PointCloudColorHandlerGenericField<PointT> intensity_distribution(cluster_i,"intensity");
        Box box = pointProcessorI->boundingBox(cluster_i);
        box.getBox();
        // box.printBoxInfo();
        boxRender(viewer, cluster_id, box, v1);
        viewer->addPointCloud(cluster_i, intensity_distribution, cluster_name + std::to_string(cluster_id));
        cluster_id++;
    }
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(outputCloudI, 0, 0, 100);
    viewer->addPointCloud<PointT>(outputCloudI, rgb,"plane_cloud");


    // // 根据强度自动调节颜色
    // for (auto cluster_i : clusters_rg)
    // {
    //     // pcl::visualization::PointCloudColorHandlerGenericField<PointT> intensity_distribution(cluster_i,"intensity");
    //     viewer->addPointCloud(cluster_i, intensity_distribution, cluster_name + std::to_string(cluster_id), v2);
    //     cluster_id++;
    // }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));

    }
    
  
}