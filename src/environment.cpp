/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-24 10:24:56
 * @LastEditTime: 2021-12-24 15:34:36
 * @FilePath: /Lidar/src/environment.cpp
 */

#include <set>
#include <algorithm>
#include "processPointClouds.hpp"
#include "render.hpp"
#include "box.h"

typedef pcl::PointXYZI PointT;


void detector(pcl::visualization::PCLVisualizer::Ptr viewer, ProcessPointClouds<PointT> *pointProcessorI, PtCdtr<PointT> inputCloudI){    
    //--------------------------------------------------------------
    // 体素滤波
    float leaf_size = 0.2;
    // ROI滤波
    Eigen::Vector4f minPoint(-50, -20, -3, 1);
    Eigen::Vector4f maxPoint(50, 20, 6, 1);
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
    // 法向量,领域点搜索
    float radius;
    // 区域生长聚类
    int nearPoints = 30;
    int minsize_rg = 10;
    int maxsize_rg = 1000;
    float SmoothnessThreshold = 30;
    float CurvatureThreshold = 0.02;
    //--------------------------------------------------------------
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
    // roi的索引和非roi的索引
    std::vector<int> roi_indices, roni_indices;
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roiResultPair = pointProcessorI->roi_filter(outPlane_Cloud, minPoint, maxPoint, roi_indices, roni_indices);
    PtCdtr<PointT> regionCloud = roiResultPair.first;
    PtCdtr<PointT> not_regionCloud = roiResultPair.second;
    // 聚类中cluster_ind中如果存在某个索引对象中包含非roi中roni_indices的点，则舍弃整个索引对象
    pointProcessorI->clusterFilter(cluster_ind, roni_indices, cluster_roni_points);
    std::vector<PtCdtr<PointT>> clusters = pointProcessorI->ind2cloud(outPlane_Cloud, cluster_ind);

    //区域生长聚类
    /* pcl::PointCloud<pcl::Normal>::Ptr normals = pointProcessorI->normalExtractor(outPlane_Cloud, radius);
    std::vector<pcl::PointIndices> clusterInd_rg = pointProcessorI->regionGrown(outPlane_Cloud, nearPoints, minsize_rg, maxsize_rg, CurvatureThreshold, SmoothnessThreshold, normals);
    std::vector<PtCdtr<PointT>> clusters_rg = pointProcessorI->ind2cloud(outPlane_Cloud, clusterInd_rg); */

    // 点云显示
    renderCloud<PointT>(viewer, outPlane_Cloud, Color(0,0,100), "filteredCloud");
    // renderClusters<PointT>(viewer, clusters, Color(255,0,0), "clusters");
    renderClusterBox<PointT>(viewer, pointProcessorI, clusters, Color(255,0,0), "clusterBox");
}

int main(){

    // 显示点云
    std::cerr << "starting enviroment" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer = std::make_shared<pcl::visualization::PCLVisualizer>("display");
    // 相机位置参数
    Eigen::VectorXf posCamera(6);
    posCamera << 2.30212, -34.4712, 42.2699, -0.000115849, 0.786469, 0.617629;
    initCamera(viewer, posCamera);
    // 加载拼接点云文件
    ProcessPointClouds<PointT> *pointProcessorI = new ProcessPointClouds<PointT>();
    // 拼接点云的文件路径
    std::string joint_pcd_file = "/home/cyr/Documents/Data/RoboSense_data/sensordata/joint_cloud";

    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(joint_pcd_file);
    std::vector<boost::filesystem::path>::iterator stream_iter = stream.begin();


    while (!viewer->wasStopped())
    {   
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        PtCdtr<PointT> inputCloudI = pointProcessorI->loadPcd((*stream_iter).string());
        detector(viewer, pointProcessorI, inputCloudI);
        stream_iter++;
        if (stream_iter == stream.end()) {
            stream_iter = stream.begin();
        }
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}