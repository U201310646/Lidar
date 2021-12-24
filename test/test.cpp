/*
 * @Description:
 * @Author: yurui
 * @Date: 2021-12-17 19:11:52
 * @LastEditTime: 2021-12-24 10:21:25
 * @FilePath: /Lidar/test/test.cpp
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
    Eigen::Vector4f minPoint(-50, -20, -1, 1);
    Eigen::Vector4f maxPoint(50, 20, 1, 1);
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

    // 点云显示
    renderCloud<PointT>(viewer, outPlane_Cloud, Color(0,0,100), "filteredCloud");
    // renderClusters<PointT>(viewer, clusters, Color(255,0,0), "clusters");
    // renderClusterBox<PointT>(viewer, pointProcessorI, clusters, Color(255,0,0), "clusterBox");
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