/*
 * @Description:
 * @Author: yurui
 * @Date: 2021-12-17 19:11:52
 * @LastEditTime: 2021-12-27 17:21:50
 * @FilePath: /Lidar/test/test.cpp
 */

#include <set>
#include <algorithm>
#include "processPointClouds.hpp"
#include "render.hpp"
#include "box.h"
#include <vector>

typedef pcl::PointXYZI PointT;

void calcLine(pcl::ModelCoefficients::Ptr coefsOfPlane1, pcl::ModelCoefficients::Ptr coefsOfPlane2, pcl::ModelCoefficients::Ptr coefsOfLine)
{
    //方向向量n=n1×n2=(b1*c2-c1*b2,c1*a2-a1*c2,a1*b2-b1*a2)
    pcl::ModelCoefficients temcoefs;
    double a1, b1, c1, d1, a2, b2, c2, d2;
    double tempy, tempz;
    a1 = coefsOfPlane1->values[0];
    b1 = coefsOfPlane1->values[1];
    c1 = coefsOfPlane1->values[2];
    d1 = coefsOfPlane1->values[3];
    a2 = coefsOfPlane2->values[0];
    b2 = coefsOfPlane2->values[1];
    c2 = coefsOfPlane2->values[2];
    d2 = coefsOfPlane2->values[3];
    tempz = -(d1 / b1 - d2 / b2) / (c1 / b1 - c2 / b2);
    tempy = (-c1 / b1) * tempz - d1 / b1;
    coefsOfLine->values.push_back(0.0);
    coefsOfLine->values.push_back(tempy);
    coefsOfLine->values.push_back(tempz);
    coefsOfLine->values.push_back(b1 * c2 - c1 * b2);
    coefsOfLine->values.push_back(c1 * a2 - a1 * c2);
    coefsOfLine->values.push_back(a1 * b2 - b1 * a2);
}

// 点到线的投影点坐标
pcl::PointXYZ projectPoint2Line(Eigen::Vector4f &point, Eigen::Vector4f &pointOnline, Eigen::Vector3f &lineDirect)
{
    pcl::PointXYZ projectPoint;

    float projectLength = lineDirect(0) * (point(0) - pointOnline(0)) + lineDirect(1) * (point(1) - pointOnline(1)) + lineDirect(2) * (point(2) - pointOnline(2));
    projectPoint.x = pointOnline(0) + projectLength * lineDirect(0);
    projectPoint.y = pointOnline(1) + projectLength * lineDirect(1);
    projectPoint.z = pointOnline(2) + projectLength * lineDirect(2);

    return projectPoint;
}


void detector(pcl::visualization::PCLVisualizer::Ptr viewer, ProcessPointClouds<PointT> *pointProcessorI, PtCdtr<PointT> inputCloudI)
{
    //--------------------------------------------------------------
    // 体素滤波
    float leaf_size = 0.2;
    // ROI滤波
    Eigen::Vector4f minPoint(-30, -30, -1, 1);
    Eigen::Vector4f maxPoint(50, 30, 2, 1);
    // 统计滤波
    int st_nearPoints = 50;
    int stdNums = 1;
    // 平面分割
    int maxIterations = 60;
    float distanceThreshold = 0.5;
    // 欧拉聚类
    std::string cluster_name = "cluster";
    int cluster_id = 0;
    float clusterTolerance = 0.2;
    int minsize = 20;
    int maxsize = 10000;
    // 聚类中在roni中的点数大于cluster_roni_points则删除整个聚类
    int cluster_roni_points = 0;
    // 法向量
    float radius = 0.5;
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
    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients);
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = pointProcessorI->segment_Plane(outputCloudI, maxIterations, distanceThreshold, coefficient);
    PtCdtr<PointT> inPlane_Cloud = segment_resultPair.first;
    PtCdtr<PointT> outPlane_Cloud = segment_resultPair.second;

    // ROI分割
    std::vector<int> roi_indices, roni_indices;
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roiResultPair = pointProcessorI->roi_filter(outPlane_Cloud, minPoint, maxPoint, roi_indices, roni_indices);
    PtCdtr<PointT> regionCloud = roiResultPair.first;
    PtCdtr<PointT> not_regionCloud = roiResultPair.second;

    // 统计滤波 PtCdtr<PointT>
    PtCdtr<PointT> stPointCloud = pointProcessorI->statistic_filter(regionCloud, st_nearPoints, stdNums);

    // 垂直面的拟合
    std::vector<verticalPlane> verticalPlanes = pointProcessorI->verticalPlane_detect(regionCloud, coefficient);

    // 点云显示
    renderCloud<PointT>(viewer, regionCloud, Color(0, 0, 100), "filteredCloud");
    // 垂直面点云显示
    int numPlanes = 0;
    for (auto verticalplane : verticalPlanes)
    {
        pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);
        planeCloud->width = verticalplane.indices.size();
        planeCloud->height = 1;
        for (auto i : verticalplane.indices)
        {
            // 索引存储的是outPlane_Cloud中的索引
            planeCloud->points.push_back(regionCloud->points[i]);
        }
        renderCloud<PointT>(viewer, planeCloud, Color(255, 0, 0), "verticalplanes" + std::to_string(numPlanes));


        // 画线
        pcl::ModelCoefficients::Ptr linecoefficient(new pcl::ModelCoefficients);
        calcLine(verticalplane.coefficient, coefficient, linecoefficient);
        Eigen::Vector3f lineDirect(linecoefficient->values[3], linecoefficient->values[4], linecoefficient->values[5]);
        Eigen::Vector4f line_minPoint;
        Eigen::Vector4f line_maxPoint;
        pcl::getMinMax3D(*planeCloud, line_minPoint, line_maxPoint);
        // 点到线的投影点
        Eigen::Vector4f pointOnline(linecoefficient->values[0],linecoefficient->values[1],linecoefficient->values[2],1);
        pcl::PointXYZ projectMin;
        projectMin = projectPoint2Line(line_minPoint, pointOnline, lineDirect);
        pcl::PointXYZ projectMax;
        projectMax = projectPoint2Line(line_maxPoint, pointOnline, lineDirect);
        viewer->addLine<pcl::PointXYZ>(projectMin,projectMax,"line"+std::to_string(numPlanes));

        numPlanes++;
    }
}

int main()
{
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