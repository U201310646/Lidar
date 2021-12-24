/*
 * @Description:
 * @Author: yurui
 * @Date: 2021-12-17 19:11:52
 * @LastEditTime: 2021-12-24 14:32:07
 * @FilePath: /Lidar/test/test.cpp
 */

#include <set>
#include <algorithm>
#include "processPointClouds.hpp"
#include "render.hpp"
#include "box.h"

typedef pcl::PointXYZI PointT;

void calcLine(pcl::ModelCoefficients::Ptr coefsOfPlane1, pcl::ModelCoefficients::Ptr coefsOfPlane2, pcl::ModelCoefficients::Ptr coefsOfLine)
{
    //方向向量n=n1×n2=(b1*c2-c1*b2,c1*a2-a1*c2,a1*b2-b1*a2)
    pcl::ModelCoefficients temcoefs;
    double a1, b1, c1, d1, a2,b2, c2, d2;
    double tempy, tempz;
    a1= coefsOfPlane1->values[0];
    b1= coefsOfPlane1->values[1];
    c1= coefsOfPlane1->values[2];
    d1= coefsOfPlane1->values[3];
    a2= coefsOfPlane2->values[0];
    b2= coefsOfPlane2->values[1];
    c2= coefsOfPlane2->values[2];
    d2= coefsOfPlane2->values[3];
    tempz= -(d1 / b1 - d2 / b2) / (c1 / b1 - c2 / b2);
    tempy= (-c1 / b1)*tempz - d1 / b1;
    coefsOfLine->values.push_back(0.0);
    coefsOfLine->values.push_back(tempy);
    coefsOfLine->values.push_back(tempz);
    coefsOfLine->values.push_back(b1*c2 - c1*b2);
    coefsOfLine->values.push_back(c1*a2 - a1*c2);
    coefsOfLine->values.push_back(a1*b2 - b1*a2);
}


void detector(pcl::visualization::PCLVisualizer::Ptr viewer, ProcessPointClouds<PointT> *pointProcessorI, PtCdtr<PointT> inputCloudI){    
    //--------------------------------------------------------------
    // 体素滤波
    float leaf_size = 0.2;
    // ROI滤波
    Eigen::Vector4f minPoint(-50, -50, -1, 1);
    Eigen::Vector4f maxPoint(50, 50, 2, 1);
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
    pcl::ModelCoefficients::Ptr coefficient1(new pcl::ModelCoefficients);
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = pointProcessorI->segment_Plane(outputCloudI, maxIterations, distanceThreshold, coefficient1);
    PtCdtr<PointT> inPlane_Cloud = segment_resultPair.first;
    PtCdtr<PointT> outPlane_Cloud = segment_resultPair.second;
    std:: cerr << "size: "<<coefficient1->values.size()<<std::endl;
    for(auto i : coefficient1->values){
        std::cerr << i << std::endl;
    }

    // new平面
    pcl::ModelCoefficients::Ptr coefficient2(new pcl::ModelCoefficients);
    std::vector<float> v1(coefficient1->values.begin(), coefficient1->values.end());
    std::vector<float> v2(v1);
    v2[0] = 2; v2[1] = 3;
    v2[2] = -(v1[0]*v2[0] + v1[1]*v2[1])/v1[2];
    float d = sqrt(v2[0]*v2[0] + v2[1]*v2[1]+v2[2]*v2[2]);
    v2[0] /= d; v2[1] /= d;; v2[2] /= d;
    v2[3] = 0.5;
    coefficient2->values.assign(v2.begin(), v2.end());
    for(auto i : coefficient2->values){
        std::cerr << i << std::endl;
    }

    // ROI分割
    std::vector<int> roi_indices, roni_indices;
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roiResultPair = pointProcessorI->roi_filter(outPlane_Cloud, minPoint, maxPoint, roi_indices, roni_indices);
    PtCdtr<PointT> regionCloud = roiResultPair.first;
    PtCdtr<PointT> not_regionCloud = roiResultPair.second;
    
    // 根据面的参数拟合平面
    viewer->addPlane(*coefficient1, "plane1");
    viewer->addPlane(*coefficient2, "plane2");
    // 平面的交线
    pcl::ModelCoefficients::Ptr coefficient3(new pcl::ModelCoefficients);
    calcLine(coefficient1, coefficient2, coefficient3);
    viewer->addLine(*coefficient3,"line");
    
    // 点云显示
    renderCloud<PointT>(viewer, regionCloud, Color(0,0,100), "filteredCloud");
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

    PtCdtr<PointT> inputCloudI = pointProcessorI->loadPcd((*stream_iter).string());
    detector(viewer, pointProcessorI, inputCloudI);

    while (!viewer->wasStopped())
    {   
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    
    }  
}