/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-17 19:11:52
 * @LastEditTime: 2021-12-20 13:49:27
 * @FilePath: /Lidar/src/environment.cpp
 */

#include "processPointClouds.hpp"

typedef pcl::PointXYZI PointT;
int main(){
    std::cerr << "starting enviroment"<< std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer = std::make_shared<pcl::visualization::PCLVisualizer>();
    viewer->setBackgroundColor(0,0,0);
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(1.0);
    // pcl::visualization::Camera中获取的相机pos,view参数
    viewer->setCameraPosition(2.30212,-34.4712,42.2699,-0.000115849,0.786469,0.617629);
    
    //********************************************************************************
    // 拼接矩阵的文件位置
    //! 只是用一个拼接pcd文件进行试验
    std::string joint_pcd_file = "/home/cyr/Documents/Data/RoboSense_data/sensordata/joint_cloud";
    ProcessPointClouds<PointT> *pointProcessorI =  new ProcessPointClouds<PointT>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(joint_pcd_file);
    std::vector<boost::filesystem::path>::iterator stream_iter = stream.begin();
    PtCdtr<PointT> inputCloudI;
    inputCloudI = pointProcessorI->loadPcd((*stream_iter).string());

    //todo 滤波，分割，聚类, 拼接点云重合处的滤波
    //##############################################
    //自定义参数
    //体素滤波
    float leaf_size  = 0.2;
    //ROI滤波
    Eigen::Vector4f minPoint(-51.2, -51.2, -3, 1);
    Eigen::Vector4f maxPoint(51.2, 51.2, 6, 1);
    //平面分割
    int maxIterations = 40; 
    float distanceThreshold = 0.1;
    //##############################################
    // 点云预处理：体素就近滤波, ROI滤波
    std::cerr << "before filtering" << std::endl;
    pointProcessorI->numPoints(inputCloudI);
    PtCdtr<PointT> outputCloudI = pointProcessorI->cloud_filter(inputCloudI,leaf_size, minPoint, maxPoint);
    std::cerr << "after filtering" << std::endl;
    pointProcessorI->numPoints(outputCloudI);
    // 点云平面分割
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = pointProcessorI->segment_Plane(outputCloudI,maxIterations,distanceThreshold);
    PtCdtr<PointT> inPlane_Cloud = segment_resultPair.first;
    // plane_vector.push_back(inPlane_Cloud);
    PtCdtr<PointT> outPlane_Cloud = segment_resultPair.second;
    

     // 设置颜色显示
    // pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(inPlane_Cloud, 255, 0, 0);
    // viewer->addPointCloud<PointT>(inPlane_Cloud, rgb, "sample cloud");

    // 根据强度自动调节颜色
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> intensity_distribution(inPlane_Cloud,"intensity");
    viewer->addPointCloud<PointT>(inPlane_Cloud, intensity_distribution);

    while(!viewer->wasStopped()){
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));   
    }
}