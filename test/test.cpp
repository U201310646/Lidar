/*
 * @Description:
 * @Author: yurui
 * @Date: 2021-12-17 19:11:52
 * @LastEditTime: 2021-12-27 11:10:00
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
pcl::PointXYZ projectPoint2Line(Eigen::Vector4f& point, Eigen::Vector4f& pointOnline, Eigen::Vector3f& lineDirect){
    pcl::PointXYZ projectPoint;

    float projectLength = lineDirect(0)*(point(0) - pointOnline(0))+lineDirect(1)*(point(1) - pointOnline(1))+lineDirect(2)*(point(2) - pointOnline(2));
    projectPoint.x = pointOnline(0)+projectLength*lineDirect(0);
    projectPoint.y = pointOnline(1)+projectLength*lineDirect(1);
    projectPoint.z = pointOnline(2)+projectLength*lineDirect(2);

    return projectPoint;

}

// 路沿或者栅栏拟合面的参数， 面内的点数， 面内点云的索引
struct verticalPlane
{
    pcl::ModelCoefficients::Ptr coefficient;
    int numPoints;
    std::vector<int> indices;
};

void detector(pcl::visualization::PCLVisualizer::Ptr viewer, ProcessPointClouds<PointT> *pointProcessorI, PtCdtr<PointT> inputCloudI)
{
    //--------------------------------------------------------------
    // 体素滤波
    float leaf_size = 0.2;
    // ROI滤波
    Eigen::Vector4f minPoint(-30, -30, -1, 1);
    Eigen::Vector4f maxPoint(30, 30, 2, 1);
    // 平面分割
    int maxIterations = 40;
    float distanceThreshold = 0.3;
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
    pcl::ModelCoefficients::Ptr coefficient1(new pcl::ModelCoefficients);
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = pointProcessorI->segment_Plane(outputCloudI, maxIterations, distanceThreshold, coefficient1);
    PtCdtr<PointT> inPlane_Cloud = segment_resultPair.first;
    PtCdtr<PointT> outPlane_Cloud = segment_resultPair.second;

    // new平面
    pcl::ModelCoefficients::Ptr coefficient2(new pcl::ModelCoefficients);
    std::vector<float> v1(coefficient1->values.begin(), coefficient1->values.end());
    std::vector<float> v2(v1);
    v2[0] = 2;
    v2[1] = 3;
    v2[2] = -(v1[0] * v2[0] + v1[1] * v2[1]) / v1[2];
    float d = sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);
    v2[0] /= d;
    v2[1] /= d;
    v2[2] /= d;
    v2[3] = 0.5;
    coefficient2->values.assign(v2.begin(), v2.end());

    // ROI分割
    std::vector<int> roi_indices, roni_indices;
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roiResultPair = pointProcessorI->roi_filter(outPlane_Cloud, minPoint, maxPoint, roi_indices, roni_indices);
    PtCdtr<PointT> regionCloud = roiResultPair.first;
    PtCdtr<PointT> not_regionCloud = roiResultPair.second;

    //********************************************************************************************************************************************************
    // 存储最终拟合结果
    std::vector<verticalPlane> verticalPlanes;

    // regionCloud中每个点的法向量,其中存在很多的0向量和nan向量
    pcl::PointCloud<pcl::Normal>::Ptr normals = pointProcessorI->normalExtractor(regionCloud, radius);
    std::cout << "normals size: " << normals->points.size() << std::endl;
    std::cout << "pointcloud size: " << regionCloud->points.size() << std::endl;
    std::cout << "indices size: " << roi_indices.size() << std::endl;

    Eigen::Map<Eigen::Vector3f> plane_normal(&coefficient1->values[0], 3);

    // 标记点数
    int marktimes = 0;
    // 标记点集合初始化
    std::vector<bool> processed(roi_indices.size(), false);
    // for (auto i : roi_indices)
    for (int i = 0; i < roi_indices.size(); i++)
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
    // 遍历点云中的所有点(对点索引进行遍历更方便)
    // for (auto ind : roi_indices)
    for (int ind = 0; ind < roi_indices.size(); ind++)
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
        point_normal(0) = normals->points[ind].normal_x;
        point_normal(1) = normals->points[ind].normal_y;
        point_normal(2) = normals->points[ind].normal_z;
        // 计算点法向量与面向量夹角(0~180),只处理法向量与地面垂直的点
        float angle = pcl::getAngle3D(plane_normal, point_normal, true);

        // std::cout <<"法向量和面的夹角: "<< angle <<std::endl;
        // 自定义参数
        float verticalThreshold = 5;
        float distance2plane_Threshold = 0.3;
        if (std::abs(angle - 90) < verticalThreshold)
        {
            pcl::PointXYZ point;
            point.x = regionCloud->points[ind].x;
            point.y = regionCloud->points[ind].y;
            point.z = regionCloud->points[ind].z;
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
            // for (auto indice : roi_indices)
            for (int indice = 0; indice < roi_indices.size(); indice++)
            {
                float distance = point_normal(0) * regionCloud->points[indice].x + point_normal(1) * regionCloud->points[indice].y + point_normal(2) * regionCloud->points[indice].z + d;
                // std::cout<< "distance: "<<std::abs(distance) << std::endl;
                // 自定义参数
                float parrallelThreshold = 20;
                if (!processed[indice] && std::abs(distance) < distance2plane_Threshold)
                {

                    Eigen::Vector3f point_normal_inner;
                    point_normal_inner(0) = normals->points[indice].normal_x;
                    point_normal_inner(1) = normals->points[indice].normal_y;
                    point_normal_inner(2) = normals->points[indice].normal_z;
                    float angle_inner = pcl::getAngle3D(point_normal_inner, point_normal, true);
                    if (angle_inner < parrallelThreshold || (180 - angle_inner) < parrallelThreshold)
                    {
                        vp.indices.push_back(roi_indices[indice]);
                        numPoints++;
                    }
                }
            }
            // 面内点数阈值, 面向量夹角阈值,面之间的距离
            // 如果面面夹角和面面距离小于阈值则可能拟合的是同一个平面，此时比较numpoints，多的留下
            // 自定义参数
            int numPointsThreshold = 200;
            float planeAngleThreshold = 20;
            int distancePlane2Plane = 10;

            // std::cout << "numPoints in plane: " << numPoints<<std::endl;
            if (numPoints > numPointsThreshold)
            {
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
    std::cout << "jumptimes: " << jumptimes << std::endl;
    std::cout << "marktimes: " << marktimes << std::endl;

    int numPlanes = 0;
    for (auto verticalplane : verticalPlanes)
    {
        // std::cout << "d: " <<verticalplane.coefficient->values[3] << std::endl;
        viewer->addPlane(*verticalplane.coefficient, "verticalPlane" + std::to_string(numPlanes));
        pcl::ModelCoefficients::Ptr linecoefficient(new pcl::ModelCoefficients);
        calcLine(verticalplane.coefficient, coefficient1, linecoefficient);
        Eigen::Vector3f lineDirect(linecoefficient->values[3], linecoefficient->values[4], linecoefficient->values[5]);

        pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);
        planeCloud->width = verticalplane.indices.size();
        planeCloud->height = 1;
        for (auto i : verticalplane.indices)
        {
            planeCloud->points.push_back(regionCloud->points[i]);
        }
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
    //********************************************************************************************************************************************************
    // 根据面的参数拟合平面
    viewer->addPlane(*coefficient1, "plane1");
    viewer->addPlane(*coefficient2, "plane2");
    // 平面的交线
    pcl::ModelCoefficients::Ptr coefficient3(new pcl::ModelCoefficients);
    calcLine(coefficient1, coefficient2, coefficient3);
    viewer->addLine(*coefficient3, "line");

    // 点云显示
    renderCloud<PointT>(viewer, regionCloud, Color(0, 0, 100), "filteredCloud");
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
    stream_iter;
    PtCdtr<PointT> inputCloudI = pointProcessorI->loadPcd((*stream_iter).string());
    detector(viewer, pointProcessorI, inputCloudI);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}