/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-24 15:57:15
 * @LastEditTime: 2021-12-27 10:39:23
 * @FilePath: /Lidar/demo/demo.cpp
 */

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <vector>

using namespace std;


int main(){
    Eigen::Vector3f v{1,2,3};
    vector<float> vv(v.data(), v.data()+3);
    for(auto i: vv){
        cout<<i<<endl;
    }
    
}
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
        vp.indices.clear();
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
        float verticalThreshold = 10;
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
            std::vector<int> index;

            // for (auto indice : roi_indices)
            for (int indice = 0; indice < roi_indices.size(); indice++)
            {
                float distance = point_normal(0) * regionCloud->points[indice].x + point_normal(1) * regionCloud->points[indice].y + point_normal(2) * regionCloud->points[indice].z + d;
                // std::cout<< "distance: "<<std::abs(distance) << std::endl;
                // 自定义参数
                float parrallelThreshold = 20;
                float curvatureThreshold = 0.05;

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
                        vp.indices.push_back(roi_indices[indice]);
                        index.push_back(indice);
                        numPoints++;
                    }

                    // 根据曲率和距离进行统计
                    /* float curvatureDiff = std::abs(normals->points[indice].curvature-pointCurvature);
                    if (curvatureDiff < curvatureThreshold)
                    {
                        vp.indices.push_back(roi_indices[indice]);
                        numPoints++;
                    } */
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
                // for(auto i : index){
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
    // std::cout << "marktimes: " << marktimes << std::endl;

    //********************************************************************************************************************************************************