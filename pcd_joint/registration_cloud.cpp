/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-17 09:45:24
 * @LastEditTime: 2021-12-18 13:50:37
 * @FilePath: /Lidar/pcd_joint/registration_cloud.cpp
 */

#include <iostream>
#include <boost/filesystem.hpp>
#include "json_read.h"
#include <vector>
#include <pcl-1.11/pcl/io/io.h>
#include <pcl-1.11/pcl/point_cloud.h>
#include <pcl-1.11/pcl/point_types.h>
#include <pcl-1.11/pcl/io/pcd_io.h>
#include <pcl-1.11/pcl/common/transforms.h>
#include <pcl-1.11/pcl/visualization/cloud_viewer.h>
#include <algorithm>
#include <memory>
#include <string>

typedef pcl::PointXYZI PointT;
int main()
{

    Eigen::Matrix4f matrix_front;
    Eigen::Matrix4f matrix_rear;
    Eigen::Matrix4f matrix_left;
    Eigen::Matrix4f matrix_right;
    // 获取世界坐标系到雷达坐标系的各个矩阵。后面拼接点云时需要对矩阵进行求逆
    get_matrix_from_world2lidar(matrix_front, matrix_rear, matrix_left, matrix_right);
    // 前雷达的初始化
    boost::filesystem::path path_front("/home/cyr/Documents/Data/RoboSense_data/sensordata/rs_PointCloud2_20_keyframe");
    boost::filesystem::directory_iterator iter_front(path_front);
    boost::filesystem::directory_iterator iter_end;
    std::vector<boost::filesystem::path> pcd_stream_front(iter_front, iter_end);
    int file_size_front = pcd_stream_front.size();
    std::vector<boost::filesystem::path>::iterator iterator_front = pcd_stream_front.begin();
    pcl::PCDReader reader_front;
    pcl::PointCloud<PointT>::Ptr cloud_front(std::make_shared<pcl::PointCloud<PointT>>());
    pcl::PointCloud<PointT>::Ptr cloud_front_out(std::make_shared<pcl::PointCloud<PointT>>());
    // 后雷达的初始化
    boost::filesystem::path path_rear("/home/cyr/Documents/Data/RoboSense_data/sensordata/rs_PointCloud2_21_keyframe");
    boost::filesystem::directory_iterator iter_rear(path_rear);
    std::vector<boost::filesystem::path> pcd_stream_rear(iter_rear, iter_end);
    int file_size_rear = pcd_stream_rear.size();
    std::vector<boost::filesystem::path>::iterator iterator_rear = pcd_stream_rear.begin();
    pcl::PCDReader reader_rear;
    pcl::PointCloud<PointT>::Ptr cloud_rear(std::make_shared<pcl::PointCloud<PointT>>());
    pcl::PointCloud<PointT>::Ptr cloud_rear_out(std::make_shared<pcl::PointCloud<PointT>>());
    // 左雷达的初始化
    boost::filesystem::path path_left("/home/cyr/Documents/Data/RoboSense_data/sensordata/rs_PointCloud2_22_keyframe");
    boost::filesystem::directory_iterator iter_left(path_left);
    std::vector<boost::filesystem::path> pcd_stream_left(iter_left, iter_end);
    int file_size_left = pcd_stream_left.size();
    std::vector<boost::filesystem::path>::iterator iterator_left = pcd_stream_left.begin();
    pcl::PCDReader reader_left;
    pcl::PointCloud<PointT>::Ptr cloud_left(std::make_shared<pcl::PointCloud<PointT>>());
    pcl::PointCloud<PointT>::Ptr cloud_left_out(std::make_shared<pcl::PointCloud<PointT>>());
    // 右雷达的初始化
    boost::filesystem::path path_right("/home/cyr/Documents/Data/RoboSense_data/sensordata/rs_PointCloud2_23_keyframe");
    boost::filesystem::directory_iterator iter_right(path_right);
    std::vector<boost::filesystem::path> pcd_stream_right(iter_right, iter_end);
    int file_size_right = pcd_stream_right.size();
    std::vector<boost::filesystem::path>::iterator iterator_right = pcd_stream_right.begin();
    pcl::PCDReader reader_right;
    pcl::PointCloud<PointT>::Ptr cloud_right(std::make_shared<pcl::PointCloud<PointT>>());
    pcl::PointCloud<PointT>::Ptr cloud_right_out(std::make_shared<pcl::PointCloud<PointT>>());
    // 拼接点云
    pcl::PointCloud<PointT>::Ptr cloud_x(new pcl::PointCloud<PointT>);

    assert(file_size_front == file_size_rear && file_size_left == file_size_right && file_size_rear == file_size_left);

    int i(0);
    while (iterator_front != pcd_stream_front.end())
    {

        std::string base_dir = "/home/cyr/Documents/Data/RoboSense_data/sensordata/joint_cloud/";
        sort(pcd_stream_front.begin(), pcd_stream_front.end());
        sort(pcd_stream_rear.begin(), pcd_stream_rear.end());
        sort(pcd_stream_left.begin(), pcd_stream_left.end());
        sort(pcd_stream_right.begin(), pcd_stream_right.end());

        reader_front.read<PointT>(iterator_front->string(), *cloud_front);
        reader_rear.read<PointT>(iterator_rear->string(), *cloud_rear);
        reader_left.read<PointT>(iterator_left->string(), *cloud_left);
        reader_right.read<PointT>(iterator_right->string(), *cloud_right);

        pcl::transformPointCloud(*cloud_front, *cloud_front_out, matrix_front.inverse());
        pcl::transformPointCloud(*cloud_rear, *cloud_rear_out, matrix_rear.inverse());
        pcl::transformPointCloud(*cloud_left, *cloud_left_out, matrix_left.inverse());
        pcl::transformPointCloud(*cloud_right, *cloud_right_out, matrix_right.inverse());

        // 拼接的点云
        *cloud_x = *cloud_front_out + *cloud_rear_out + *cloud_left_out + *cloud_right_out;
        pcl::PCDWriter writer;
        std::cout << i << std::endl;
        writer.write(base_dir += (std::to_string(i).append(".pcd")), *cloud_x, false);

        i++;
        iterator_front++;
        iterator_rear++;
        iterator_left++;
        iterator_right++;
    }
}