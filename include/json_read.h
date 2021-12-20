/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-16 13:07:36
 * @LastEditTime: 2021-12-18 14:03:40
 * @FilePath: /Lidar/include/json_read.h
 */
/*read matrix json file*/
#pragma once
#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Core>

Eigen::Matrix4f ReadJsonMatrix(Json::Value& Array)
{
    int arr_size = Array.size();
    assert(arr_size==16);
    float arr[arr_size];
    for(int i = 0; i<arr_size; i++){
        arr[i] = Array[i].asFloat();
    }
    Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> map(arr, 4, 4);
    
    return map;
}


void get_matrix_from_world2lidar(Eigen::Matrix4f& matrix_front, Eigen::Matrix4f& matrix_rear, Eigen::Matrix4f& matrix_left, Eigen::Matrix4f& matrix_right){

    // 转换矩阵雷达外参文件的位置
    std::string matrix_file_front = "/home/cyr/Documents/Data/RoboSense_data/calib/LidarFront.json"; 
    std::string matrix_file_rear = "/home/cyr/Documents/Data/RoboSense_data/calib/LidarRear.json";
    std::string matrix_file_left = "/home/cyr/Documents/Data/RoboSense_data/calib/LidarLeft.json";
    std::string matrix_file_right = "/home/cyr/Documents/Data/RoboSense_data/calib/LidarRight.json";

    Json::Reader reader_front;
    Json::Value value_front;
    std::ifstream ifile_front(matrix_file_front, std::ios::binary);
    if(reader_front.parse(ifile_front ,value_front)){
        std::cout<< "matrix_front lidar(from world coordinate to lidar): "<<std::endl;
        matrix_front = ReadJsonMatrix(value_front["Rt"]["data"]);
        std::cout<< matrix_front<<std::endl;
    }
    
    Json::Reader reader_rear;
    Json::Value value_rear;
    std::ifstream ifile_rear(matrix_file_rear, std::ios::binary);
    if(reader_rear.parse(ifile_rear ,value_rear)){
        std::cout<< "matrix_rear lidar(from world coordinate to lidar): "<<std::endl;
        matrix_rear = ReadJsonMatrix(value_rear["Rt"]["data"]);
        std::cout<< matrix_rear<<std::endl;
    }
    Json::Reader reader_left;
    Json::Value value_left;
    std::ifstream ifile_left(matrix_file_left, std::ios::binary);
    if(reader_left.parse(ifile_left ,value_left)){
        std::cout<< "matrix_left lidar(from world coordinate to lidar): "<<std::endl;
        matrix_left = ReadJsonMatrix(value_left["Rt"]["data"]);
        std::cout<< matrix_left<<std::endl;
    }
    Json::Reader reader_right;
    Json::Value value_right;
    std::ifstream ifile_right(matrix_file_right, std::ios::binary);
    if(reader_right.parse(ifile_right ,value_right)){
        std::cout<< "matrix_right lidar(from world coordinate to lidar): "<<std::endl;
        matrix_right = ReadJsonMatrix(value_right["Rt"]["data"]);
        std::cout<< matrix_right<<std::endl;
    }

}