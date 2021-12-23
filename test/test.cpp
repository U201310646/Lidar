/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-20 16:56:30
 * @LastEditTime: 2021-12-23 11:16:36
 * @FilePath: /Lidar/test/test.cpp
 */

#include <eigen3/Eigen/Dense>
#include <iostream>

int main(){
    Eigen::VectorXf v(6);
    v << 1,2,3,4,5,6;
    std::cout << v(0) << std::endl;
    
}

