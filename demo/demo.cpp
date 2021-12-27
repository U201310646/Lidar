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