#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h> 

typedef pcl::PointXYZI PointT;

int main(int argc, char **argv)
{

    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(3.0); 
    viewer.initCameraParameters();
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile("/home/cyr/Documents/Data/RoboSense_data/sensordata/joint_cloud/0.pcd", *cloud);  
    // 设定camera视角，以保持固定视角显示
    pcl::visualization::Camera camera;
    viewer.setCameraPosition(2.30212,-34.4712,42.2699,-0.000115849,0.786469,0.617629);
    // 设置显示颜色
    // pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud, 255, 0, 0);
    // viewer.addPointCloud<PointT>(cloud, rgb, "sample cloud");
    // 以光强进行颜色分配
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> intensity_distribution(cloud,"intensity");
    viewer.addPointCloud<PointT>(cloud, intensity_distribution, "sample cloud");

    while (!viewer.wasStopped())
    {
        viewer.getCameraParameters(camera);
        viewer.spinOnce(10);
        std::cout<< camera.pos[0] << "," <<  camera.pos[1] << "," << camera.pos[2]<<std::endl;
        std::cout<< camera.view[0] << "," << camera.view[1] << "," << camera.view[2]<<std::endl;
    }
    return 0;}