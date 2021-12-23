/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-22 16:42:55
 * @LastEditTime: 2021-12-22 20:09:45
 * @FilePath: /Lidar/src/box.cpp
 */

#include "box.h"

void boxRender(pcl::visualization::PCLVisualizer::Ptr viewer, int cluster_id, Box box, int v){
    if (box.volume<40 && box.z_min<1 && box.height>1){
    std::string cube = "box" + std::to_string(cluster_id);
        viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, 255, 0, 0, cube, v);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, cube);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, cube);}
}
