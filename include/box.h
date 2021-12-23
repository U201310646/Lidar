/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-22 16:05:19
 * @LastEditTime: 2021-12-22 19:24:46
 * @FilePath: /Lidar/include/box.h
 */
#pragma once
#include <pcl-1.11/pcl/visualization/pcl_visualizer.h>
#include <string>

struct Box
{
	void getBox(){
		height = z_max - z_min;
		width = (y_max - y_min)<(x_max-x_min)?(y_max - y_min):(x_max-x_min);
		length = (y_max - y_min)>(x_max-x_min)?(y_max - y_min):(x_max-x_min);
		volume = height*width*length;
		area = 2*(height*width + height*length+ width*length);
	}

	void printBoxInfo(){
		std::cerr << "min_x: "<<x_min<< " min_y: "<<y_min<<" min_z: "<<z_min<<std::endl;
		std::cerr << "max_x: "<<x_max<< " max_y: "<<y_max<<" max_z: "<<z_max<<std::endl;
		std::cerr << "height: "<<height<< " width: "<<width<<" length: "<<length<<std::endl;
		std::cerr << "volume: "<<volume<< " area: "<<area<<std::endl;
	}
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;

	float height;
	float width;
	float length;
	float area;
	float volume;
};

void boxRender(pcl::visualization::PCLVisualizer::Ptr viewer, int cluster_id, Box box, int v);