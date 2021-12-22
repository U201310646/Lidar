/*
 * @Description: 
 * @Author: yurui
 * @Date: 2021-12-20 16:56:30
 * @LastEditTime: 2021-12-21 16:42:40
 * @FilePath: /Lidar/test/test.cpp
 */
#include<iostream>
#include<vector>
#include<set>
#include<algorithm>
using namespace std;

/* int main(){
    vector<int> v1{1,2,3,3,4,4,5};
    vector<int> v2{4,4,4,5,6,7,8};
    
    vector<vector<int>> vv1;
    for(int i =0; i<4; i++){
        vv1.push_back(v1);
    }
    vv1.push_back(vector<int>{9,10,11,12,13,12,12});

    vector<vector<vector<int>>::iterator> v_itr;
    multiset<int> s2(v2.begin(), v2.end());
    for(auto it = vv1.begin(); it != vv1.end(); it++){
        cerr << it->size() << endl;
        multiset<int> s1(it->begin(), it->end());
        cerr << s1.size()<<endl;
        multiset<int> s;
        set_intersection(s1.begin(),s1.end(), s2.begin(), s2.end(), inserter(s, s.begin()));
        cerr<< s.size()<<endl;

        if(s.size()>0){
            v_itr.push_back(it);
        }
    }
    for(auto i : v_itr){
        vv1.erase(i);
    }
    
    cout<<vv1.size()<<endl;
    
} */

#include <set>
#include <algorithm>
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
    Eigen::Vector4f minPoint(-50, -50, -3, 1);
    Eigen::Vector4f maxPoint(50, 50, 6, 1);
    //平面分割
    int maxIterations = 40; 
    float distanceThreshold = 0.1;
    //欧拉聚类
    std::string cluster_name = "cluster";
    int cluster_id = 0;
    float clusterTolerance = 0.5;
    int minsize = 10;
    int maxsize = 1000;
    // 聚类中在roni中的点数大于cluster_roni_points则删除整个聚类
    int cluster_roni_points = 0;
    //##############################################
    // 点云预处理：ROI滤波,体素就近滤波
    std::cerr << "before filtering" << std::endl;
    pointProcessorI->numPoints(inputCloudI);
    //! 先不使用ROI滤波
    // std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roiResultPair = pointProcessorI->roi_filter(inputCloudI, minPoint, maxPoint);
    // PtCdtr<PointT> regionCloud = roiResultPair.first;
    PtCdtr<PointT> outputCloudI = pointProcessorI->voxel_filter(inputCloudI, leaf_size);
    std::cerr << "after filtering" << std::endl;
    pointProcessorI->numPoints(outputCloudI);
    // 点云平面分割
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> segment_resultPair = pointProcessorI->segment_Plane(outputCloudI,maxIterations,distanceThreshold);
    PtCdtr<PointT> inPlane_Cloud = segment_resultPair.first;
    PtCdtr<PointT> outPlane_Cloud = segment_resultPair.second;
    // 点云聚类
    // 聚类中的索引
    std::vector<pcl::PointIndices> cluster_ind = pointProcessorI->cluster(outPlane_Cloud, clusterTolerance, minsize, maxsize);
    std::vector<PtCdtr<PointT>> clusters = pointProcessorI->ind2cloud(outPlane_Cloud, cluster_ind);
    std::cerr<< "clusters number: " << clusters.size()<<endl;
    // std::cerr << "cluster_ind: "<< cluster_ind.size()<<std::endl;

    std::vector<int> totalIndices_cluster;
    int count = 0;
    for(auto clusi : cluster_ind){
        count += clusi.indices.size();
        totalIndices_cluster.insert(totalIndices_cluster.end(),clusi.indices.begin(), clusi.indices.end());
    }
    std::cerr << "clusi_size: " << count<<std::endl;
    std::cerr << "total_clusiIndices size: " << totalIndices_cluster.size()<< std::endl;


    // roi的索引和非roi的索引
    std::vector<int> roi_indices, roni_indices;
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> roiResultPair = pointProcessorI->roi_filter(outPlane_Cloud, minPoint, maxPoint, roi_indices, roni_indices);
    PtCdtr<PointT> regionCloud = roiResultPair.first;
    PtCdtr<PointT> not_regionCloud = roiResultPair.second;
    std::cerr << "roi_indices: "<< roi_indices.size()<<std::endl;
    std::cerr << "roni_indices: "<< roni_indices.size()<<std::endl;

    std::vector<int> outPlane_indices(roni_indices.begin(), roni_indices.end());
    outPlane_indices.insert(outPlane_indices.end(), roi_indices.begin(), roi_indices.end());
    std::cerr << "outPlane_indices size: " <<outPlane_indices.size()<< std::endl;

    int inner_count = 0;
    for(auto i : totalIndices_cluster){
        std::vector<int>::iterator it = std::find(outPlane_indices.begin(), outPlane_indices.end(), i);
        if(it != outPlane_indices.end()){
            inner_count ++;
            // outPlane_indices.erase(it);
        }
    }
    std::cerr << "insect size: "<<inner_count<<std::endl;

    // 以下出问题！！？？
    // std::set<int> clust_set_indices(totalIndices_cluster.begin(), totalIndices_cluster.end());
    // std::set<int> outPlane_set_indices(outPlane_indices.begin(), outPlane_indices.end());
    // std::cerr << "clust_set_indices size: "<<clust_set_indices.size() << std::endl;
    // std::cerr << "outPlane_set_indices size: "<<outPlane_set_indices.size() << std::endl;
    // std::set<int> set_insect;
    // set_intersection(clust_set_indices.begin(), clust_set_indices.end(), outPlane_indices.begin(), outPlane_indices.end(), std::inserter(set_insect,set_insect.begin()));
    // std::cerr << "insect size: "<<set_insect.size()<<std::endl;

    
    std::sort(totalIndices_cluster.begin(), totalIndices_cluster.end());
    std::cerr << totalIndices_cluster[0] << " "<< totalIndices_cluster[1]<< " "<< totalIndices_cluster[2]<< " "<< totalIndices_cluster[3]<< " "<< totalIndices_cluster[4]<< " "<< totalIndices_cluster[5]<< " "<< totalIndices_cluster[6]<< " "<< totalIndices_cluster[7]<< " "<< totalIndices_cluster[10]<<std::endl;
    std::sort(outPlane_indices.begin(),outPlane_indices.end());
    std::vector<int> vector_insect;
    vector_insect.resize(min(totalIndices_cluster.size(), outPlane_indices.size()));
    vector<int>::iterator itEnd = std::set_intersection(totalIndices_cluster.begin(), totalIndices_cluster.end(),outPlane_indices.begin(),outPlane_indices.end(),vector_insect.begin());
    int insect_count = 0;
    for(auto it = vector_insect.begin(); it != itEnd; it++){
        insect_count++;
    };
    std::cerr<< "insect size: "<<insect_count << std::endl;

    /* // 聚类中cluster_ind中如果存在某个索引对象中包含非roi中roni_indices的点，则舍弃整个索引对象
    std::vector<std::vector<pcl::PointIndices>::iterator> position;
    std::set<int> s_roni(roni_indices.begin(), roni_indices.end());
    std::cerr<<"roni_set: "<<s_roni.size()<<std::endl;
    for(auto it = cluster_ind.begin();it != cluster_ind.end(); it++){
        std::set<int> s_cluster(it->indices.begin(), it->indices.end());
        std::set<int> s_insect;
        std::set_intersection(s_cluster.begin(), s_cluster.end(), s_roni.begin(), s_roni.end(), std::inserter(s_insect, s_insect.begin()));
        if(s_insect.size()>cluster_roni_points){
            position.push_back(it);
        }
    }
    for(auto pos: position){
        cluster_ind.erase(pos);
    }
    clusters.clear();
    clusters = pointProcessorI->ind2cloud(outPlane_Cloud, cluster_ind);
    std::cerr<< "clusters number: " << clusters.size()<<endl; */


    // 设置颜色显示
    // pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(inPlane_Cloud, 255, 0, 0);
    // viewer->addPointCloud<PointT>(inPlane_Cloud, rgb, "sample cloud");

    /* int v1(0), v2(0);
    viewer->createViewPort(0.5,0,1,1,v1);
    viewer->createViewPort(0,0,0.5,1,v2);
    
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> intensity_distribution(regionCloud, "intensity");
    viewer->addPointCloud<PointT>(regionCloud, intensity_distribution,"regionCloud",v1);
    // 根据强度自动调节颜色
    for(auto cluster_i: clusters){
        // pcl::visualization::PointCloudColorHandlerGenericField<PointT> intensity_distribution(cluster_i,"intensity");
        viewer->addPointCloud(cluster_i, intensity_distribution, cluster_name+std::to_string(cluster_id),v2);
        cluster_id ++;
    }
    // viewer->addPointCloud<PointT>(inPlane_Cloud, intensity_distribution1,"plane_cloud",v2);

    


    while(!viewer->wasStopped()){
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));   
    } */
}