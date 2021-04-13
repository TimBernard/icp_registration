#ifndef POINTCLOUD_VIEWER_HPP
#define POINTCLOUD_VIEWER_HPP

#include <pcl/pcl_config.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <eigen3/Eigen/Dense>
#include <string>

#include <thread>
#include <mutex>
using namespace std::chrono_literals;

extern std::mutex sceneUpdateMutex; 
extern bool sceneUpdate; 

//TODO: Convert Eigen matrices (of points) to pcl point cloud (pointers)
void eigenToPcl(Eigen::MatrixXd& point_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr);

//TODO: Create pcl visualization viewer and add two point clouds 
pcl::visualization::PCLVisualizer::Ptr twoCloudVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr);

//TODO: visualize point clouds (to be called by separate thread) 
void visualizeClouds(Eigen::MatrixXd& model_set, Eigen::MatrixXd& new_scene_set);

#endif /* POINTCLOUD_VIEWER_HPP */  
