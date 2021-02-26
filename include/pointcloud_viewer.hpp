#include <pcl/pcl_config.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <eigen3/Eigen/Dense>

#include <thread>
using namespace std::chrono_literals;


//TODO: Convert Eigen matrices (of points) to pcl point cloud (pointers)
void eigenToPcl(Eigen::MatrixXd& point_cloud_one, Eigen::MatrixXd& point_cloud_two,
                pcl::PointCloud<pcl::PointXYZ>::Ptr pc_one_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_two_ptr);

//TODO: Create pcl visualization viewer and add two point clouds 
pcl::visualization::PCLVisualizer::Ptr twoCloudVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_one, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_two);
  
