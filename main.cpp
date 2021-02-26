#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <cstring>
#include <cerrno> 

#include <thread>
using namespace std::chrono_literals;

#include "include/KDTree.hpp"
#include "include/data.hpp"
#include "include/registration.hpp"
#include "include/pointcloud_viewer.hpp"


int main(int argc, char** argv){

  std::cout << PCL_VERSION_PRETTY << std::endl;

  /* Import First Cloud */
  std::ifstream file_one;
  std::string path_one = "/home/timmy/icp_registration/practice_data/cloud_0.vtk";
  file_one.open(path_one.c_str());

  Eigen::MatrixXd point_cloud_one;
  
  if(!file_one){
    std::cerr << "First file didn't open: " << std::strerror(errno) << std::endl;
    return EXIT_FAILURE;    
  }else{
    point_cloud_one = getMatrix(file_one);
    
    std::cout << "--------------------- " << std::endl;
    std::cout << "First Cloud: " << std::endl;
    std::cout << "Number of rows: " << point_cloud_one.rows() << std::endl;
    std::cout << "Number of cols: " << point_cloud_one.cols() << std::endl;
    std::cout << "--------------------- " << std::endl;
    point_cloud_one.transposeInPlace();
    std::cout << "New number of rows: " << point_cloud_one.rows() << std::endl;
    std::cout << "New number of cols: " << point_cloud_one.cols() << std::endl;
    //std::cout << std::endl << point_cloud_one << std::endl;
  }

  /* Import Second Cloud */
  std::ifstream file_two;
  std::string path_two = "/home/timmy/icp_registration/practice_data/cloud_1.vtk";
  file_two.open(path_two.c_str());

  Eigen::MatrixXd point_cloud_two;
    
  if(!file_two){
    std::cerr << "Second file didn't open: " << std::strerror(errno) << std::endl;
    return EXIT_FAILURE;
  }else{
    point_cloud_two = getMatrix(file_two);
    
    std::cout << "\n";
    std::cout << "--------------------- " << std::endl;
    std::cout << "Second Cloud: " << std::endl;
    std::cout << "Number of rows: " << point_cloud_two.rows() << std::endl;
    std::cout << "Number of cols: " << point_cloud_two.cols() << std::endl;
    std::cout << "--------------------- " << std::endl;
    point_cloud_two.transposeInPlace();
    std::cout << "New number of rows: " << point_cloud_two.rows() << std::endl;
    std::cout << "New number of cols: " << point_cloud_two.cols() << std::endl;
    //std::cout << std::endl << point_cloud_two << std::endl; 
  }

  // Test out icp
  kd_tree my_tree(point_cloud_two);
  Eigen::MatrixXd Final = reg::icp(my_tree, point_cloud_one);
  std::cout << "Final Transformation: \n" << Final << std::endl;

  // Use final transformation to transform original point cloud
  Eigen::MatrixXd point_cloud_one_Transformed = Final * point_cloud_one.colwise().homogeneous(); 
  point_cloud_one_Transformed = reg::makeNotHomogeneous(point_cloud_one_Transformed);
  
  // Test out visualizaiton  
  // Create and populate two PCL PointCLoud objects from eigen matrices 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_one_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_two_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  eigenToPcl(point_cloud_one_Transformed, point_cloud_two, 
              cloud_one_ptr, cloud_two_ptr);

  // Create viewer and add both PointClouds 
  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = twoCloudVis(cloud_one_ptr, cloud_two_ptr);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  return EXIT_SUCCESS;
}
