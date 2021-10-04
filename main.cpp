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

// Global variables 
std::mutex sceneUpdateMutex;
bool sceneUpdate;
Eigen::MatrixXd point_cloud_one;
Eigen::MatrixXd point_cloud_two;


int main(int argc, char** argv){

  std::cout << PCL_VERSION_PRETTY << std::endl;

  /* Import First Cloud */
  std::ifstream file_one;
  std::string path_one = "/home/tim/icp_registration/practice_data/cloud_0.vtk";
  file_one.open(path_one.c_str());
  
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
  std::string path_two = "/home/tim/icp_registration/practice_data/cloud_1.vtk";
  file_two.open(path_two.c_str());

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
  Eigen::MatrixXd Final = reg::icp(my_tree, point_cloud_two, point_cloud_one);
  std::cout << "Final Transformation: \n" << Final << std::endl;

  return EXIT_SUCCESS;
}
