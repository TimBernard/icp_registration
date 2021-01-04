#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <cstring>
#include <cerrno> 

#include "include/KDTree.hpp"
#include "include/data.hpp"
#include "include/registration.hpp"

typedef Eigen::Matrix<double,1,3> row_vec;

int main(int argc, char** argv){

  /* Import First Cloud */
  std::ifstream file_zero;
  std::string path_zero = "/home/timmy/icp_registration/practice_data/cloud_0.vtk";
  file_zero.open(path_zero.c_str());

  Eigen::MatrixXd point_cloud_zero;
  
  if(!file_zero){
    std::cerr << "First file didn't open: " << std::strerror(errno) << std::endl;
    return EXIT_FAILURE;    
  }else{
    point_cloud_zero = getMatrix(file_zero);
    
    std::cout << "--------------------- " << std::endl;
    std::cout << "First Cloud: " << std::endl;
    std::cout << "Number of rows: " << point_cloud_zero.rows() << std::endl;
    std::cout << "Number of cols: " << point_cloud_zero.cols() << std::endl;
    std::cout << "--------------------- " << std::endl;
    point_cloud_zero.transposeInPlace();
    //std::cout << std::endl << point_cloud_zero << std::endl;
  }

  /* Import Second Cloud */
  std::ifstream file_one;
  std::string path_one = "/home/timmy/icp_registration/practice_data/cloud_1.vtk";
  file_one.open(path_one.c_str());

  Eigen::MatrixXd point_cloud_one;
    
  if(!file_one){
    std::cerr << "Second file didn't open: " << std::strerror(errno) << std::endl;
    return EXIT_FAILURE;
  }else{
    point_cloud_one = getMatrix(file_one);
    
    std::cout << "\n";
    std::cout << "--------------------- " << std::endl;
    std::cout << "Second Cloud: " << std::endl;
    std::cout << "Number of rows: " << point_cloud_one.rows() << std::endl;
    std::cout << "Number of cols: " << point_cloud_one.cols() << std::endl;
    std::cout << "--------------------- " << std::endl;
    point_cloud_one.transposeInPlace();
    //std::cout << std::endl << point_cloud_one << std::endl; 
  }

  // Test out icp
  kd_tree my_tree(point_cloud_one);
  Eigen::MatrixXd Final = reg::icp(my_tree, point_cloud_zero);
  
  std::cout << "Final Transformation: \n" << Final << std::endl;
  return EXIT_SUCCESS;
}














