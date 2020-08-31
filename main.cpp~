#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <cstring>
#include <cerrno> 

#define EXIT_SUCESS 0

#include "include/KDTree.hpp"
#include "include/data.hpp"
#include "include/registration.hpp"

typedef Eigen::Matrix<double,1,3> row_vec;

int main(int argc, char** argv){

  // Test out c+11 and boost
  std::shared_ptr<std::string> string_ptr = std::make_shared<std::string>("Hello CMake!");
  std::cout << *string_ptr << std::endl;
  boost::shared_ptr<std::shared_ptr<std::string>> string_ptr_ptr;
  
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

  /* Test out point cloud registration */
  Eigen::MatrixXd Estimate = reg::rigidPointToPointSVD(point_cloud_zero,point_cloud_zero);
  std::cout << std::endl << Estimate << std::endl;

  /* Test out Linear Transformations in Eigen */
  Eigen::Matrix4d my_mat(4,4);
  Eigen::Matrix3d my_rot = Eigen::Matrix3d::Random(3,3);  
  Eigen::Vector3d my_p = Eigen::Vector3d::Random(3,1);
  my_mat << my_rot, my_p, 0, 0, 0, 1;

  // Do Above with 3xN point set instead of Nx3 point set 

  Eigen::MatrixXd my_points(3,10);
  my_points = Eigen::MatrixXd::Random(3,10);

  std::cout << "Original Point set: " << std::endl;
  std::cout << my_points << std::endl;

  std::cout << "Transformation: " << std::endl;
  std::cout << my_mat << std::endl;

  Eigen::MatrixXd new_my_points(4,10);
  new_my_points = my_mat * my_points.colwise().homogeneous();

  std::cout << "Transformed points: " << std::endl;
  std::cout << new_my_points << std::endl;

  //Eigen::MatrixXd new_my_points_regular(3,10);
  //new_my_points_regular << new_my_points.row(0),  new_my_points.row(1), new_my_points.row(2);
  std::cout << "Now without the ones row on the bottom: " << std::endl;
  std::cout << reg::makeNotHomogeneous(new_my_points) << std::endl;

  /* Test out finding closest points to transformed scene set in the model set */
  Eigen::MatrixXd my_CP = reg::findClosestPoints(point_cloud_one, point_cloud_zero);
  std::cout << my_CP << std::endl;
     
  return EXIT_SUCESS;
}
