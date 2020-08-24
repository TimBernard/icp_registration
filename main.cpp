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
//#define EXIT_FAILURE -1

#include "include/KDTree.hpp"
#include "include/data.hpp"
#include "include/registration.hpp"

// delete 
typedef Eigen::Matrix<double,1,3> row_vec;
int getNumLines(std::ifstream&,std::string);
// this 

int main(int argc, char** argv){

  // Test out c+11 and boost
  std::shared_ptr<std::string> string_ptr = std::make_shared<std::string>("Hello CMake!");
  std::cout << *string_ptr << std::endl;
  boost::shared_ptr<std::shared_ptr<std::string>> string_ptr_ptr;
  
  /* Import Ffirst Cloud */
  std::ifstream file_zero;
  std::string path_zero = "/home/timmy/icp_registration/practice_data/cloud_0.vtk";
  file_zero.open(path_zero.c_str());

  Eigen::MatrixX3d point_cloud_zero;
  
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
    //std::cout << std::endl << point_cloud_zero << std::endl;
  }

  /* Import Second Cloud */
  std::ifstream file_one;
  std::string path_one = "/home/timmy/icp_registration/practice_data/cloud_0.vtk";
  file_one.open(path_one.c_str());

  Eigen::MatrixX3d point_cloud_one;
    
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
    //std::cout << std::endl << point_cloud_one << std::endl; 
  }

  /**
   * Test out point cloud registration 
   */

  Eigen::MatrixXd Estimate = rigidPointToPointSVD(point_cloud_zero,point_cloud_one);
  std::cout << std::endl << Estimate << std::endl;



  
  //int num_lines = getNumLines(file_zero,
  /*
  Eigen::Matrix3d mine = Eigen::Matrix3d::Zero(3,3);

  int count = 0;
  for(int i = 0; i < 3; ++i){
    for(int j = 0; j < 3; ++j){
      count++;
      mine(i,j) = count;	
    }
  }
  
  std::cout << "mine: \n" << mine << std::endl;
  */
  
  
  /*
  // Test file i/o
  std::ifstream my_file;
  std::string pls = "/home/timmy/reg_project/data_PA/Problem4-BodyB.txt";
  my_file.open(pls.c_str());
  std::string line;
  std::string line2;

  int num_lines = getNumLines(my_file,line2);
  std::cout << "Btw, number of lines= " << num_lines << std::endl;
  my_file.close();my_file.open(pls.c_str());
  my_file.clear();
  my_file.open(pls.c_str());
  
  
  if(my_file.is_open()){
    while(std::getline(my_file,line)){
      std::cout << line << std::endl;
    }
    
    my_file.close();
  }
  else{
    std::cerr << "Could not open file!" << std::endl;
    return EXIT_ERROR;
  }
  */
  
  // Test out Eigen
  
  Eigen::MatrixX3d my_matrix(4,3);
  
  my_matrix << 1.0,2.0,3.0,
    1.0, 2.0, 3.0,                                                                                                                                                                     
    4.0, 5.0, 6.0,
    7.0, 8.0, 9.0;
  
  Eigen::Vector3d my_centroid = my_matrix.colwise().mean();
  Eigen::RowVector3d my_centroid_row = my_centroid.transpose();
    
  //std::cout << "my_centroid Number of rows: " << my_centroid.rows() << std::endl;
  //std::cout << "my_centroid Number of cols: " << my_centroid.cols() << std::endl;
  //std::cout << "my_centroid : " << my_centroid << std::endl;

  //std::cout << "my_centroid_row Number of rows: " << my_centroid_row.rows() << std::endl;
  //std::cout << "my_centroid_row Number of cols: " << my_centroid_row.cols() << std::endl;
  //std::cout << "my_centroid_row : " << my_centroid_row << std::endl;                  

  
  //std::cout << my_matrix.rowwise() - my_centroid_row << std::endl;
  
  my_matrix.col(0) *= -1;

  //std::cout << my_matrix << std::endl;

  Eigen::MatrixXd trans(4,4);
  Eigen::Matrix3d rot = Eigen::Matrix3d::Random(3,3);
  Eigen::Vector3d p = Eigen::Vector3d::Random(3,1);

  trans << rot, p, 0, 0, 0, 1;

  //std::cout << trans << std::endl;
  //std::cout << add(1,3) << std::endl;
  
  return EXIT_SUCESS;
}

// Get the number of lines in a file 
int getNumLines(std::ifstream& file ,std::string line){
    
  int count = 0;
  while(std::getline(file,line)){
    count++;
  }
  return count;
}

/*
int getNumPoints(std::ifstream& file){
  
  int num_points = 0;
  int num_lines = 4; 
  
  std::string line;
  for(int i = 0; i < num_lines; ++i){
    getline(file, line);
  }
  std::getline(file, line);

  num_points = std::stoi(line.substr(7,5));
  std::cout << "Points: " << num_points << std::endl;

  return num_points;
}
*/
