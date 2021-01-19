#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <cstring>
#include <cerrno> 
#include <pcl/pcl_config.h>

#include "include/KDTree.hpp"
#include "include/data.hpp"
#include "include/registration.hpp"

typedef Eigen::Matrix<double,1,3> row_vec;

int main(int argc, char** argv){

  std::cout << PCL_VERSION_PRETTY << std::endl;

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
    std::cout << "New number of rows: " << point_cloud_zero.rows() << std::endl;
    std::cout << "New number of cols: " << point_cloud_zero.cols() << std::endl;
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
    std::cout << "New number of rows: " << point_cloud_one.rows() << std::endl;
    std::cout << "New number of cols: " << point_cloud_one.cols() << std::endl;
    //std::cout << std::endl << point_cloud_one << std::endl; 
  }

  // Test out matrix column removal (point rejection code) from registration.hpp 
  Eigen::MatrixXd test(3,7);
  test << 1,2,3,4,5,6,7,
          8,9,10,11,12,13,14,
          15,16,17,18,19,20,21;

  std::cout << test << std::endl;

  ////////std::cout << "Remove indices: 0,2" << std::endl;
  ////////std::vector<int> indices = {0,2};

  ////////reg::discardPoints(test, indices);
  ////////std::cout << test << std::endl;
  ////////std::cout << "\n\n\n\n\n" << std::endl;


  //Eigen::VectorXd ex_vector;
  //ex_vector = test.colwise().maxCoeff();
  //std::cout << "\nBefore " << std::endl << ex_vector << std::endl;
  //reg::removeElement(ex_vector, 2);
  //std::cout << "\nAfter " << std::endl << ex_vector << std::endl;
  //reg::removeElement(ex_vector, 4);
  //std::cout << "\nAfter again" << std::endl << ex_vector << std::endl;
  //unsigned int pos = 0; 
  //unsigned int* pos_ptr = &pos;
  //ex_vector.maxCoeff(pos_ptr); 
  //reg::removeElement(ex_vector, *pos_ptr);
  //std::cout << "\nYet again" << std::endl << ex_vector << std::endl;
  //std::cout << "\nIndex location of maximum elmeent" << std::endl << *pos_ptr << std::endl;
  //ex_vector.maxCoeff(pos_ptr); 
  //reg::removeElement(ex_vector, *pos_ptr);
  //std::cout << "\nWoohooo again" << std::endl << ex_vector << std::endl;
  //std::cout << std::endl << ex_vector.rows() << std::endl;
  //ex_vector.segment(0,4) = ex_vector.tail(4);
  //std::cout << "------------ \n Changes" <<  std::endl << ex_vector << std::endl;
  //std::cout << "\n ########################## \n First, let's see what head produces \n" << ex_vector.head(4) << std::endl;
  //std::cout << "\n ########################## \n Now let's see what the entire ex_vector looks like right now \n" << ex_vector.head(4) << std::endl;
  //Eigen::VectorXd new_vector = ex_vector.head(4); 
  //std::cout << "------------ \n More Changes" <<  std::endl << new_vector << std::endl;

  // Test out vector element removal () from registration.hpp 
  //std::cout << "index to remove: 2" << std::endl;
  //int rowToRemove = 2;
  //int numRows = ex_vector.rows() -1;
  //ex_vector.segment(rowToRemove,numRows-rowToRemove) = ex_vector.tail(numRows-rowToRemove);
  //ex_vector.conservativeResize(ex_vector.rows(),1);
  //std::cout << ex_vector << std::endl;
 // Eigen::VectorXd tester(1,10);
  //tester << 1.123, 2.234, 3.4893, 4.13893, 5.1234, 6.3289, 7.19430, 8.134590, 9.134950, 10.1239501; 
  //std::cout << tester.cols() << std::endl;
  //std::cout << tester.rows() << std::endl;
  //
  
  ////////////////////////////////////////
  ////////////////////////////////////////
  ////////////////////////////////////////
  ////////////////////////////////////////
                                          
  ////////////////////////////////////////
  //// Test out icp
  kd_tree my_tree(point_cloud_one);
  Eigen::MatrixXd Final = reg::icp(my_tree, point_cloud_zero);
  //
  std::cout << "Final Transformation: \n" << Final << std::endl;
  return EXIT_SUCCESS;
}
