#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>

#define EXIT_SUCESS 0
#define EXIT_ERROR -1

#include "include/KDTree.hpp"

typedef Eigen::Matrix<double,1,3> row_vec;

int getNumLines(std::ifstream&,std::string);

int main(int argc, char** argv){

  // Test out c+11 and boost
  std::shared_ptr<std::string> string_ptr = std::make_shared<std::string>("Hello CMake!");
  std::cout << *string_ptr << std::endl;
  boost::shared_ptr<std::shared_ptr<std::string>> string_ptr_ptr;

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
    
  std::cout << "my_centroid Number of rows: " << my_centroid.rows() << std::endl;
  std::cout << "my_centroid Number of cols: " << my_centroid.cols() << std::endl;
  std::cout << "my_centroid : " << my_centroid << std::endl;

  std::cout << "my_centroid_row Number of rows: " << my_centroid_row.rows() << std::endl;
  std::cout << "my_centroid_row Number of cols: " << my_centroid_row.cols() << std::endl;
  std::cout << "my_centroid_row : " << my_centroid_row << std::endl;                  

  
  std::cout << my_matrix.rowwise() - my_centroid_row << std::endl;
  
  my_matrix.col(0) *= -1;

  std::cout << my_matrix << std::endl;

  Eigen::MatrixXd trans(4,4);
  Eigen::Matrix3d rot = Eigen::Matrix3d::Random(3,3);
  Eigen::Vector3d p = Eigen::Vector3d::Random(3,1);

  trans << rot, p, 0, 0, 0, 1;

  std::cout << trans << std::endl;
  std::cout << add(1,3) << std::endl;
  
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
