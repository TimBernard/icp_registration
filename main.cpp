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
#include <filesystem>
#include <tuple> 

#include "include/KdTree.hpp"
#include "include/data.hpp"
#include "include/registration.hpp"
#include "include/pointcloud_viewer.hpp"

using namespace std::chrono_literals;

// Global variables 
std::mutex sceneUpdateMutex;
bool sceneUpdate;
Eigen::MatrixXd point_cloud_one;
Eigen::MatrixXd point_cloud_two;

// function to load cloud data from file name
bool import_cloud(std::string file_name, Eigen::MatrixXd& point_cloud);

int main(int argc, char** argv){

  /* Import Clouds */
  std::string file_one = "cloud_0.vtk";
  std::string file_two = "cloud_1.vtk";
  bool open_success_one = import_cloud(file_one, point_cloud_one);
  bool open_success_two = import_cloud(file_two, point_cloud_two);

  if (!open_success_one || !open_success_two){
    return EXIT_FAILURE;
  }

  // Test out icp
  KdTree my_tree(point_cloud_two);
  Eigen::MatrixXd Final = reg::icp(my_tree, point_cloud_two, point_cloud_one);
  std::cout << "Final Transformation: \n" << Final << std::endl;

  return EXIT_SUCCESS;
}

bool import_cloud(std::string file_name, Eigen::MatrixXd& point_cloud){
  std::ifstream file; 
  std::string path = std::string(std::filesystem::current_path()) +  "/../practice_data/" + file_name;
  file.open(path.c_str());
  bool open_success;

  if(!file){
    std::cerr << file_name + " didn't open: " << std::strerror(errno) << std::endl;
    open_success = false; 
  }else{
    point_cloud = getMatrix(file);
    
    std::cout << "--------------------- " << std::endl;
    std::cout << "Cloud from " << file_name << ": "  << std::endl;
    point_cloud.transposeInPlace();
    std::cout << "Number of rows: " << point_cloud.rows() << std::endl;
    std::cout << "Number of cols: " << point_cloud.cols() << std::endl;
    std::cout << "--------------------- " << std::endl << std::endl;
    open_success = true; 
  }
  return open_success;
}
