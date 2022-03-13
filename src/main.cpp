#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <thread>

#include "registration.hpp"
#include "data_import.hpp"

using namespace std::chrono_literals;

// Global variables 
std::mutex sceneUpdateMutex;
bool sceneUpdate = false;
Eigen::MatrixXd point_cloud_one;
Eigen::MatrixXd point_cloud_two;

int main(){

  /* Import Clouds */
  std::string file_one = "cloud_0.vtk";
  std::string file_two = "cloud_1.vtk";
  bool open_success_one = data::import_cloud(file_one, point_cloud_one);
  bool open_success_two = data::import_cloud(file_two, point_cloud_two);

  if (!open_success_one || !open_success_two){
    return EXIT_FAILURE;
  }

  // Test out icp
  Eigen::MatrixXd Final = reg::icp(point_cloud_two, point_cloud_one);
  std::cout << "Final Transformation: \n" << Final << std::endl;

  return EXIT_SUCCESS;
}

