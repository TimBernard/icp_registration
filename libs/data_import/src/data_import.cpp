#include <filesystem>
#include <iostream>
#include <vector>
#include <cstring>
#include <cerrno>
#include <sstream>
#include <eigen3/Eigen/Dense>

#include <data_import.hpp>

namespace data {
  /**
   * Takes an open file to get the total number of points.
   * Currently works with vtk data format
   *
   * @param file the opened file containing the points
   * @return num_points the number of points 
   */
  int get_num_points(std::ifstream& file){

    int num_points = 0;
    const int num_lines = 4;

    std::string line;
    for(int i = 0; i < num_lines; ++i){
      getline(file, line);
    }
    std::getline(file, line);
    num_points = std::stoi(line.substr(7,5));
      
    return num_points;  
  }

  /**
   * Takes an open file and imports points into an Nx3 matrix
   *
   * @param file the opened file containing the points
   * @return point_set the point set in a row major matrix storage format 
   */
  Eigen::MatrixX3d get_matrix(std::ifstream& file){
    
    // Find the number of points 
    const int num_points = get_num_points(file);
    file.clear();
    file.seekg(0);
    
    // Information on what line, how many lines to skip
    int count = 0;
    const int skip_count = 4;
    std::string line;

    // vector for conversion later
    std::vector<double> coords;
    coords.reserve(num_points*3);

    // Input all points from file 
    while(std::getline(file,line)){
          
      if(count <= skip_count){ 
        count++;  continue;
      }
      else if(count > (skip_count + num_points)){ 
        break;
      }
      else{
        std::stringstream line_ss(line);
        double coord;
        while(line_ss >> coord){
          coords.push_back(coord);
        }
        count++;
      }
    }

    // populate matrix from std::vector 
    Eigen::MatrixX3d point_set(num_points,3);
    double* coords_data = coords.data();
    point_set = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>(coords_data, num_points,3);
    
    return point_set;
  }

  /**
   * Takes a file name, opens the file and places in Eigen Matrix 
   *
   * @param file_name the name of the vtk file to be opened 
   * @return open_success a boolean value if successfully opened and converted to matrix 
   */
  bool import_cloud(const std::string& file_name, Eigen::MatrixXd& point_cloud){
    std::ifstream file; 
    std::string path = std::string(std::filesystem::current_path()) +  "/../practice_data/" + file_name;
    file.open(path.c_str());
    bool open_success = false;

    if(!file){
      std::cerr << file_name + " didn't open: " << std::strerror(errno) << std::endl;
      open_success = false; 
    }else{
      point_cloud = get_matrix(file);
      
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
}

