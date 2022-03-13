#ifndef DATA_IMPORT_HPP
#define DATA_IMPORT_HPP

#include <string>
#include <fstream>
#include <eigen3/Eigen/Dense>

namespace data {
  int get_num_points(std::ifstream& file);
  Eigen::MatrixX3d get_matrix(std::ifstream& file);
  bool import_cloud(const std::string& file_name, Eigen::MatrixXd& point_cloud);
}
#endif /* DATA_IMPORT_HPP */
