#ifndef DATA_HPP
#define DATA_HPP

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <cstring>
#include <cerrno>

int get_num_points(std::ifstream& file);
Eigen::MatrixX3d get_matrix(std::ifstream& file);

#endif /* DATA_HPP */
