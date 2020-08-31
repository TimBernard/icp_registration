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

// find number of points
int getNumPoints(std::ifstream& file);

// Return Nx3 matrix representing point set
Eigen::MatrixX3d getMatrix(std::ifstream& file);

#endif /* DATA_HPP */
