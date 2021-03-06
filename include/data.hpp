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

/**
 * Takes an open file to get the total number of points.
 * Currently works with vtk data format
 *
 * @param file the opened file containing the points
 * @return num_points the number of points 
 */
int getNumPoints(std::ifstream& file);

/**
 * Takes an open file and imports points into an Nx3 matrix
 *
 * @param file the opened file containing the points
 * @return point_set the point set in a matrix
 */
Eigen::MatrixX3d getMatrix(std::ifstream& file);

#endif /* DATA_HPP */
