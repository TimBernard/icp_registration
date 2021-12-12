#ifndef REGISTRATION_HPP
#define REGISTRATION_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <typeinfo>
#include <KdTree.hpp>
#include <pointcloud_viewer.hpp>

extern std::mutex sceneUpdateMutex;
extern bool sceneUpdate;

namespace reg{
  
  // rigid point cloud alignment between two point sets 
  Eigen::MatrixXd rigid_point2point_SVD(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

  // iteratively align scene set to model set 
  Eigen::MatrixXd icp(KdTree& tree, /*const*/ Eigen::MatrixXd& model_set, /*const*/ Eigen::MatrixXd& scene_set);

  double compute_error(Eigen::MatrixXd& error_set, Eigen::MatrixXd& point_set);

  // Helper Eigen functions 
  void discard_point(Eigen::MatrixXd& mat, unsigned int index);
  void remove_element(Eigen::VectorXd& vec, unsigned int index);
  
  // find set of closest points using brute-force or KD-Tree
  Eigen::MatrixXd find_closest_points(const Eigen::MatrixXd& model_set, const Eigen::MatrixXd& new_scene_set);
  Eigen::MatrixXd find_closest_points_faster(KdTree& tree, const Eigen::MatrixXd& new_scene_set);   
  // Transition to and from homogeneous coordinates 
  Eigen::MatrixXd make_not_homogeneous(const Eigen::MatrixXd& points);
  Eigen::MatrixXd make_homogeneous(const Eigen::MatrixXd& points);

  // multiply set of N homogeneous points by same homogeneous SE(3) transformation 
  void multiply(const Eigen::Matrix4d& transformation, Eigen::MatrixXd& point_set);
}
#endif /* REGISTRATION_HPP */
