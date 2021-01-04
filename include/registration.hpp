#ifndef REGISTRATION_HPP
#define REGISTRATION_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <typeinfo>
#include <KDTree.hpp>

namespace reg{
  
  /**
   * Solve for rigid transformation between two 3D point sets using Singular Value Decomposition:
   *
   * Arun KS, Huang TS, Blostein SD. Least-squares fitting of two 3-d point sets.                                                                                                      
   * IEEE Trans Pattern Anal Mach Intell. 1987;9(5):698-700. doi:10.1109/tpami.1987.4767965
   * 
   * @param A set of 3D points
   * @param B set of 3D points 
   * @return SE3 the transformation that best aligns these two point clouds 
   */
  Eigen::MatrixXd rigidPointToPointSVD(const Eigen::MatrixXd& A,  const Eigen::MatrixXd& B);
  
  /**
   * Very simple version of Iterative Closest Point, TODO: Implement better
   * initial transformation, remove worst point matches, use more sophisticated
   * convergence criteria <---> has not been checked for performance 
   *
   * @param tree, kd-tree with model points
   * @param scene_set, point cloud of scene points 
   * @return F_reg, best current estimate of aligning transformation 
   *
   */
  Eigen::MatrixXd icp(kd_tree& tree, const Eigen::MatrixXd& scene_set);

  //TODO: Take in error matrix and compute sum of squared error 
  double computeError(const Eigen::MatrixXd& A);

  /**
   * Take the transformed scene point set and find the points in the model that are closest to 
   * the points from this current estimate: Uses linear search 
   *
   * @param model_set the set of points from the model cloud
   * @param new_scene_set the set of points from the transformed scene
   * @return CP the set of closest points to the transformed scene cloud
   */
  Eigen::MatrixXd findClosestPoints(const Eigen::MatrixXd& model_set, const Eigen::MatrixXd& new_scene_set);

  /**
   * Take the transformed scene point set and find the points in the model that are closest to 
   * the points from this current estimate: Uses a KD Tree for sped up search  
   *
   * @param tree the set of points from the model cloud (held in a tree)
   * @param new_scene_set the set of points from the transformed scene
   * @return CP the set of closest points to the transformed scene cloud
   */
  Eigen::MatrixXd findClosestPointsFaster(kd_tree& tree, const Eigen::MatrixXd& new_scene_set);   

  /**
   * Make set of homogenous points non-homogeneous 4xN -> 3xN
   * 
   * @param points homogeneous coordinates
   * @return points_ non-homogeneous coordinates
   */
  Eigen::MatrixXd makeNotHomogeneous(const Eigen::MatrixXd& points);

  /**
   * Make set of non-homogenous points homogeneous 3xN -> 4xN,
   * 
   * @param points non-homogeneous coordinates
   * @return points_ homogeneous coordinates
   */
  Eigen::MatrixXd makeHomogeneous(const Eigen::MatrixXd& points);
    
}
#endif /* REGISTRATION_HPP */
