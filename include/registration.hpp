#ifndef REGISTRATION_HPP
#define REGISTRATION_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>

namespace reg{
  
  /**
     Solve for Rigid Transformation between two 3D point sets using Singular Value Decomposition. See: 

     Arun KS, Huang TS, Blostein SD. Least-squares fitting of two 3-d point sets. 
     IEEE Trans Pattern Anal Mach Intell. 1987;9(5):698-700. doi:10.1109/tpami.1987.4767965
     
     \param[in] A        Set of 3D points 
     \param[in] B        Set of 3D points 
     \return             SE(3) Homogeneous Transformation 
  */
  Eigen::MatrixXd rigidPointToPointSVD(const Eigen::MatrixXd& A,  const Eigen::MatrixXd& B);
  
  // Function that iteratively finds point correspondences and registrations between two point clouds
  Eigen::MatrixXd icp(const Eigen::MatrixXd& model_set, const Eigen::MatrixXd& scene_set);

  // Make matrix of closest points in model to scene set 
  Eigen::MatrixXd findClosestPoints(const Eigen::MatrixXd& model_set, const Eigen::MatrixXd& new_scene_set);

  // Make not homogeneous
  Eigen::MatrixXd makeNotHomogeneous(const Eigen::MatrixXd& points);

  // sets to homogeneous coordinates 
  Eigen::MatrixXd makeHomogeneous(const Eigen::MatrixXd& points);
    
}
#endif /* REGISTRATION_HPP */
