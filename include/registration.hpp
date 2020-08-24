#ifndef REGISTRATION_HPP
#define REGISTRATION_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>

namespace reg{
  
  /**
     Solve for Rigid Transformation between two 3D point sets using Singular Value Decomposition.

     See: 
     Arun KS, Huang TS, Blostein SD. Least-squares fitting of two 3-d point sets. 
     IEEE Trans Pattern Anal Mach Intell. 1987;9(5):698-700. doi:10.1109/tpami.1987.4767965
     
     \param[in] A        Set of 3D points 
     \param[in] B        Set of 3D points 
     \return             SE3 Homogeneous Transformation 
  */
  Eigen::MatrixXd rigidPointToPointSVD(const Eigen::MatrixX3d& A,  const Eigen::MatrixX3d& B);
  
  // Function that iteratively finds point correspondences and registrations between two point clouds
  Eigen::MatrixXd icp();
    
}
#endif /* REGISTRATION_HPP */
