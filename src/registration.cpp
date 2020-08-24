#include "registration.hpp"

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
  Eigen::MatrixXd rigidPointToPointSVD(const Eigen::MatrixX3d& A,  const Eigen::MatrixX3d& B){
    
    int n_A = A.rows();
    int n_B = B.rows();
    int n;
    
    if(n_A != n_B){std::cout << "There is a problem" << std::endl;}
    else{n = n_A;}
    
    //q Calculate centroid of each point set 
    Eigen::Vector3d a_centroid = A.colwise().mean();
    Eigen::Vector3d b_centroid = B.colwise().mean();
    Eigen::RowVector3d a_centroid_ = a_centroid.transpose();
    Eigen::RowVector3d b_centroid_ = b_centroid.transpose();
    
    // Centering 
    Eigen::MatrixX3d A_prime = A.rowwise() - a_centroid_;
    Eigen::MatrixX3d B_prime = B.rowwise() - b_centroid_;  
    
    // Find 3x3 Matrix 
    Eigen::Matrix3d H;
    int x = 0;
    int y = 1;
    int z = 2;
    
    for(int i = 0; i < n; ++i){
      
      Eigen::Matrix3d mat;
      mat << A_prime(i,x)*B_prime(i,x), A_prime(i,x)*B_prime(i,y), A_prime(i,x)*B_prime(i,z),
	A_prime(i,y)*B_prime(i,x), A_prime(i,y)*B_prime(i,y), A_prime(i,y)*B_prime(i,z),
	A_prime(i,z)*B_prime(i,x), A_prime(i,z)*B_prime(i,y), A_prime(i,z)*B_prime(i,z);
      
      H += mat;    
    }
    
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    
    // Rotation
    Eigen::Matrix3d R = U * V.transpose();
    
    if(R.determinant() < 0){
      std::cout << "Condition reached" << std::endl;
      V.col(2)*=-1;
      R  = U * V.transpose();
      
      if(R.determinant() < 0){
	std::cout << "Algorithm failed" << std::endl;
      }
    }
    
    // Create Full Transformation 
    Eigen::Vector3d p = b_centroid -(R*a_centroid);
    Eigen::MatrixXd SE3(4,4);
    SE3 << R, p, 0, 0, 0, 1;
    
    return SE3;  
  }
  
}


// TODO For Direct Point set Registration: Quaternion method as seen in http://www.cs.jhu.edu/cista/455/Lectures/Rigid3D3DCalculations.pdf
// TODO For Direct Point ser Registration: The other Quaternion method as seen in http://www.cs.jhu.edu/cista/455/Lectures/Rigid3D3DCalculations.pdf
// TODO: KD-Tree for search


// TODO: ICP 
