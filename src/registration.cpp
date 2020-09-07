#include "registration.hpp"

namespace reg{  

  /**  
     Solve for Rigid Transformation between two 3D point sets using Singular Value Decomposition. See:

     Arun KS, Huang TS, Blostein SD. Least-squares fitting of two 3-d point sets.                                                                                                      
     IEEE Trans Pattern Anal Mach Intell. 1987;9(5):698-700. doi:10.1109/tpami.1987.4767965

     \param[in] A        Set of 3D points                                                                                                                                              
     \param[in] B        Set of 3D points                                                                      
     \return             SE(3) Homogeneous Transformation 
  */
  Eigen::MatrixXd rigidPointToPointSVD(const Eigen::MatrixXd& A,  const Eigen::MatrixXd& B){
    
    int n_A = A.cols();
    int n_B = B.cols();
    int n;
    
    if(n_A != n_B){std::cout << "There is a problem" << std::endl;}
    else{n = n_A;}
    
    // Calculate centroid of each point set 
    Eigen::Vector3d a_centroid = A.rowwise().mean();
    Eigen::Vector3d b_centroid = B.rowwise().mean();
    
    // Centering 
    Eigen::MatrixXd A_prime = A.colwise() - a_centroid;
    Eigen::MatrixXd B_prime = B.colwise() - b_centroid;  
    
    // Find 3x3 Matrix 
    Eigen::Matrix3d H;
    int x = 0, y = 1, z = 2;
    
    for(int i = 0; i < n; ++i){
      Eigen::Matrix3d mat;
      mat << A_prime(x,i)*B_prime(x,i), A_prime(x,i)*B_prime(y,i), A_prime(x,i)*B_prime(z,i),
             A_prime(y,i)*B_prime(x,i), A_prime(y,i)*B_prime(y,i), A_prime(y,i)*B_prime(z,i),
             A_prime(z,i)*B_prime(x,i), A_prime(z,i)*B_prime(y,i), A_prime(z,i)*B_prime(z,i);
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

  // TODO: ICP
  Eigen::MatrixXd icp(const Eigen::MatrixXd& model_set, const Eigen::MatrixXd& scene_set){

    // Initial Pose 
    Eigen::MatrixXd initial_pose(4,4);

    // Initial Error
    Eigen::MatrixXd reg_final(4,4);
    reg_final = Eigen::MatrixXd::Random(4,4);
    return reg_final;
  }

  //TODO: Find closest points
  Eigen::MatrixXd findClosestPoints(const Eigen::MatrixXd& model_set, const Eigen::MatrixXd& new_scene_set){

    int Nm = model_set.cols();
    int Ns = new_scene_set.cols();

    Eigen::MatrixXd CP(3,(int)new_scene_set.cols());
    Eigen::Vector3d scene_point;

    std::vector<double> distances;
    distances.resize(Nm);
    for(int i = 0; i < Ns; ++i){

      std::cout << "Point Number: " << i << std::endl;
      scene_point = new_scene_set.col(i);
      std::cout << "Point: " << scene_point << std::endl;

      // Find the distances between this point and all model points
      //std::vector<double> distances;
      //distances.resize(Nm);
      for(int j = 0; j < Nm; ++j){
        distances[j]=(scene_point-model_set.col(j)).norm();
      }

      // Sort distances and take the closest one 
      std::vector<double> distances_org = distances;
      std::sort(distances.begin(),distances.end(), [] (double a, double b){return (a<b);});
      
      std::vector<double>::iterator it;
      it = std::find(distances_org.begin(),distances_org.end(),distances[0]);
      CP.col(i) = model_set.col(it-distances_org.begin());
    }
     
    return CP;
  }
  
  // make Nx4 set of points all non-homogneous 
  Eigen::MatrixXd makeNotHomogeneous(const Eigen::MatrixXd& points){

    Eigen::MatrixXd points_((int)points.rows()-1,(int)points.cols());
    points_ << points.row(0), points.row(1), points.row(2); 
    return points_; 
  }

  // sets a 3xN set of points to 4xN, appending 1 to each row
  Eigen::MatrixXd makeHomogeneous(const Eigen::MatrixXd& points){

    Eigen::MatrixXd points_((int)(points.rows()+1),(int)(points.cols()));
    Eigen::VectorXd ones_vec = Eigen::VectorXd::Ones(1,(int)points.cols());
    points_ << points.row(0), points.row(1), points.row(2), ones_vec;
    return points_; 
  }
}
