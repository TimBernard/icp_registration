#include "registration.hpp"

namespace reg{  

  /**
   * Solve for rigid transformation between two 3D point sets using Singular Value Decomposition. See:
   *
   * Arun KS, Huang TS, Blostein SD. Least-squares fitting of two 3-d point sets.                                                                                                      
   * IEEE Trans Pattern Anal Mach Intell. 1987;9(5):698-700. doi:10.1109/tpami.1987.4767965
   * 
   * @param A set of 3D points
   * @param B set of 3D points 
   * @return SE3 the transformation that best aligns these two point clouds 
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
  Eigen::MatrixXd icp(kd_tree& tree, const Eigen::MatrixXd& scene_set){

    // Initial Pose 
    Eigen::MatrixXd F_reg = Eigen::MatrixXd::Identity(4,4);

    // Useful Constants
    int max_iterations = 50;
    double threshold = 0.1;
      
    // Apply intial transformation to the scene points
    Eigen::MatrixXd new_scene_set = F_reg * scene_set.colwise().homogeneous();
    new_scene_set = reg::makeNotHomogeneous(new_scene_set);
    
    for(int i=0; i < max_iterations; ++i){
      
      std::cout << "Iteration " << (i+1) << ": ";

      // Find the closest points to the newly transformed scene
      Eigen::MatrixXd CP = reg::findClosestPointsFaster(tree, new_scene_set);
      
      // Compute Alignment 
      F_reg = reg::rigidPointToPointSVD(new_scene_set, CP);

      // Apply Alignment and compute error 
      new_scene_set = F_reg * scene_set.colwise().homogeneous();
      new_scene_set = reg::makeNotHomogeneous(new_scene_set);
      Eigen::MatrixXd diff = CP - new_scene_set;
      double error = reg::computeError(diff);

      std::cout << "Error (sum of squared difference): " <<  error << std::endl;
      
      if(error <= threshold){
        std::cout << "Final Error (sum of squared difference) :" <<  error << std::endl;
        break;
      }
    }      
    
    return F_reg;
  }

  // sum of squared error 
  double computeError(const Eigen::MatrixXd& error_set){

    int N = error_set.cols();
    return (error_set.colwise().squaredNorm().sum())/N;
  }

  /**
   * Take the transformed scene point set and find the points in the model that are closest to 
   * the points from this current estimate: Uses linear search approach 
   *
   * @param model_set the set of points from the model cloud
   * @param new_scene_set the set of points from the transformed scene
   * @return CP the set of closest points to the transformed scene cloud
   */
  Eigen::MatrixXd findClosestPoints(const Eigen::MatrixXd& model_set, const Eigen::MatrixXd& new_scene_set){

    int Nm = model_set.cols();
    int Ns = new_scene_set.cols();

    //Eigen::MatrixXd CP(3,(int)new_scene_set.cols());
    Eigen::MatrixXd CP(3,Ns);
    Eigen::Vector3d scene_point; 
    std::vector<double> distances;
    distances.resize(Nm);
    for(int i = 0; i < Ns; ++i){

      std::cout << "Point Number: " << i << std::endl;
      scene_point = new_scene_set.col(i);
      std::cout << "Point: " << scene_point << std::endl;

      // Find the distances between this point and all model points
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

  /**
   * Take the transformed scene point set and find the points in the model that are closest to 
   * the points from this current estimate: Uses a KD Tree for sped up search  
   *
   * @param tree the set of points from the model cloud (held in a tree)
   * @param new_scene_set the set of points from the transformed scene
   * @return CP the set of closest points to the transformed scene cloud
   */
  Eigen::MatrixXd findClosestPointsFaster(kd_tree& tree, const Eigen::MatrixXd& new_scene_set){
    
    int Ns = new_scene_set.cols();
    //Eigen::MatrixXd CP(3,(int)new_scene_set.cols());
    Eigen::MatrixXd CP(3,Ns);
    
    // Traverse through each scene point to find the closest in the model 
    Node* root = tree.get_root(); 
    for(int i = 0; i < Ns; ++i){
    
      tree.get_nn(new_scene_set.col(i),root,0);
      CP.col(i) = tree.get_best()->point;
      tree.reset_best();
    }
    return CP;
  }

  /**
   * Make set of homogenous points non-homogeneous 4xN -> 3xN
   * 
   * @param points set of homogeneous coordinates
   * @return points_ set of non-homogeneous coordinates
   */
  Eigen::MatrixXd makeNotHomogeneous(const Eigen::MatrixXd& points){

    Eigen::MatrixXd points_((int)points.rows()-1,(int)points.cols());
    points_ << points.row(0), points.row(1), points.row(2); 
    return points_; 
  }

  /**
   * Make set of non-homogenous points homogeneous 3xN -> 4xN,
   * appending 1 to each row 
   * 
   * @param points set of non-homogeneous coordinates
   * @return points_ set of homogeneous coordinates
   */
  Eigen::MatrixXd makeHomogeneous(const Eigen::MatrixXd& points){

    Eigen::MatrixXd points_((int)(points.rows()+1),(int)(points.cols()));
    Eigen::VectorXd ones_vec = Eigen::VectorXd::Ones(1,(int)points.cols());
    points_ << points.row(0), points.row(1), points.row(2), ones_vec;
    return points_; 
  }
}
