#include "registration.hpp"

namespace reg{  

  /**
   * Solve for rigid transformation between two 3D point sets using Singular Value Decomposition
   *
   * @param A set of 3D points
   * @param B set of 3D points 
   * @return SE3 the transformation that best aligns these two point clouds 
   */
  Eigen::MatrixXd rigidPointToPointSVD(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B){
    
    assert (A.cols() == B.cols());
    int n = A.cols();
    
    // Center point sets
    Eigen::Vector3d a_centroid = A.rowwise().mean();
    Eigen::Vector3d b_centroid = B.rowwise().mean();
    Eigen::MatrixXd A_prime = A.colwise() - a_centroid;
    Eigen::MatrixXd B_prime = B.colwise() - b_centroid;  
    
    // Find 3x3 Matrix, H
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
    
    // Full Transformation with translation
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
  Eigen::MatrixXd icp(kd_tree& tree, /*const*/ Eigen::MatrixXd& model_set, /*const*/ Eigen::MatrixXd& scene_set){

    // Initial Pose 
    Eigen::MatrixXd F_reg = Eigen::MatrixXd::Identity(4,4);
    
    // Adjust Initial Rotation 
    Eigen::Matrix3d InitRotation;
    // InitRotation = Matrix::Identity(3,3); 
    double theta = 0.5*M_PI; //0.25*M_PI; 
    InitRotation << cos(theta), -sin(theta), 0,
                    sin(theta), cos(theta), 0,
                    0, 0, 1;
    F_reg.topLeftCorner(3,3) = InitRotation;

    // // Adjust Initial Translation 
    // F_reg.col(3).setOnes(); 
    // F_reg.topRightCorner(3,1) *= 1.0;

    std::cout << "Initial guess: " << std::endl << F_reg << std::endl;

    // Useful Constants
    int max_iterations = 50;
    double threshold = 0.1; 
      
    // Apply intial transformation to the scene points
    Eigen::MatrixXd new_scene_set = scene_set;
    reg::multiply(F_reg, new_scene_set);

    // start visualizer thread 
    std::thread visualizerThread(visualizeClouds, std::ref(model_set), std::ref(new_scene_set));
    
    for(int i=0; i < max_iterations; ++i){

      //  std::lock_guard<std::mutex> lg(sceneUpdateMutex);
      sceneUpdate = true;
      
      std::cout << "\n--------------------------------------" << std::endl;
      std::cout << "Iteration " << (i+1) << " " << std::endl; 

      // Find the closest points to the newly transformed scene
      Eigen::MatrixXd CP = reg::findClosestPointsFaster(tree, new_scene_set);

      // Compute Alignment 
      F_reg = reg::rigidPointToPointSVD(new_scene_set, CP);
      //std::cout << "Current transform estimate: " << std::endl<< F_reg << std::endl;

      reg::multiply(F_reg, new_scene_set);
        
      Eigen::MatrixXd diff = CP - new_scene_set;
      double error = reg::computeError(diff,new_scene_set); 
      std::cout << "Error (MSE): " <<  error << std::endl;

      if(error <= threshold){
        std::cout << "Final Error (MSE):" <<  error << std::endl;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }      

    visualizerThread.join();
    
    return F_reg;
  }

  /**
   * Compute MSE error across set of points, and then discard worst 10% of points
   *
   * @param error_set, the difference between scene and closest model points 
   * @param point_set, the scene points
   */
  double computeError(Eigen::MatrixXd& error_set, Eigen::MatrixXd& point_set){

    int N = error_set.cols();
    int iterations = (int)(N*0.05);
    Eigen::VectorXd squaredDiff = error_set.colwise().squaredNorm();

    // Find maximum error values and remove from the point set
    /*  
    unsigned int maxIndex = 0; 
    unsigned int* maxIndex_ptr = &maxIndex;
    
    for(int i=0; i < iterations; ++i){
      squaredDiff.maxCoeff(maxIndex_ptr);
      reg::removeElement(squaredDiff, *maxIndex_ptr);
      reg::discardPoint(point_set, *maxIndex_ptr);
    }
   
    std::cout << "Current Number of scene points being used: " << point_set.cols() << std::endl;
    */  
    
    double mseError = squaredDiff.sum()/N;
    return mseError;
  }
  
  /**
   * Remove column from Eigen matrix from vector
   *
   * @param mat, matrix from Eigen lib
   * @param index, column location to do removal 
   */
  void discardPoint(Eigen::MatrixXd& mat, unsigned int index){

    unsigned int colToRemove = index;
    unsigned int numCols = mat.cols() -1;
    if( colToRemove < numCols ){
      mat.block(0,colToRemove,mat.rows(),numCols-colToRemove) = mat.rightCols(numCols-colToRemove);
    }
    mat.conservativeResize(mat.rows(),numCols);
  }
  
  /**
   * Remove element from vector
   *
   * @param vec, vector from Eigen lib
   * @param index, location to do removal 
   */
  void removeElement(Eigen::VectorXd& vec, unsigned int index){

    unsigned int numRows = vec.rows()-1;
    if (index < numRows){
      vec.segment(index,numRows-index) = vec.tail(numRows-index);
    }
    Eigen::VectorXd temp_vec = vec.head(numRows);
    vec = temp_vec;
  }

  /**
   * Take the transformed scene point set and find the points in the model that are closest to 
   * the points from this current estimate: Uses linear search  
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
    Eigen::MatrixXd CP(3,Ns);
    
    // Traverse through each scene point to find the closest in the model 
    Node* root = tree.get_root(); 
    for(int i = 0; i < Ns; ++i){
    
      tree.get_nn(new_scene_set.col(i),root,0);
      CP.col(i) = tree.get_best()->point;
      tree.reset_best();
    }
    std::cout << "[####---------------------------------------- "
    << "Closest points have been acquired" << " --------------------------------------####]" 
    << std::endl;
    return CP;
  }

  /**
   * Make set of homogenous points non-homogeneous 4xN -> 3xN
   * 
   * @param points homogeneous coordinates
   * @return points_ non-homogeneous coordinates
   */
  Eigen::MatrixXd makeNotHomogeneous(const Eigen::MatrixXd& points){

    Eigen::MatrixXd points_((int)points.rows()-1,(int)points.cols());
    points_ << points.row(0), points.row(1), points.row(2); 
    return points_; 
  }

  /**
   * Make set of non-homogenous points homogeneous 3xN -> 4xN,
   * 
   * @param points non-homogeneous coordinates
   * @return points_ homogeneous coordinates
   */
  Eigen::MatrixXd makeHomogeneous(const Eigen::MatrixXd& points){

    Eigen::MatrixXd points_((int)(points.rows()+1),(int)(points.cols()));
    Eigen::VectorXd ones_vec = Eigen::VectorXd::Ones(1,(int)points.cols());
    points_ << points.row(0), points.row(1), points.row(2), ones_vec;
    return points_; 
  }

  // TODO: Abstract away muplication of set of points by rigid transformation 
  void multiply(const Eigen::Matrix4d& transformation, Eigen::MatrixXd& point_set){

    point_set = transformation * point_set.colwise().homogeneous();
    point_set = reg::makeNotHomogeneous(point_set);
  }
}
