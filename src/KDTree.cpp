#include <KDTree.hpp>

// intialize empty tree
kd_tree::kd_tree(){

  root = nullptr;
  smallestPoint = nullptr;
  dim = 0;
  Nm = 0;
}

// intialize tree and populate with 3xN point set (from model)
kd_tree::kd_tree(const Eigen::MatrixXd& points){

  std::cout << "Building tree..." << std::endl;

  dim = points.rows();
  Nm = points.cols();

  std::vector<Eigen::Vector3d> points_vec;
  points_vec.resize(points.cols());
  int count = 0;
  std::vector<Eigen::Vector3d>::iterator it = points_vec.begin();
  for(; it != points_vec.end(); ++it){
    *it = points.col(count);
    count++;
  }
  root = make_tree(points_vec.begin(),points_vec.end(),0);
  smallestPoint = nullptr;
  
}

// destroy objects and free memory 
kd_tree::~kd_tree(){ delete root;}
 
// make the tree from iterator to beginning and end of Eigen matrix (point set)
Node* kd_tree::make_tree(const std::vector<Eigen::Vector3d>::iterator& pts_begin,
                         const std::vector<Eigen::Vector3d>::iterator& pts_end,
                         int depth){

  if(pts_begin == pts_end){ return nullptr;}

  int axis = depth % dim;
  //int len = pts_end - pts_begin;
  
  auto cmp_eig_vec = [axis](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2){
    return (p1[axis] < p2[axis]); };
  auto pts_mid = (pts_begin + (pts_end-pts_begin)/2);
  std::nth_element(pts_begin, pts_mid, pts_end, cmp_eig_vec);

  Node* node = new Node(*pts_mid,depth);
  node->left = make_tree(pts_begin,pts_mid,depth+1);
  node->right = make_tree(pts_mid+1,pts_end,depth+1);
  return node;
} 

// in progress
Node* kd_tree::get_nn(const Eigen::Vector3d& query, Node* root){

  if (root == nullptr){
    return nullptr;
  } 

  double dist = (query - root->point).norm();
  if (dist < best_dist){
    best = root;
    best_dist = dist;
  }

return nullptr;
}












/*
  int n = points.cols();
  if (n <= 0){
    return nullptr;
  }

  std::vector<double> points_(&points[0],points.data()
*/
