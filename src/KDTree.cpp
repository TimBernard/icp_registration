#include <KDTree.hpp>

/**
 * Default constructor 
 */
kd_tree::kd_tree(){

  root = nullptr;
  dim = 0;
  Nm = 0;
}

/** 
 * Takes an Eigen Matrix of points and creates a tree (a KD Tree)
 * with the tree's toplevel node stored as the root member variable
 *
 * @param points the set of points to make a tree of 
 */
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
  std::cout << "Tree built" << std::endl;
}

/**
 * Destroys tree and frees memory
 */
kd_tree::~kd_tree(){ 

  delete root;
  root = NULL;
}
 
/**
 * Takes the beginning and end of a point set and recursively creates subtrees that
 * split the current dimension in half (according to the midpoint of the data along 
 * that axis)
 *
 * @param pts_begin an iterator of the beginning of a vector of points
 * @param pts_end an iterator to the end of a vector of points
 * @param depth the current depth of the tree   
 */
Node* kd_tree::make_tree(const std::vector<Eigen::Vector3d>::iterator& pts_begin,
                         const std::vector<Eigen::Vector3d>::iterator& pts_end,
                         int depth){

  if(pts_begin == pts_end){ return nullptr;}

  int axis = depth % dim;
  
  auto cmp_eig_vec = [axis](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2){
    return (p1[axis] < p2[axis]); };
  auto pts_mid = (pts_begin + (pts_end-pts_begin)/2);
  std::nth_element(pts_begin, pts_mid, pts_end, cmp_eig_vec);

  Node* node = new Node(*pts_mid,depth);
  node->left = make_tree(pts_begin,pts_mid,depth+1);
  node->right = make_tree(pts_mid+1,pts_end,depth+1);
  return node;
} 


/**
 * Takes a query point, and finds its nearest neighbor within the KD Tree
 * by updating the "best" node pointer member of the tree associated with 
 * the input root, T
 *
 * @param query the point we are querying 
 * @param T the root node of the KD Tree
 * @param depth the depth of the tree
 */
void kd_tree::get_nn(const Eigen::Vector3d& query, Node* T, int depth){
  
  if (T == nullptr){ return; } 

  int axis = depth % dim;

  // Update best estimate, if the current node is closer 
  double dist = (query - T->point).norm();
  if (dist < best_dist){
    best = T;
    best_dist = dist;
  }
  
  // Search the side of the tree/subtree according to if it's less than,
  // greater than or equal to current point along this depth's axis 
  bool search_left = false;
  if(query[axis] < T->point[axis]){
    search_left = true;
    get_nn(query, T->left, depth+1);
  }else{
    search_left = false;
    get_nn(query, T->right, depth+1);
  }

  // Restrict search to only areas within the current hypersphere 
  if(std::abs(T->point[axis]-best->point[axis]) < best_dist){
    if (search_left == false){
      get_nn(query, T->left, depth+1);
    }
    else{
      get_nn(query, T->right, depth+1);
    }
  }
}


