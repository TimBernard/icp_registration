#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <eigen3/Eigen/Core>
#include <iostream>
#include <algorithm>
#include <limits>
#include <boost/shared_ptr.hpp>

// Dummy function
int add(int a, int b);

struct Node{
  Node(const Eigen::Vector3d& pt, int lvl) : point(pt), depth(lvl) {} 
  Node(const Eigen::Vector3d& pt, Node* l, Node* r) : point(pt), left(l), right(r) {}
  ~Node(){ 
    delete left;
    delete right;
  } 
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Node* left = nullptr;
  Node* right = nullptr;
  int depth = 0;
};

class kd_tree{
  public:
    kd_tree();
    kd_tree(const Eigen::MatrixXd& points);
    ~kd_tree();
    Node* make_tree(const std::vector<Eigen::Vector3d>::iterator& pts_begin,
                    const std::vector<Eigen::Vector3d>::iterator& pts_end, 
                    int depth);     
    Node* get_nn(const Eigen::Vector3d& query, Node* T);  
    Node* get_root() const{ return root; }


  private:
    Node* root;
    Node* smallestPoint;
    Node* best = nullptr;
    double best_dist = std::numeric_limits<double>::max();
    int dim;
    int Nm; 
};



#endif /* KDTREE_HPP */
