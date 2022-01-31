#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <eigen3/Eigen/Core>
#include <iostream>
#include <algorithm>
#include <limits>
#include <vector>
#include <cmath>

struct Node{
  Node(const Eigen::Vector3d& pt, int lvl) : point(pt), depth(lvl) {} 
  ~Node(){ 
    delete left;
    delete right;
    left = nullptr;
    right = nullptr;
  } 
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Node* left = nullptr; 
  Node* right = nullptr; 
  int depth = 0;
};


/**
 * Stores root of KD Tree and estimates of current "best" point in terms of 
 * distance (for nn lookup). Has methods for creating tree from vector of points
 * and for doing nn search. 
 */
class KdTree{
  public:

    // Constructors
    KdTree() = default;
    KdTree(const Eigen::MatrixXd& points);

    // Destroy tree, free mem
    ~KdTree();

    // Make tree from points 
    Node* make_tree(const std::vector<Eigen::Vector3d>::iterator& pts_begin,
                                    const std::vector<Eigen::Vector3d>::iterator& pts_end, 
                                    int depth);     
    // Get nearest neighbor 
    void get_nn(const Eigen::Vector3d& query, Node* T, int depth);  

    // Helper functions
    Node* get_root() const { return root;}
    Node* get_best() const { return best;}
    void reset_best(){ best = nullptr; best_dist = std::numeric_limits<double>::max();}
    
  private:
    Node* root = nullptr;
    Node* best = nullptr;
    double best_dist = std::numeric_limits<double>::max();
    int dim = 3;
    int Nm = 0; 
};
#endif /* KDTREE_HPP */
