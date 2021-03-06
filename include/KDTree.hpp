#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <eigen3/Eigen/Core>
#include <iostream>
#include <algorithm>
#include <limits>
#include <boost/shared_ptr.hpp>
#include <cmath>

/** Node class to handle hold point data and children nodes */
struct Node{
  Node(const Eigen::Vector3d& pt, int lvl) : point(pt), depth(lvl) {} 
  Node(const Eigen::Vector3d& pt, Node* l, Node* r) : point(pt), left(l), right(r) {}
  ~Node(){ 
    delete left;
    left = NULL;
    delete right;
    right = NULL;
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
class kd_tree{
  public:

    // Default constructor
    kd_tree();

    /** 
     * Takes an Eigen Matrix of points and creates a tree (a KD Tree)
     * with the tree's toplevel node stored as the root member variable
     *
     * @param points the set of points to make a tree of 
     */
    kd_tree(const Eigen::MatrixXd& points);

    // Destroys tree and frees memory
    ~kd_tree();

    /**
     * Takes the beginning and end of a point set and recursively creates subtrees that
     * split the current dimension in half (according to the midpoint of the data along 
     * that axis)
     *
     * @param pts_begin an iterator of the beginning of a vector of points
     * @param pts_end an iterator to the end of a vector of points
     * @param depth the current depth of the tree   
     */
    Node* make_tree(const std::vector<Eigen::Vector3d>::iterator& pts_begin,
                    const std::vector<Eigen::Vector3d>::iterator& pts_end, 
                    int depth);     
    /**
     * Takes a query point, and finds its nearest neighbor within the KD Tree
     * by updating the "best" node pointer member of the tree associated with 
     * the input root, T
     *
     * @param query the point we are querying 
     * @param T the root node of the KD Tree
     * @param depth the depth of the tree
     */
    void get_nn(const Eigen::Vector3d& query, Node* T, int depth);  

    // Helper functions
    Node* get_root() const{ return root; }
    Node* get_best() const{ return best; }
    void reset_best(){ best = nullptr; best_dist = std::numeric_limits<double>::max();}
    
    // Comparison for Eigen::Vector3d along specified axis
    struct compare_vec {
      compare_vec(int axis_): axis(axis_){}
      bool operator()(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2) {
        return (pt1[axis] < pt2[axis]);
      }
      int axis;
    };
    
  private:
    Node* root;
    Node* best = nullptr;
    double best_dist = std::numeric_limits<double>::max();
    int dim;
    int Nm; 
};

#endif /* KDTREE_HPP */
