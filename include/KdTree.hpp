#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <eigen3/Eigen/Core>
#include <iostream>
#include <algorithm>
#include <limits>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <cmath>

/** Node class to handle hold point data and children nodes */
struct Node{
  Node(const Eigen::Vector3d& pt, int lvl) : point(pt), depth(lvl) {} 
  /*
  ~Node(){ 
    delete left;
    left = NULL;
    delete right;
    right = NULL;
  } 
  */
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  std::shared_ptr<Node> left = nullptr;
  std::shared_ptr<Node> right = nullptr;
  int depth = 0;
};


/**
 * Stores root of KD Tree and estimates of current "best" point in terms of 
 * distance (for nn lookup). Has methods for creating tree from vector of points
 * and for doing nn search. 
 */
class KdTree{
  public:

    // Default constructor
    KdTree() = default;

    /** 
     * Takes an Eigen Matrix of points and creates a tree (a KD Tree)
     * with the tree's toplevel node stored as the root member variable
     *
     * @param points the set of points to make a tree of 
     */
    KdTree(const Eigen::MatrixXd& points);

    // Destroys tree and frees memory
    //~KdTree();

    /**
     * Takes the beginning and end of a point set and recursively creates subtrees that
     * split the current dimension in half (according to the midpoint of the data along 
     * that axis)
     *
     * @param pts_begin an iterator of the beginning of a vector of points
     * @param pts_end an iterator to the end of a vector of points
     * @param depth the current depth of the tree   
     */
    std::shared_ptr<Node> make_tree(const std::vector<Eigen::Vector3d>::iterator& pts_begin,
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
    void get_nn(const Eigen::Vector3d& query, std::shared_ptr<Node> T, int depth);  

    // Helper functions
    std::shared_ptr<Node> get_root() const { return root;}
    std::shared_ptr<Node> get_best() const { return best;}
    void reset_best(){ best = nullptr; best_dist = std::numeric_limits<double>::max();}
    
  private:
    std::shared_ptr<Node> root = nullptr;
    std::shared_ptr<Node> best = nullptr;
    double best_dist = std::numeric_limits<double>::max();
    int dim = 3;
    int Nm = 0; 
};

#endif /* KDTREE_HPP */
