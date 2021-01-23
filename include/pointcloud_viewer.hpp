#include <pcl/pcl_config.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


class pointcloud_viewer {
  
  public:
    // Default constructor 
    pointcloud_viewer();

    // Desctructor 
    ~pointcloud_viewer();
    



};
/*
 * cloud viewer class
 *
 * methods: 
 *  take in Eigen matrix (representing a 3xN point cloud) and convert that into a PCL object (resarch to find  which one is best)
 *  take PCL object made in previous step and use that to create a 
 *
 *
 */




