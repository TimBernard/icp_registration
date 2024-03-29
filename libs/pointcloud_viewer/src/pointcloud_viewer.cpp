
#include <pcl/pcl_config.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <eigen3/Eigen/Dense>

#include <pointcloud_viewer.hpp>

using std::cout; 
using std::endl; 

//TODO: Convert Eigen matrix (of points) to pcl point cloud (pointers)
void eigenToPcl(Eigen::MatrixXd& point_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr){

  double* pc_data = point_cloud.data();

  for(int i = 0; i < (point_cloud.rows() * point_cloud.cols()); ++i){
  
    pcl::PointXYZ my_point; 
    my_point.x = (float)pc_data[i];
    my_point.y = (float)pc_data[i+point_cloud.rows()];
    my_point.z = (float)pc_data[i+2*point_cloud.rows()];
    cloud_ptr->points.push_back(my_point);
  }
  cloud_ptr->width = cloud_ptr->size();
  cloud_ptr->height=1;

}

//TODO: Create pcl visualization viewer and add two point clouds 
pcl::visualization::PCLVisualizer::Ptr twoCloudVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr model_cloud, 
                                                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr scene_cloud){

  // Create viewer 
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_single_color(model_cloud, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_single_color(scene_cloud, 0, 0, 255);

  // add first cloud (colored green)
  viewer->addPointCloud<pcl::PointXYZ> (model_cloud, green_single_color, "Model Cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Model Cloud");

  // add second cloud (colored blue)
  viewer->addPointCloud<pcl::PointXYZ> (scene_cloud, blue_single_color, "Scene Cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Scene Cloud");

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


//TODO: This function takes in the model set and the scene set transformed by the current estimate of the best 
//      transformation
void visualizeClouds(Eigen::MatrixXd& model_set, Eigen::MatrixXd& new_scene_set){ 

  cout << "Visualization thread has been started" << endl;

 // Test out visualizaiton  
  // Create and populate two PCL PointCLoud objects from eigen matrices 
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  eigenToPcl(model_set, model_cloud_ptr);
  eigenToPcl(new_scene_set, scene_cloud_ptr);

  // Create viewer and add both PointClouds 
  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = twoCloudVis(model_cloud_ptr, scene_cloud_ptr);

  //cout << "Viewer has been created!" << endl;

  // Run and update viewer while registration is being computed
  while (!viewer->wasStopped())
  {
   // std::cout << "The viewer is running" << std::endl;
    viewer->spinOnce (100);
    
    cout << "#--------------------------------- sceneUpdate: " << (bool)sceneUpdate << endl;
    // If the scene has been updated, update the viewer to reflect that 
    if (sceneUpdate){
      pcl::PointCloud<pcl::PointXYZ>::Ptr update_ptr (new pcl::PointCloud<pcl::PointXYZ>);
      eigenToPcl(new_scene_set,update_ptr);
      viewer->updatePointCloud(update_ptr,"Scene Cloud");
      sceneUpdate = false;
    }
  }
 
}
