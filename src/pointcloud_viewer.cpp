#include <pointcloud_viewer.hpp>

//TODO: Convert Eigen matrices (of points) to pcl point cloud (pointers)
void eigenToPcl(Eigen::MatrixXd& point_cloud_one, Eigen::MatrixXd& point_cloud_two,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_one_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_two_ptr){

  double* pc_one_data = point_cloud_one.data();
  double* pc_two_data = point_cloud_two.data();

  for(int i = 0; i < (point_cloud_one.rows() * point_cloud_one.cols()); ++i){
  
    pcl::PointXYZ my_point; 
    my_point.x = (float)pc_one_data[i];
    my_point.y = (float)pc_one_data[i+point_cloud_one.rows()];
    my_point.z = (float)pc_one_data[i+2*point_cloud_one.rows()];
    cloud_one_ptr->points.push_back(my_point);
  }
  cloud_one_ptr->width = cloud_one_ptr->size();
  cloud_one_ptr->height=1;

  for(int i = 0; i < (point_cloud_two.rows() * point_cloud_two.cols()); ++i){
  
    pcl::PointXYZ my_point; 
    my_point.x = (float)pc_two_data[i];
    my_point.y = (float)pc_two_data[i+point_cloud_two.rows()];
    my_point.z = (float)pc_two_data[i+2*point_cloud_two.rows()];
    cloud_two_ptr->points.push_back(my_point);
  }
  cloud_two_ptr->width = cloud_two_ptr->size();
  cloud_two_ptr->height=1;
}

//TODO: Create pcl visualization viewer and add two point clouds 
pcl::visualization::PCLVisualizer::Ptr twoCloudVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_one, 
                                                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_two){

  // Create viewer 
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_single_color(cloud_one, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_single_color(cloud_two, 0, 0, 255);

  // add first cloud (colored green)
  viewer->addPointCloud<pcl::PointXYZ> (cloud_one, green_single_color, "Cloud One");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud One");

  // add second cloud (colored blue)
  viewer->addPointCloud<pcl::PointXYZ> (cloud_two, blue_single_color, "Cloud Two");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud Two");

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
