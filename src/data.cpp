#include "data.hpp"

/**
 * Takes an open file to get the total number of points.
 * Currently works with vtk data format
 *
 * @param file the opened file containing the points
 * @return num_points the number of points 
 */
int getNumPoints(std::ifstream& file){

  int num_points = 0;
  int num_lines = 4;
      
  std::string line;
  for(int i = 0; i < num_lines; ++i){
    getline(file, line);
  }
  std::getline(file, line);
  num_points = std::stoi(line.substr(7,5));
    
  return num_points;  
}

/**
 * Takes an open file and imports points into an Nx3 matrix
 *
 * @param file the opened file containing the points
 * @return point_set the point set in a row major matrix storage format 
 */
Eigen::MatrixX3d getMatrix(std::ifstream& file){
  
  // Find the number of points 
  int num_points = getNumPoints(file);
  //std::cout << "Number of points " << num_points << std::endl;
  file.clear();
  file.seekg(0);
  
  // Information on what line, how many lines to skip
  int count = 0;
  int skip_count = 4;
  std::string line;

  // vector for conversion later
  std::vector<double> coords;
  coords.reserve(num_points*3);

  // Input all points from file 
  while(std::getline(file,line)){
        
    if(count <= skip_count){ count++;  continue; }
    else if(count > (skip_count + num_points)){ break;}
    else{
      std::stringstream line_ss(line);
      double coord;
      while(line_ss >> coord){
	coords.push_back(coord);
      }
      count++;
    }
  }

  // populate matrix from std::vector 
  Eigen::MatrixX3d point_set(num_points,3);
  double* coords_data = coords.data();
  point_set = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> >(coords_data, num_points,3);
  
  return point_set;
}
