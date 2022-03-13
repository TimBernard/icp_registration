# icp\_registration

An (in-progress) implementation of iterative closest point using c++

## Build 
After cloning the repository to your local machine: 
```
cd icp_registraion 
mkdir build && cd build 
cmake ../ -DCMAKE_BUILD_TYPE=Release
make # or make -j <however_many_processes> 
```

## Run 
```
cd build 
./main 
```

## Usage
* Currently, main.cpp is an example that loads two loads two mis-aligned point clouds[[1]](#1) and runs icp to return an aligning transformation
* Opens PCL Visualizer simultaneously to visualize alginment of "scene" cloud to "model" cloud
* Has been run on Ubuntu 16.04 and 20.04 

## Dependcies

* PCL 
* Eigen3 
* pthread 

## References 
<a id="1">[1]</a> 
F. Pomerleau, M. Liu, F. Colas, and R. Siegwart, Challenging data sets for point cloud registration algorithms, International Journal of Robotic Research, vol. 31, no. 14, pp. 1705–1711, Dec. 2012.

<a id="2">[2]</a> 
Y.  Liu  and  T.  Bernard.   Programming  assignment  4. Johns  Hopkins  University  Computer Integrated Surgery, 10 2019.

<a id="3">[3]</a> 
https://rosettacode.org/wiki/K-d_tree
