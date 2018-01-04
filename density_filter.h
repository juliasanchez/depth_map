#ifndef DENSITY_FILTER
#define DENSITY_FILTER

#include <iostream>
#include <fstream>
#include <cstdlib>
#include "/usr/local/include/Eigen/Core"
#include "/usr/local/include/Eigen/Dense"
#include "/usr/local/include/Eigen/Eigen"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <map>


using namespace std;
typedef pcl::PointXYZ pcl_point;

void density_filter(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, float radius, std::vector<pcl::PointNormal>& clus);

#include "density_filter.inl"

#endif // DENSITY_FILTER
