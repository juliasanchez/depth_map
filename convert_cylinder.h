#ifndef CONVERT_CYLINDER
#define CONVERT_CYLINDER

#include <iostream>
#include <string>
#include <chrono>
#include <map>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/uniform_sampling.h>

#include "convert_cylinder.h"

typedef pcl::PointXYZ pcl_point;

void convert_cylinder(pcl::PointCloud<pcl_point>::Ptr cloud_in, Eigen::MatrixXf *image, std::multimap<std::vector<int>, std::vector<float>> *mappy, int max_col);

#include "convert_cylinder.inl"

#endif // CONVERT_CYLINDER
