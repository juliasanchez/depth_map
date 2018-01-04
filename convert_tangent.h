#ifndef CONVERT_TANGENT
#define CONVERT_TANGENT

#include <iostream>
#include <string>
#include <chrono>
#include <map>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/uniform_sampling.h>

typedef pcl::PointXYZ pcl_point;

void convert_tangent(pcl::PointCloud<pcl_point>::Ptr cloud_in, Eigen::MatrixXf *image, std::multimap<std::vector<int>, std::vector<float>> *mappy, int max_col, std::vector<float> axis, float theta_app, float phi_app, int luz);

#include "convert_tangent.inl"

#endif // CONVERT_TANGENT
