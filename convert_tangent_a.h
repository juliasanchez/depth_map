#ifndef CONVERT_TANGENT_A
#define CONVERT_TANGENT_A

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

void convert_tangent_a(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, Eigen::MatrixXf *image, std::multimap<std::vector<int>, std::vector<float>> *mappy, int max_col, std::vector<float> axis, std::vector<pcl::PointNormal> clus, float theta_app, float phi_app);

#include "convert_tangent_a.inl"

#endif // CONVERT_TANGENT_A
