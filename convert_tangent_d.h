#ifndef CONVERT_TANGENT_D
#define CONVERT_TANGENT_D

#include <iostream>
#include <string>
#include <chrono>
#include <map>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/uniform_sampling.h>
#include </usr/local/include/Eigen/Dense>
#include </usr/local/include/Eigen/Core>

typedef pcl::PointXYZ pcl_point;

void convert_tangent_d(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, Eigen::MatrixXf *image, std::multimap<std::vector<int>, std::vector<float>> *mappy, int max_col, Eigen::Vector3f axis, float theta_app, float phi_app, int luz);

#include "convert_tangent_d.inl"

#endif // CONVERT_TANGENT_D
