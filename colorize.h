#ifndef COLORIZE
#define COLORIZE

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

void colorize(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, Eigen::MatrixXf *image, std::multimap<std::vector<int>, std::vector<float>> *mappy, int max_col, Eigen::Vector3f axis, std::vector<pcl::PointNormal> clus, float theta_app, float phi_app);

#include "colorize.inl"

#endif // COLORIZE
