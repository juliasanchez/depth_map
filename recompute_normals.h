#ifndef RECOMPUTE_NORMALS
#define RECOMPUTE_NORMALS

#include <iostream>
#include <string>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/uniform_sampling.h>


typedef pcl::PointXYZ pcl_point;

void recompute_normals(Eigen::MatrixXf *image, Eigen::MatrixXf *image_out, Eigen::MatrixXf *image_s, int max_col);

#include "recompute_normals.inl"

#endif // RECOMPUTE_NORMALS
