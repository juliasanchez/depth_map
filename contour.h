#ifndef CONTOUR
#define CONTOUR

#include <iostream>
#include <string>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/uniform_sampling.h>


typedef pcl::PointXYZ pcl_point;

void contour(Eigen::MatrixXf *image, Eigen::MatrixXf *image_out, Eigen::MatrixXf *image_s, int max_col);

#include "contour.inl"

#endif // CONTOUR
