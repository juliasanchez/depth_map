#ifndef FILL
#define FILL

#include <iostream>
#include <string>
#include <chrono>
#include <map>

#include "/usr/local/include/Eigen/Core"
#include "/usr/local/include/Eigen/Dense"
#include "/usr/local/include/Eigen/Eigen"

#include "fill.h"

typedef pcl::PointXYZ pcl_point;

void fill(Eigen::MatrixXf *image);

#include "fill.inl"

#endif // FILL
