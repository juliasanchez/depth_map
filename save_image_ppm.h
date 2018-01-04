#ifndef SAVE_IMAGE_PPM
#define SAVE_IMAGE_PPM

#include <iostream>
#include <string>
#include <fstream>

#include "/usr/local/include/Eigen/Core"
#include "/usr/local/include/Eigen/Dense"
#include "/usr/local/include/Eigen/Eigen"

void save_image_ppm(std::string file_name, std::string complement, Eigen::MatrixXf *image, int max_col);

#include "save_image_ppm.inl"

#endif // SAVE_IMAGE_PPM
