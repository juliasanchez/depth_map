#ifndef PCN2PC
#define PCN2PC

#include<iostream>

typedef pcl::PointXYZ pcl_point;

void pcn2pc(pcl::PointCloud<pcl::PointNormal>::Ptr cloudin, pcl::PointCloud<pcl::PointNormal>::Ptr cloudout);

#include "pcn2pc.inl"

#endif // PCN2PC
