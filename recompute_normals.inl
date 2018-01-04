void recompute_normals(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, Eigen::MatrixXf *image, std::multimap<std::vector<int>, std::vector<float>> *mappy, int max_col, std::vector<float> axis, std::vector<pcl::PointNormal> clus, float theta_app, float phi_app)
{
    for (int i =0; i<transformed_cloud->size(); ++i)
    {
        float r = sqrt((transformed_cloud->points[i].x)*(transformed_cloud->points[i].x)+(transformed_cloud->points[i].y)*(transformed_cloud->points[i].y)+(transformed_cloud->points[i].z)*(transformed_cloud->points[i].z));
        if(r>0.05)
        {
            std::vector<float> dot (clus.size());
            for (int k = 0; k < clus.size(); ++k)
            {
                dot[k] = 1+transformed_cloud->points[i].normal_x*clus[k].x+ transformed_cloud->points[i].normal_y*clus[k].y + transformed_cloud->points[i].normal_z*clus[k].z;
            }
            int index = std::max_element(dot.begin(),dot.end())-dot.begin();

        }

    }
}
