void convert_tangent_a(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, Eigen::MatrixXf *image, std::multimap<std::vector<int>, std::vector<float>> *mappy, int max_col, Eigen::Vector3f axis, std::vector<pcl::PointNormal> clus, float theta_app, float phi_app)
{
    axis = -axis;
    // depth_map : each color is a cluster on the gaussian image (main normal)
    float alpha = acos(axis.dot(Eigen::Vector3f::UnitX()));
    Eigen::Vector3f rot_axis = axis.cross(Eigen::Vector3f::UnitX());
    rot_axis /= rot_axis.norm();
    Eigen::Affine3f rotation_transform = Eigen::Affine3f::Identity();
    rotation_transform.rotate(Eigen::AngleAxisf (alpha, rot_axis));

    pcl::PointCloud<pcl::PointNormal>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::transformPointCloudWithNormals(*cloud_in, *transformed_cloud, rotation_transform);

    int Ncol = image->cols();
    int Nrow = image->rows();
    Eigen::MatrixXf image0(Nrow, Ncol);
    image0 = Eigen::MatrixXf::Zero(Nrow,Ncol);

    float Ymin = tan(-theta_app/2)/cos(phi_app/2);
    float Xmin = tan(-phi_app/2);
    float Ymax = tan(theta_app/2)/cos(phi_app/2)+0.0001;
    float Xmax = tan(phi_app/2)+0.0001;

    float deltaY = (Ymax - Ymin)/Nrow;
    float deltaX = (Xmax - Xmin)/Ncol;


    for (int i =0; i<transformed_cloud->size(); ++i)
    {
        float norm = sqrt(transformed_cloud->points[i].normal_x*transformed_cloud->points[i].normal_x + transformed_cloud->points[i].normal_y*transformed_cloud->points[i].normal_y + transformed_cloud->points[i].normal_z*transformed_cloud->points[i].normal_z);
        transformed_cloud->points[i].normal_x = transformed_cloud->points[i].normal_x/norm;
        transformed_cloud->points[i].normal_y = transformed_cloud->points[i].normal_y/norm;
        transformed_cloud->points[i].normal_z = transformed_cloud->points[i].normal_z/norm;
    }

    for (int i =0; i<transformed_cloud->size(); ++i)
    {
        float r = sqrt((transformed_cloud->points[i].x)*(transformed_cloud->points[i].x)+(transformed_cloud->points[i].y)*(transformed_cloud->points[i].y)+(transformed_cloud->points[i].z)*(transformed_cloud->points[i].z));
        float theta = asin(transformed_cloud->points[i].z/r);
        float phi = atan2(transformed_cloud->points[i].y,transformed_cloud->points[i].x);
//        if(phi<0)
//        {
//            phi += 2*M_PI;
//        }
        if(r>0.05)
        {
            if((phi<phi_app/2 && phi>-phi_app/2) && (theta<theta_app/2 && theta>-theta_app/2))
            {
                std::vector<float> dot (clus.size());
                for (int k = 0; k < clus.size(); ++k)
                {
                    dot[k] = 1+transformed_cloud->points[i].normal_x*clus[k].x+ transformed_cloud->points[i].normal_y*clus[k].y + transformed_cloud->points[i].normal_z*clus[k].z;
                }
                int index = std::max_element(dot.begin(),dot.end())-dot.begin();
                float X = tan(phi);
                float Y = tan(theta)/cos(phi);
                if(X<Xmin || Y<Ymin || X>Xmax || Y>Ymax)
                {
                    int test = 0;
                }
                image0( (int) ((Y-Ymin)/deltaY), (int) ((X-Xmin)/deltaX) ) = (index+1)*max_col/clus.size();

                std::vector<int> pol(2);
                pol[0] =  (int) ((Y-Ymin)/deltaY) ;
                pol[1] = (int) ((X-Xmin)/deltaX);
                std::vector<float> cart(3);
                cart[0] = cloud_in->points[i].x;
                cart[1] = cloud_in->points[i].y;
                cart[2] = cloud_in->points[i].z;

                mappy->insert(make_pair(pol, cart));
            }
        }

    }

    *image=image0;
}
