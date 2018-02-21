void colorize(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, Eigen::MatrixXf *image, std::multimap<std::vector<int>, std::vector<float>> *mappy, int max_col, Eigen::Vector3f axis, std::vector<pcl::PointNormal> clus, float theta_app, float phi_app)
{
    std::vector< std::vector<float> > color (8, std::vector<float>(3));
    color[0] = {0.5,0.4,0.1};
    color[1] = {0,0,1};
    color[2] = {0,1,0};
    color[3] = {0,1,1};
    color[4] = {1,0,0};
    color[5] = {1,0,1};
    color[6] = {1,1,0};
    color[7] = {1,1,1};

    float alpha = acos(axis.dot(Eigen::Vector3f::UnitX()));
    Eigen::Vector3f rot_axis = axis.cross(Eigen::Vector3f::UnitX());
    rot_axis /= rot_axis.norm();
    Eigen::Affine3f rotation_transform = Eigen::Affine3f::Identity();
    rotation_transform.rotate(Eigen::AngleAxisf (alpha, rot_axis));

    pcl::PointCloud<pcl::PointNormal>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::transformPointCloudWithNormals(*cloud_in, *transformed_cloud, rotation_transform);

    pcl::io::savePCDFileASCII ("cloud.pcd", *transformed_cloud);

    for (int i = 0; i < clus.size(); ++i)
    {
        float x=rotation_transform(0,0)*clus[i].x + rotation_transform(0,1)*clus[i].y + rotation_transform(0,2)*clus[i].z;
        float y=rotation_transform(1,0)*clus[i].x + rotation_transform(1,1)*clus[i].y + rotation_transform(1,2)*clus[i].z;
        float z=rotation_transform(2,0)*clus[i].x + rotation_transform(2,1)*clus[i].y + rotation_transform(2,2)*clus[i].z;
        clus[i].x = x;
        clus[i].y = y;
        clus[i].z = z;
    }

    int Ncol = image->cols();
    int Nrow = image->rows();
    Eigen::MatrixXf image0(Nrow, Ncol);
    image0 = Eigen::MatrixXf::Zero(Nrow,Ncol);

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
        float theta = acos(transformed_cloud->points[i].z/r);
        float phi = abs((atan(transformed_cloud->points[i].y/transformed_cloud->points[i].x)));
        if(r>0.05)
        {
            if(transformed_cloud->points[i].x<0 && transformed_cloud->points[i].y>0)
            {
                phi = M_PI-phi;
            }
            if(transformed_cloud->points[i].x<0 && transformed_cloud->points[i].y<0)
            {
                phi = phi+M_PI;
            }
            if(transformed_cloud->points[i].x>0 && transformed_cloud->points[i].y<0)
            {
                phi = 2*M_PI-phi;
            }

            float offset = abs(1/ ( tan(M_PI/2+theta_app/2)*cos(phi_app/2)) );
            float ancho = 2 * offset;

            if((phi<phi_app/2 || phi>2*M_PI-phi_app/2) && (theta>M_PI/2-theta_app/2 && theta<M_PI/2+theta_app/2))
            {
                std::vector<float> dot (clus.size());
                for (int k = 0; k < clus.size(); ++k)
                {
                    dot[k] = 1+transformed_cloud->points[i].normal_x*clus[k].x+ transformed_cloud->points[i].normal_y*clus[k].y + transformed_cloud->points[i].normal_z*clus[k].z;
                }
                int index = std::max_element(dot.begin(),dot.end())-dot.begin();
                //std::cout<<"y : "<< (int)( (1/(cos(phi)*tan(theta))+offset) * (Nrow-1)/ancho )<<std::endl<<std::endl;
                for (int n = 0; n < 3; ++n)
                    image0( (int)( (1/(cos(phi)*tan(theta))+offset) * (Nrow-1)/ancho ), (int) (( tan(phi)+1) * ((Ncol/3)-1)/2)*3 + n ) = (int) (color[index][n]*max_col);

                std::vector<int> pol(2);
                pol[0] = (int)( (1/(cos(phi)*tan(theta))+offset) * (Nrow-1)/ancho );
                pol[1] = (int) (( tan(phi)+1) * ((Ncol/3)-1)/2)*3;
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
