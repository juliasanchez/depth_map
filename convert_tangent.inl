void convert_tangent(pcl::PointCloud<pcl_point>::Ptr cloud_in, Eigen::MatrixXf *image, std::multimap<std::vector<int>, std::vector<float>> *mappy, int max_col, std::vector<float> axis, float theta_app, float phi_app, int luz)
{
    float alpha = acos(axis[0]);
    if (axis[1]<0)
            alpha=2*M_PI-alpha;
    Eigen::Matrix4f rotation_transform = Eigen::Matrix4f::Identity();
    rotation_transform (0,0)=cos(M_PI-alpha);
    rotation_transform (0,1)=-sin(M_PI-alpha);
    rotation_transform (1,0)=sin(M_PI-alpha);
    rotation_transform (1,1)=cos(M_PI-alpha);
    pcl::PointCloud<pcl_point>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(*cloud_in, *transformed_cloud, rotation_transform);

    int Ncol = image->cols();
    int Nrow = image->rows();
    Eigen::MatrixXf image0(Nrow, Ncol);
    image0 = Eigen::MatrixXf::Zero(Nrow,Ncol);

    float temp=0;
    std::vector <float> vec_r (transformed_cloud->size());

    for (int i =0; i<transformed_cloud->size(); ++i)
    {
        float r = sqrt((transformed_cloud->points[i].x)*(transformed_cloud->points[i].x)+(transformed_cloud->points[i].y)*(transformed_cloud->points[i].y)+(transformed_cloud->points[i].z)*(transformed_cloud->points[i].z));
        vec_r[i]=r;
    }

    std::sort (vec_r.begin(), vec_r.end());
    temp=vec_r[vec_r.size()-luz];
//    temp = 12.0;

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

//            float theta_app = 120*M_PI/180;
//            float phi_app = M_PI/2;

            float offset = abs(1/ ( tan(M_PI/2+theta_app/2)*cos(-phi_app/2)) );
            float ancho = 2 * offset;

            if((phi<phi_app/2 || phi>2*M_PI-phi_app/2) && (theta>M_PI/2-theta_app/2 && theta<M_PI/2+theta_app/2))
            {
                if((int)(r*max_col/temp)<max_col)
                    image0( (int)( (1/(cos(phi)*tan(theta))+offset) * (Nrow-1)/ancho ), (int) (( tan(phi)+1) * (Ncol-1)/2) ) = (int)(r*max_col/temp);
                else
                    image0( (int)( (1/(cos(phi)*tan(theta))+offset) * (Nrow-1)/ancho ), (int) (( tan(phi)+1) * (Ncol-1)/2) ) = max_col;

                std::vector<int> pol(2);
                pol[0] = (int)( (1/(cos(phi)*tan(theta))+offset) * (Nrow-1)/ancho );
                pol[1] = (int) ( (tan(phi)+1) * (Ncol-1)/2 );
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
