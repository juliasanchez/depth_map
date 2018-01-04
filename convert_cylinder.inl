void convert_cylinder(pcl::PointCloud<pcl_point>::Ptr cloud_in, Eigen::MatrixXf *image, std::multimap<std::vector<int>, std::vector<float>> *mappy, int max_col)
{

    Eigen::Matrix4f rotation_transform = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl_point>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::transformPointCloud(*cloud_in, *transformed_cloud, rotation_transform);

    int Ncol = image->cols();
    int Nrow = image->rows();
    Eigen::MatrixXf image0(Nrow, Ncol);
    image0 = Eigen::MatrixXf::Zero(Nrow,Ncol);
    float delta1=180.0/(float)Nrow;
    float delta2=360.0/(float)Ncol;
    std::multimap<std::vector<int>, std::vector<float>> mappy0;

    float temp=0;
    std::vector <float> vec_r (transformed_cloud->size());

    for (int i =0; i<transformed_cloud->size(); ++i)
    {
        float r = sqrt((transformed_cloud->points[i].x)*(transformed_cloud->points[i].x)+(transformed_cloud->points[i].y)*(transformed_cloud->points[i].y)+(transformed_cloud->points[i].z)*(transformed_cloud->points[i].z));
//        if(temp<abs(r))
//        {temp=abs(r);}
        vec_r[i]=r;
    }

    std::sort (vec_r.begin(), vec_r.end());
    temp=vec_r[vec_r.size()-5];
//    temp = 12;

    for (int i =0; i<transformed_cloud->size(); ++i)
    {
        float r = sqrt((transformed_cloud->points[i].x)*(transformed_cloud->points[i].x)+(transformed_cloud->points[i].y)*(transformed_cloud->points[i].y)+(transformed_cloud->points[i].z)*(transformed_cloud->points[i].z));
        float theta = acos(transformed_cloud->points[i].z/r)*180/M_PI;
        float phi = abs((atan(transformed_cloud->points[i].y/transformed_cloud->points[i].x))*180/M_PI);
        if(r>0.05)
        {
            if(transformed_cloud->points[i].x<0 && transformed_cloud->points[i].y>0)
            {
                phi = 180-phi;
            }
            if(transformed_cloud->points[i].x<0 && transformed_cloud->points[i].y<0)
            {
                phi = phi+180;
            }
            if(transformed_cloud->points[i].x>0 && transformed_cloud->points[i].y<0)
            {
                phi = 360-phi;
            }

            if((int)(r*max_col/temp)<max_col)
                image0((int)(theta/delta1),(int)(phi/delta2)) = (int)(r*max_col/temp);
            else
                image0((int)(theta/delta1),(int)(phi/delta2)) = max_col;

            std::vector<int> pol(2);
            pol[0]=(int)(phi/delta2);
            pol[1]= (int)(theta/delta1);
            std::vector<float> cart(3);
            cart[0] = transformed_cloud->points[i].x;
            cart[1] = transformed_cloud->points[i].y;
            cart[2] = transformed_cloud->points[i].z;

            mappy0.insert(make_pair(pol, cart));

        }
    }

    *image=image0;
    *mappy=mappy0;
}
