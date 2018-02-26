void convert_tangent_a(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, Eigen::MatrixXf *image, std::multimap<std::vector<int>, std::vector<float>> *mappy, int max_col, Eigen::Vector3f axis, std::vector<pcl::PointNormal> clus, float theta_app, float phi_app)
{
    axis = -axis;
    // depth_map : each color is a cluster on the gaussian image (main normal)
    float alpha = acos(axis.dot(Eigen::Vector3f::UnitX()));
    Eigen::Vector3f rot_axis = axis.cross(Eigen::Vector3f::UnitX());
    if(rot_axis.norm()<0.1) // pour avoir un truc aligné en XY (pas nécessaire normalement)
    {
        if(alpha>0.1)
        {
            rot_axis = {0,0,1};
            alpha = M_PI;
        }
        else
        {
            rot_axis = {0,0,1};
            alpha = 0;
        }
    }

    rot_axis /= rot_axis.norm();
    Eigen::Affine3f rotation_transform = Eigen::Affine3f::Identity();
    rotation_transform.rotate(Eigen::AngleAxisf (alpha, rot_axis));

    pcl::PointCloud<pcl::PointNormal>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::transformPointCloudWithNormals(*cloud, *transformed_cloud, rotation_transform);

    pcl::io::savePCDFileASCII ("transformed_cloud.csv", *transformed_cloud);
    pcl::io::savePCDFileASCII ("cloud.csv", *cloud);

    axis = -axis;

    int Ncol = image->cols();
    int Nrow = image->rows();
    Eigen::MatrixXf image0(Nrow, Ncol);
    Eigen::MatrixXf image1 = Eigen::MatrixXf::Zero(Nrow, Ncol);
    image0 = Eigen::MatrixXf::Zero(Nrow,Ncol);

//    float Ymin = tan(-theta_app/2)/cos(phi_app/2);
    float Ymin = tan(-theta_app/2);
    float Xmin = tan(-phi_app/2);
//    float Ymax = tan(theta_app/2)/cos(phi_app/2)+0.0001;
    float Ymax = tan(theta_app/2);
    float Xmax = tan(phi_app/2)+0.0001;

    float deltaY = (Ymax - Ymin)/Nrow;
    float deltaX = (Xmax - Xmin)/Ncol;


    for (int i =0; i<transformed_cloud->size(); ++i)
    {
        float norm = sqrt(cloud->points[i].normal_x*cloud->points[i].normal_x + cloud->points[i].normal_y*cloud->points[i].normal_y + cloud->points[i].normal_z*cloud->points[i].normal_z);
        transformed_cloud->points[i].normal_x = transformed_cloud->points[i].normal_x/norm;
        transformed_cloud->points[i].normal_y = transformed_cloud->points[i].normal_y/norm;
        transformed_cloud->points[i].normal_z = transformed_cloud->points[i].normal_z/norm;

        cloud->points[i].normal_x /= norm;
        cloud->points[i].normal_y /= norm;
        cloud->points[i].normal_z /= norm;
    }

    float temp = 0;

    for (int i =0; i<transformed_cloud->size(); ++i)
    {
        float r = sqrt((transformed_cloud->points[i].x)*(transformed_cloud->points[i].x)+(transformed_cloud->points[i].y)*(transformed_cloud->points[i].y)+(transformed_cloud->points[i].z)*(transformed_cloud->points[i].z));
        float theta = asin(transformed_cloud->points[i].z/r);
        float phi = atan2(transformed_cloud->points[i].y,transformed_cloud->points[i].x);

        float X = tan(phi);
        float Y = tan(theta)/cos(phi);

        if(r>0.05)
        {
            if((X<Xmax && X>=Xmin) && (Y<Ymax && Y>=Ymin) && (phi<phi_app/2 && phi>-phi_app/2) && (theta>-theta_app/2 && theta<theta_app/2))
            {

                ///-------------------------------------------------------------------------------------------
                float doti = cloud->points[i].normal_x*axis(0)+ cloud->points[i].normal_y*axis(1) + cloud->points[i].normal_z*axis(2);

                if(doti > 0.9)
                {
                    float distance = abs(cloud->points[i].x*axis(0)+ cloud->points[i].y*axis(1) + cloud->points[i].z*axis(2));
                    if(temp<distance)
                    {
                        temp = distance;
                    }
                    image0( (int) ((Y-Ymin)/deltaY), (int) ((X-Xmin)/deltaX) ) += 1; //Some average is done with the number of points in the pixel belonging to doti<0.9 and doti>0.9 (ie in the current studied plane or not)
                    image1( (int) ((Y-Ymin)/deltaY), (int) ((X-Xmin)/deltaX) ) = distance ;
                }
                else
                {
                    image0( (int) ((Y-Ymin)/deltaY), (int) ((X-Xmin)/deltaX) ) -= 1; //Some average is done with the number of points in the pixel belonging to doti<0.9 and doti>0.9 (ie in the current studied plane or not)
                }

                ///-------------------------------------------------------------------------------------------
//                std::vector<float> dot (clus.size());
//                for (int k = 0; k < clus.size(); ++k)
//                {
//                    dot[k] = 1+cloud->points[i].normal_x*clus[k].x+ cloud->points[i].normal_y*clus[k].y + cloud->points[i].normal_z*clus[k].z;
//                }
//                int index = std::max_element(dot.begin(),dot.end())-dot.begin();
//                float X = tan(phi);
//                float Y = tan(theta)/cos(phi);
//                image0( (int) ((Y-Ymin)/deltaY), (int) ((X-Xmin)/deltaX) ) = (index+1)*max_col/clus.size();

                ///-------------------------------------------------------------------------------------------

                std::vector<int> pol(2);
                pol[0] =  (int) ((Y-Ymin)/deltaY) ; //image(Y,X)
                pol[1] = (int) ((X-Xmin)/deltaX);

                std::vector<float> cart(3);
                cart[0] = cloud->points[i].x;
                cart[1] = cloud->points[i].y;
                cart[2] = cloud->points[i].z;

                mappy->insert(make_pair(pol, cart));
            }
        }

    }

//    float max_distance = image1.maxCoeff();
    float max_distance = temp;

    for( int i = 0; i < image0.rows(); ++i)
    {
        for( int j = 0; j < image0.cols(); ++j)
        {
            if(image0( i, j ) > 0)
            {
                image0( i, j ) = (int) (image1( i, j )*(max_col-20)/max_distance + 20); //each detected plane appear with its distance from origin as color
            }
            else if (image0( i, j ) < 0)
            {
                image0( i, j ) = 10; //other elements appear with color 10
            }
        }
    }

    *image=image0;
}
