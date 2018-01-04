void pcn2pc(pcl::PointCloud<pcl::PointNormal>::Ptr cloudin, pcl::PointCloud<pcl::PointNormal>::Ptr cloudout)
{
    cloudout->width    = cloudin->points.size();
    cloudout->height   = 1;
    cloudout->is_dense = false;
    cloudout->points.resize (cloudin->width * cloudin->height);


    for (int i=0; i<cloudin->points.size(); i++)
    {
        cloudout->points[i].x=cloudin->points[i].normal_x;
        cloudout->points[i].y=cloudin->points[i].normal_y;
        cloudout->points[i].z=cloudin->points[i].normal_z;

        cloudout->points[i].normal_x=cloudin->points[i].x;
        cloudout->points[i].normal_y=cloudin->points[i].y;
        cloudout->points[i].normal_z=cloudin->points[i].z;
    }

}
