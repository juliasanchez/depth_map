void density_filter(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, float radius, std::vector<pcl::PointNormal>& clus)
{
    pcl::KdTreeFLANN<pcl::PointNormal> tree;
    tree.setInputCloud(cloud_in);

    int N = 50;
    std::map<int, int> index_density;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    for (int i=0; i< cloud_in->size(); i=i+N)
    {
        if ( tree.radiusSearch (cloud_in->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
         {
           index_density[i]= pointIdxRadiusSearch.size();  //key = index, value =density
         }
    }


    //select the points with max density with density threshold (all plans which has a high number of plans (min 1/5 of the maximum plan).

    auto pr = std::max_element (index_density.begin(), index_density.end(), [] (std::pair<int, int> p1, std::pair<int,int> p2) {return p1.second < p2.second;} );
    float max_plan = pr->second;
    int i =0;

    while (pr->second>max_plan/10)
    {
        clus.push_back(cloud_in->points[pr->first]);
        float r = sqrt(clus[i].x*clus[i].x+clus[i].y*clus[i].y+clus[i].z*clus[i].z);
        clus[i].x=clus[i].x/r;
        clus[i].y=clus[i].y/r;
        clus[i].z=clus[i].z/r;

        tree.radiusSearch (cloud_in->points[pr->first], 0.2, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        for (int k=0; k<pointIdxRadiusSearch.size(); k++)
        {
           index_density[pointIdxRadiusSearch[k]]=0;
        } 

        pr = std::max_element (index_density.begin(), index_density.end(), [] (std::pair<int, int> p1, std::pair<int,int> p2) {return p1.second < p2.second;} );
        ++i;
    }


}
