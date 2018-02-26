#include <iostream>
#include <fstream>
#include <vector>
#include "convert_cylinder.h"
#include "convert_tangent.h"
#include "convert_tangent_d.h"
#include "convert_tangent_a.h"
#include "colorize.h"
#include "contour.h"
#include "density_filter.h"
#include "pcn2pc.h"
#include "median_filter.h"
#include "save_image_pgm.h"
#include "save_image_ppm.h"
#include "PCL_normEst.h"
#include <chrono>
#include <pcl/features/normal_3d_omp.h>

#define USE_OPENMP_FOR_NORMEST

int vecMed(std::vector<int> vec);

int main(int argc, char *argv[])
{

    if(argc != 10)
    {
        std::cout<< "file neighbors_for_normals_computation radius_for_density_filter Ncol Nrow max_color angle_apperture depth_map_type hokuyo_ou_autre"<<std::endl<<std::endl;
    }

    //pointcloud features---------------------

    int n = 1;
    std::string file_name = argv[n];
    ++n;
    int normals_neighbors = atoi(argv[n]);
    ++n;
    float density_radius = atof(argv[n]);
    ++n;

    //image features---------------------

    int Nrow=atoi(argv[n]);
    ++n;

    int Ncol=atoi(argv[n]);
    ++n;

    float theta_app = atof(argv[n]);    //degrees
    theta_app *= M_PI/180;              //radians
    float phi_app = atof(argv[n]);      //degrees
    phi_app *= M_PI/180;                //radians
    ++n;

    int max_col =atoi(argv[n]);
    ++n;

    //depth map method---------------------

    int depth_map = atoi(argv[n]);
    ++n;

    //first transform if hokuyo device---------------------

    int hokuyo = atoi(argv[n]);
    ++n;

    pcl::PointCloud<pcl_point>::Ptr cloud(new pcl::PointCloud<pcl_point>);

    if( pcl::io::loadPCDFile<pcl::PointXYZ >( file_name, *cloud ) == -1 )
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

    std::cout<< "start initial transformation"<<std::endl;


    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    if(hokuyo)
    {
        transform (0,3) = 0.027;
        transform (1,3) = 0;
        transform (2,3) = -0.18;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    std::cout<< "stop initial transformation"<<std::endl;

    /// COMPUTE MAIN NORMALS USING GAUSSIAN IMAGE DENSITY--------------------------------------------------------------------------------------------------------

    std::cout<< "start computing normals"<<std::endl;

    std::stringstream sstm;
    std::string file_name1;

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    auto t_bef_normals= std::chrono::high_resolution_clock::now();

    pcl::NormalEstimationOMP<pcl_point, pcl::PointNormal> normal_estimation;
    normal_estimation.setSearchMethod(typename pcl::search::KdTree<pcl_point>::Ptr(new pcl::search::KdTree<pcl_point>));
    normal_estimation.setKSearch(normals_neighbors);
    normal_estimation.setViewPoint (0, 0, 0);
    normal_estimation.setInputCloud(transformed_cloud);
    normal_estimation.compute(*pointNormals);

    for (int i=0; i<pointNormals->points.size(); i++)
    {
        pointNormals->points[i].x=transformed_cloud->points[i].x;
        pointNormals->points[i].y=transformed_cloud->points[i].y;
        pointNormals->points[i].z=transformed_cloud->points[i].z;
    }

//    PCL_Normal_Estimator<pcl::PointXYZ,pcl::Normal> NE(transformed_cloud,normals);

//    //change values of the parameters
//    NE.number_of_planes() = 100;
//    NE.normal_selection_mode() = PCL_Normal_Estimator<pcl::PointXYZ,pcl::Normal>::MEAN;
//    NE.rotation_number() = 1;
//    NE.accum_slices() = 100;

//    //estimate the normals
//    NE.estimate(PCL_Normal_Estimator<pcl::PointXYZ,pcl::Normal>::CUBES, PCL_Normal_Estimator<pcl::PointXYZ,pcl::Normal>::RADIUS, 0.1); //changing method
//    pcl::concatenateFields (*transformed_cloud, *normals, *pointNormals);

    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*pointNormals, *pointNormals, indices);

    pcl::io::savePCDFileASCII ("points_with_normals.csv", *pointNormals);

    auto t_aft_normals= std::chrono::high_resolution_clock::now();

    pcl::io::savePCDFileASCII ("cloud.pcd", *pointNormals);

    std::cout<< "stop computing normals"<<std::endl;
    std::cout<<"total time to compute normals :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_aft_normals-t_bef_normals).count()<<" milliseconds"<<std::endl<<std::endl;
    std::cout<< "source: points number after preprocessing : "<<pointNormals->size()<<std::endl<<std::endl;

    std::cout<< "start selecting main normals"<<std::endl;

    //1_create a pointcloud representing normals as points on a sphere

    pcl::PointCloud<pcl::PointNormal>::Ptr normals_ball(new pcl::PointCloud<pcl::PointNormal>);
    pcn2pc(pointNormals, normals_ball);

//    2_ filter normals to enhance clusters   HEAVY STEP

    pcl::io::savePCDFileASCII ("normals_ball_before.pcd", *normals_ball);

    std::vector<pcl::PointNormal> clus;
    density_filter(normals_ball, density_radius, clus);

    std::string file_name_tot2 = "clus.csv";
    ofstream file2 (file_name_tot2);
    for (int i = 0; i < clus.size(); ++i)
    {
        for (int k = 0; k < 1500; ++k)
        {
            file2<<k*clus[i].x/1000<<","<<k*clus[i].y/1000<<","<<k*clus[i].z/1000<<"\n";
        }
    }
    if(file2.is_open())
        file2.close();

    std::cout<< "main normals found : "<<clus.size()<<std::endl;
    std::cout<< "stop selecting main normals"<<std::endl<<std::endl;


    /// CONVERTIR EN CARTES DE PROFONDEUR SUR CHAQUE AXE-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    Eigen::MatrixXf image;
    std::vector<Eigen::MatrixXf> image_vec;
    std::multimap<std::vector<int>, std::vector<float>> map_gen;

    if(depth_map==0) // projection of points distance onto a cylinder
    {
        std::cout<< "start points distance projection onto cylinder "<<std::endl;
        image = Eigen::MatrixXf::Zero(Nrow,Ncol);
        convert_cylinder(cloud, &image, &map_gen, max_col);
        save_image_pgm(file_name, "", &image, max_col);
        std::cout<< "stop points distance projection onto cylinder "<<std::endl;
    }
    else // projection of points distance onto tangent planes (corresponding to walls)
    {
        std::cout<< "-------------Projection onto wall planes----------------"<<std::endl<<std::endl;

        std::vector<std::multimap<std::vector<int>, std::vector<float>>> mappy_vec;
        std::vector<float> alpha_vec(clus.size());
        float luz = 6.0;

        // projection on detected walls

        for(int i = 0; i<clus.size(); ++i)
        {
            if (clus[i].x<0)
                alpha_vec[i] = (2*M_PI-acos(clus[i].x));
            else
                alpha_vec[i] = acos(clus[i].x);

            if(abs(clus[i].z)<0.1 && clus[i].x>0.8) //to select ceiling (clus[i].z<-0.97)/floor or walls can be removed
            {
                std::cout<<"start building image from cluster number "<<i<<" values : "<< clus[i].x<<" "<<clus[i].y<<" "<<clus[i].z<<std::endl;
                std::multimap<std::vector<int>, std::vector<float>> mappy;
                Eigen::MatrixXf im=Eigen::MatrixXf::Zero(Nrow,Ncol);
                Eigen::Vector3f axis = {clus[i].x,clus[i].y,clus[i].z};
                if(depth_map==1) // each pixel = point distance
                    convert_tangent(transformed_cloud, &im, &mappy, max_col, axis, theta_app , phi_app, luz);
                else if (depth_map==2) // each pixel = plane distance
                    convert_tangent_d(pointNormals, &im, &mappy, max_col, axis, theta_app , phi_app, luz);
                else if (depth_map==3) // each pixel = cluster on gaussian image
                    convert_tangent_a(pointNormals, &im, &mappy, max_col, axis, clus, theta_app , phi_app);
                else
                {
                    im=Eigen::MatrixXf::Zero(Nrow,Ncol*3);
                    colorize(pointNormals, &im, &mappy, max_col, axis, clus, 120*M_PI/180, M_PI/2);
//                    save_image_ppm(file_name, "_test", &im, max_col);
                }

                std::cout<<"stop building image from cluster number "<<i<<std::endl<<std::endl;
                image_vec.push_back(im);
                mappy_vec.push_back(mappy);
            }
        }

        // projection on parallel walls if not detected (threshold 11°)

//        for(int i = 0; i<clus.size(); ++i)
//        {
//            std::vector<float> diff(clus.size());
//            for(int j = 0; j<clus.size(); ++j)
//            {
//                float alpha_reversed =  alpha_vec[i] + M_PI;
//                if(alpha_reversed > 2*M_PI)
//                    alpha_reversed =  alpha_reversed-2*M_PI;
//                diff[j] = abs( alpha_reversed - alpha_vec[j]);
//            }

//            auto mini = std::min_element( diff.begin(), diff.end());

//            if(abs(clus[i].z)<0.1 && *mini>0.2)
//            {
//                std::multimap<std::vector<int>, std::vector<float>> mappy;
//                Eigen::MatrixXf im=Eigen::MatrixXf::Zero(Nrow,Ncol);
//                std::vector<float> axis = {-clus[i].x,-clus[i].y,-clus[i].z};
//                if(depth_map==1)
//                    convert_tangent(transformed_cloud, &im, &mappy, max_col, axis, 120*M_PI/180, M_PI/2, luz);
//                else if (depth_map==2)
//                    convert_tangent_d(pointNormals, &im, &mappy, max_col, axis, 120*M_PI/180, M_PI/2, luz);
//                else if (depth_map==3)
//                    convert_tangent_a(pointNormals, &im, &mappy, max_col, axis, clus, 120*M_PI/180, M_PI/2);
//                else
//                {
//                    im=Eigen::MatrixXf::Zero(Nrow,Ncol*3);
//                    colorize(pointNormals, &im, &mappy, max_col, axis, clus, 120*M_PI/180, M_PI/2);
//                }
//                image_vec.push_back(im);
//                mappy_vec.push_back(mappy);


//            }
//        }

        //Creation of whole image with all images concatenation

        std::cout<<"start building and saving total image"<<std::endl;
        image = Eigen::MatrixXf::Zero(image_vec[0].rows(),image_vec[0].cols()*image_vec.size());

        for( int i = 0; i < image_vec.size(); ++i)
        {
            for( int k = 0; k < image_vec[0].rows(); ++k)
            {
                for( int l = 0; l < image_vec[0].cols(); ++l)
                {
                    image(k,l+i*image_vec[0].cols()) = image_vec[i](k,l);
                }
            }
        }

        //Save Image
        if (depth_map==4)
            save_image_ppm(file_name, "", &image, max_col);
        else
            save_image_pgm(file_name, "", &image, max_col);

        std::cout<<"stop building and saving total image"<<std::endl<<std::endl;

        //Creation of conversion map


        std::cout<< "start generating conversion map"<<std::endl;

        map_gen = mappy_vec[0];
        for (int i = 1; i < mappy_vec.size(); ++i)
        {
            for(std::multimap<std::vector<int>, std::vector<float>>::const_iterator it = mappy_vec[i].begin(); it != mappy_vec[i].end(); ++it)
            {
                std::vector<int> pol = it->first;
                pol[1]=pol[1]+i*Ncol;                               //because the global image is the concatenation of various images in y direction
                map_gen.insert(std::make_pair(pol, it->second) );
            }
        }

        std::cout<< "stop generating conversion map"<<std::endl<<std::endl;


    }

    // FILTRER (MEDIAN) POUR REMPLIR LES PIXELS NOIRS (SANS VALEUR)------------------------------------------------------------------------------------------------------------------------

    std::cout<< "start filling black pixels"<<std::endl;
    Eigen::MatrixXf image_filt (image.rows(),image.cols());
    image_filt = Eigen::MatrixXf::Zero(image.rows(),image.cols());
    bool black;

    median_filter(&image, &image_filt, 1, &black);

    while (black)
    {
        median_filter(&image_filt, &image_filt, 1, &black);
    }
//    median_filter(&image_filt, &image_filt, 1, &black);
//    median_filter(&image_filt, &image_filt, 1, &black);
//    median_filter(&image_filt, &image_filt, 1, &black);
//    median_filter(&image_filt, &image_filt, 1, &black);
    median_filter(&image_filt, &image_filt, 0, &black);


    image_filt(210,230) = 0;
    save_image_pgm(file_name, "_filtered", &image_filt, max_col);

    std::cout<< "stop filling black pixels"<<std::endl<<std::endl;

//    // GRADIENT------------------------------------------------------------------------------------------------------------------------

    std::cout<< "start computing boundary"<<std::endl;

    Eigen::MatrixXf image_grad (image.rows(),image.cols());
    image_grad = Eigen::MatrixXf::Zero(image.rows(),image.cols());

    Eigen::MatrixXf image_s (image.rows(),image.cols());
    image_s = Eigen::MatrixXf::Zero(image.rows(),image.cols());

    contour(&image_filt, &image_grad, &image_s, max_col);

    save_image_pgm(file_name, "_gradient", &image_grad, max_col);

    Eigen::MatrixXf image_super (image.rows(),image.cols());
    image_super = Eigen::MatrixXf::Zero(image.rows(),image.cols());

    for (int i = 0; i < image.rows(); ++i)
    {
        for (int j = 0; j < image.cols(); ++j)
        {
            if (image_s(i,j)>200)
            {
                image_super(i,j)=max_col;
            }
            else
            {
                image_super(i,j)=image_filt(i,j);
            }
        }
    }

    save_image_pgm(file_name, "_gradient_superimposed", &image_super, max_col);
    save_image_pgm(file_name, "_gradient_seuille", &image_s, max_col);

    std::cout<< "stop computing boundary"<<std::endl<<std::endl;


//    // Get points on pointcloud laying on planes boundaries-----------------------------------------------------------------------------------------------------------------------------------------------

    std::cout<< "start building boundary pointcloud"<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr hull (new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < image.rows(); ++i)
    {
        for (int j = 0; j < image.cols(); ++j)
        {
            if(image_s(i,j)>0)
            {
                std::vector<int> pol = {i , j};
                std::vector<float> cart;

                int rad = 0;
                int test = 1;
                while (test)
                {
                    std::vector<std::vector<int>> neigh (0, std::vector<int>(2));
                    for (int k = i-rad; k <= i+rad; ++k)
                    {
                        for (int l = j-rad; l <= j+rad; ++l)
                        {
                            neigh.push_back({k,l});
                        }
                    }

                    int ind = 0;
                    while( test && ind < neigh.size() )
                    {
                        if(image_vec[0](neigh[ind][0],neigh[ind][1])>10)
                        {
                            std::pair <std::multimap<std::vector<int>, std::vector<float>>::iterator, std::multimap<std::vector<int>, std::vector<float>>::iterator> ret;
                            ret = map_gen.equal_range(neigh[ind]);
                            for (auto it=ret.first; it!=ret.second; ++it)
                            {
                                cart = it->second;
                                pcl::PointXYZ p;
                                p.x = cart[0];
                                p.y = cart[1];
                                p.z = cart[2];
                                hull->points.push_back(p);
                                test = 0;
                            }
                        }
                        ++ind;
                    }
                    ++rad;
                }
            }
        }
    }

    hull->width    = hull->points.size();
    hull->height   = 1;
    hull->is_dense = false;
    hull->points.resize (hull->width * hull->height);

    if(hokuyo)
    {
        transform (0,3) = -0.027;
        transform (1,3) = -0;
        transform (2,3) = +0.18;

        pcl::transformPointCloud(*hull, *hull, transform);
    }

    pcl::io::savePCDFileASCII ("boundary.csv", *hull);

    std::cout<< "stop building boundary pointcloud"<<std::endl<<std::endl;


}

