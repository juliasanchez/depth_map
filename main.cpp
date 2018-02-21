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
        std::cout<< "file 20 Ncol Nrow max_color radius_for_density_filter luz depth_map_type hokuyo_ou_autre"<<std::endl<<std::endl;
    }



    int hokuyo = atoi(argv[9]);

    std::string file_name = argv[1];
    pcl::PointCloud<pcl_point>::Ptr cloud(new pcl::PointCloud<pcl_point>);

    if( pcl::io::loadPCDFile<pcl::PointXYZ >( file_name, *cloud ) == -1 )
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

    std::cout<< "start initial transformation"<<std::endl;


    Eigen::Matrix4f rotation_transform = Eigen::Matrix4f::Identity();
//        rotation_transform (0,0)=cos(20*M_PI/180);
//        rotation_transform (0,1)=-sin(20*M_PI/180);
//        rotation_transform (1,0)=sin(20*M_PI/180);
//        rotation_transform (1,1)=cos(20*M_PI/180);
    if(hokuyo)
    {
        rotation_transform (0,3) = 0.027;
        rotation_transform (1,3) = 0;
        rotation_transform (2,3) = -0.18;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, rotation_transform);

    std::cout<< "stop initial transformation"<<std::endl;

    // COMPUTE MAIN NORMALS USING GAUSSIAN IMAGE DENSITY--------------------------------------------------------------------------------------------------------

    std::cout<< "start computing normals"<<std::endl;

    std::stringstream sstm;
    std::string file_name1;

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    auto t_bef_normals= std::chrono::high_resolution_clock::now();

    pcl::NormalEstimationOMP<pcl_point, pcl::PointNormal> normal_estimation;
    normal_estimation.setSearchMethod(typename pcl::search::KdTree<pcl_point>::Ptr(new pcl::search::KdTree<pcl_point>));
    normal_estimation.setKSearch(atoi(argv[2]));
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

    auto t_aft_normals= std::chrono::high_resolution_clock::now();

    pcl::io::savePCDFileASCII ("cloud.pcd", *pointNormals);

    std::cout<< "stop computing normals"<<std::endl;
    std::cout<<"total time to compute normals :" <<std::chrono::duration_cast<std::chrono::milliseconds>(t_aft_normals-t_bef_normals).count()<<" milliseconds"<<std::endl<<std::endl;

    std::cout<< "source: points number after preprocessing : "<<pointNormals->size()<<std::endl;

    std::cout<< "start selecting main normals"<<std::endl;

    //1_create a pointcloud representing normals as points on a sphere

    pcl::PointCloud<pcl::PointNormal>::Ptr normals_ball(new pcl::PointCloud<pcl::PointNormal>);
    pcn2pc(pointNormals, normals_ball);

    //2_ filter normals to enhance clusters   HEAVY STEP

    pcl::io::savePCDFileASCII ("normals_ball_before.pcd", *normals_ball);

    float radius = atof(argv[6]);
    std::vector<pcl::PointNormal> clus;
    density_filter(normals_ball, radius, clus);

    std::string file_name_tot2 = "clus.csv";
    ofstream file2 (file_name_tot2);
    for (int i = 0; i < clus.size(); ++i)
    {
        for (int k = 0; k < 1500; ++k)
        {
            file2<<k*clus[i].x/1000<<","<<k*clus[i].y/1000<<","<<k*clus[i].z/1000<<"\n";
        }
    }

    std::vector<pcl::PointNormal> clus_cpy = clus;
    for (int i = 0; i<clus.size(); i++)
    {
        clus_cpy[0]=clus[1];
        clus_cpy[1]=clus[3];
        clus_cpy[2]=clus[5];
        clus_cpy[3]=clus[4];
        clus_cpy[4]=clus[0];
        clus_cpy[5]=clus[2];
    }

    clus = clus_cpy;

    file2.close();
    std::cout<< "main normals found : "<<clus.size()<<std::endl;
    std::cout<< "stop selecting main normals"<<std::endl;


    // CONVERTIR EN CARTE DE PROFONDEUR-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    int depth_map = atoi(argv[8]);
    int max_col =atoi(argv[5]);
    int Nrow=atoi(argv[3]);
    int Ncol=atoi(argv[4]);
    float theta_app = 140;
    theta_app *= M_PI/180;
    float phi_app = 140;
    phi_app *= M_PI/180;

    Eigen::MatrixXf image;
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
        std::cout<< "start points distance projection onto wall planes"<<std::endl;

        std::vector<std::multimap<std::vector<int>, std::vector<float>>> mappy_vec;

        std::vector<Eigen::MatrixXf> image_vec;
        std::vector<float> alpha_vec(clus.size());
        int luz = atoi(argv[7]);

        // projection on detected walls

        for(int i = 0; i<clus.size(); ++i)
        {
            if (clus[i].x<0)
                alpha_vec[i] = (2*M_PI-acos(clus[i].x));
            else
                alpha_vec[i] = acos(clus[i].x);

            if(clus[i].z<-0.97)
            {
                std::cout<<"use of cluster number "<<i<<std::endl<<std::endl;
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
                image_vec.push_back(im);
                save_image_pgm(file_name, "_test", &im, max_col);
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

        image = Eigen::MatrixXf::Zero(image_vec[0].rows(),image_vec[0].cols()*image_vec.size());

        for( int i = 0; i < image_vec.size(); ++i)
        {
            for( int k = 0; k < image_vec[0].rows(); ++k)
            {
                for( int l = 0; l < image_vec[0].cols(); ++l)
                {
                    Eigen::MatrixXf im=image_vec[i];
                    image(k,l+i*image_vec[0].cols()) = im(k,l);
                }
            }
        }
        //Save Image
        if (depth_map==4)
            save_image_ppm(file_name, "", &image, max_col);
        else
            save_image_pgm(file_name, "", &image, max_col);

//        //Creation of conversion map

//        map_gen = mappy_vec[0];
//        for (int i = 1; i < mappy_vec.size(); ++i)
//        {
//            for(std::multimap<std::vector<int>, std::vector<float>>::const_iterator it = mappy_vec[i].begin(); it != mappy_vec[i].end(); ++it)
//            {
//                std::vector<int> pol = it->first;
//                pol[1]=pol[1]+i*Ncol;
//                map_gen.insert(std::make_pair(pol, it->second) );
//            }
//        }

//        std::cout<< "stop projection on wall planes"<<std::endl;
    }

    // FILTRER (MEDIAN) POUR REMPLIR LES PIXELS NOIRS (SANS VALEUR)------------------------------------------------------------------------------------------------------------------------

    Eigen::MatrixXf image_filt (image.rows(),image.cols());
    image_filt = Eigen::MatrixXf::Zero(image.rows(),image.cols());

    median_filter(&image, &image_filt, 1);
    median_filter(&image_filt, &image_filt, 1);
    median_filter(&image_filt, &image_filt, 1);
    median_filter(&image_filt, &image_filt, 1);
    median_filter(&image_filt, &image_filt, 1);
    median_filter(&image_filt, &image_filt, 1);
    median_filter(&image_filt, &image_filt, 0);

    save_image_pgm(file_name, "_filtered", &image_filt, max_col);

//    // SEUILLAGE------------------------------------------------------------------------------------------------------------------------


//    Eigen::MatrixXf image_filt_s (image.rows(), image.cols());
//    image_filt_s = Eigen::MatrixXf::Zero(image.rows(), image.cols());

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trimmed(new pcl::PointCloud<pcl::PointXYZ>);

//    // Fill in the cloud data
//    int n=0;

//    for (int i = 0; i < image.rows(); ++i)
//    {
//        for (int j = 0; j < image.cols(); ++j)
//        {
//            ++n;
//            if(image_filt(i,j) > max_col/2)
//            {
//                image_filt_s(i,j) = image_filt(i,j);
//                std::vector<int> pol = {i , j};
//                std::vector<float> cart;
//                auto it = map_gen.find(pol);
//                if (it != map_gen.end())
//                {
//                    cart = it->second;
//                    pcl::PointXYZ p;
//                    p.x = cart[0];
//                    p.y = cart[1];
//                    p.z = cart[2];
//                    cloud_trimmed->points.push_back(p);
//                }
//                else
//                {
//                    std::cout<<"pas trouvé!"<<std::endl<<std::endl;
//                }
//            }
//        }
//    }

//    cloud_trimmed->width    = cloud_trimmed->points.size();
//    cloud_trimmed->height   = 1;
//    cloud_trimmed->is_dense = false;
//    cloud_trimmed->points.resize (cloud_trimmed->width * cloud_trimmed->height);

//    pcl::io::savePCDFileASCII ("cloud_trimmed.pcd", *cloud_trimmed);

//    save_image_pgm(file_name, "_seuillee", &image_filt_s, max_col);

//    // GRADIENT------------------------------------------------------------------------------------------------------------------------

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
            if (image_s(i,j)>10)
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

//    // Get points on pointcloud laying on openings-----------------------------------------------------------------------------------------------------------------------------------------------

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_open(new pcl::PointCloud<pcl::PointXYZ>);

//    for (int i = 0; i < image.rows(); ++i)
//    {
//        for (int j = 0; j < image.cols(); ++j)
//        {
//            if(image_s(i,j)>50)
//            {
//                std::vector<int> pol = {i , j};
//                std::vector<float> cart;
//                auto it = map_gen.find(pol);
//                if (it != map_gen.end())
//                {
//                    cart = it->second;
//                    pcl::PointXYZ p;
//                    p.x = cart[0];
//                    p.y = cart[1];
//                    p.z = cart[2];
//                    cloud_open->points.push_back(p);
//                }

//            }
//        }
//    }

//    cloud_open->width    = cloud_open->points.size();
//    cloud_open->height   = 1;
//    cloud_open->is_dense = false;
//    cloud_open->points.resize (cloud_open->width * cloud_open->height);

//    pcl::io::savePCDFileASCII ("cloud_open.pcd", *cloud_open);


}

