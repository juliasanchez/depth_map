void save_image_ppm(std::string file_name, std::string complement, Eigen::MatrixXf *image, int max_col)
{
    std::stringstream sstm;
    std::string file_name1;
    size_t lastindex_point = file_name.find_last_of(".");
    size_t lastindex_slash = file_name.find_last_of("/");
    if (lastindex_slash==std::string::npos)
    {
       lastindex_slash = 0;
    }

    file_name1 = file_name.substr(lastindex_slash+1, lastindex_point-(lastindex_slash+1));
    sstm.str("");
    sstm<<file_name1<<complement<<".ppm";
    std::string file_name_tot = sstm.str();
    ofstream file (file_name_tot, std::ios::trunc);
    file<<"P3\n";
    file<<image->cols()/3<<" "<<image->rows()<<"\n";
    file<<max_col<<"\n";
    file<<*image;
    file.close();

}
