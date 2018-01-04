void contour(Eigen::MatrixXf *image, Eigen::MatrixXf *image_out, Eigen::MatrixXf *image_s, int max_col)
{
    int Ncol = image->cols();
    int Nrow = image->rows();
    Eigen::MatrixXf image_cp = *image;
    Eigen::MatrixXf imagex = Eigen::MatrixXf::Zero(Nrow,Ncol);
    Eigen::MatrixXf imagey = Eigen::MatrixXf::Zero(Nrow,Ncol);
    Eigen::MatrixXf imager = Eigen::MatrixXf::Zero(Nrow,Ncol);
    Eigen::MatrixXf imager_seuil = Eigen::MatrixXf::Zero(Nrow,Ncol);

    float temp=0;

    for (int i = 1; i<Nrow-1; ++i)
    {
        for (int j = 0; j<Ncol; ++j)
        {
            int l=std::max(image_cp(i+1,j),image_cp(i-1,j));
            int p=std::min(image_cp(i+1,j),image_cp(i-1,j));
            int x=image_cp(i,j);
            if(image_cp(i+1,j)!=0 && image_cp(i-1,j)!=0 && image_cp(i,j)!=0 && abs(image_cp(i,j)-p)<abs(image_cp(i,j)-l))
                imagey(i,j) = abs(image_cp(i+1,j)-image_cp(i-1,j));
        }
    }

    for (int i = 0; i<Nrow; ++i)
    {
        for (int j = 1; j<Ncol-1; ++j)
        {
            int l=std::max(image_cp(i,j+1),image_cp(i,j-1));
            int p=std::min(image_cp(i,j+1),image_cp(i,j-1));
            if(image_cp(i,j+1)!=0 && image_cp(i,j-1)!=0 && image_cp(i,j)!=0 && abs(image_cp(i,j)-p)<abs(image_cp(i,j)-l))
                imagex(i,j) = abs(image_cp(i,j+1)-image_cp(i,j-1));
        }
    }

    for (int i = 0; i<Nrow; ++i)
    {
        for (int j = 1; j<Ncol-1; ++j)
        {
            imager(i,j) = sqrt(imagex(i,j)*imagex(i,j)+imagey(i,j)*imagey(i,j));

            if(temp<imager(i,j))
            {
                temp=imager(i,j);
            }
        }
    }

    for (int i = 0; i<Nrow; ++i)
    {
        for (int j = 0; j<Ncol; ++j)
        {
           imager(i,j) = (int)(imager(i,j)*max_col/temp);
           if(imager(i,j)<(int)(max_col/20))
             imager_seuil(i,j)=0;
           else
           {
             imager_seuil(i,j)=max_col;
           }
        }
    }


    *image_out=imager;
    *image_s=imager_seuil;

}
