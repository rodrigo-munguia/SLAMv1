#include "g_store.h"



void GSTORE_data_for_plot(parameters &par,LOCKS &locks,GSTORE &store,vectorFeat& AnchorsDATA, vectorKeyF& KeyFDATA  )
{

store.GLOBALmap.release();
store.GLOBALmap_color.release();
store.path.clear();

locks.ReadGlobalMAP_mtx.lock();

 for(int i = 0;i < AnchorsDATA.size();  i++)
    {
        cv::Mat pt(1, 3, CV_64FC3);               
                
        pt.at<double>(0,0) = AnchorsDATA[i].AnchorState(0);
        pt.at<double>(0,1) = -AnchorsDATA[i].AnchorState(1);
        pt.at<double>(0,2) = -AnchorsDATA[i].AnchorState(2);
        store.GLOBALmap.push_back(pt);

        cv::Mat col(1, 3, CV_8UC3);           
        if (AnchorsDATA[i].init_type == 1) // init by global map
            { 
            col.at<char>(0,0) = 51;  // B
            col.at<char>(0,1) = 255;  // G
            col.at<char>(0,2) = 51;  // R              
            } 
        else
            {                
            col.at<char>(0,0) = 51;  // B
            col.at<char>(0,1) = 255;  // G
            col.at<char>(0,2) = 128;  // R              
            }
        store.GLOBALmap_color.push_back(col);          
    }  

    for(int i = 0;i < KeyFDATA.size();  i++)
    {
        
          std::vector<double> quat;
          quat = arma::conv_to<vec_t>::from(KeyFDATA[i].CameraAtt); // convert from armadillo vector to c++ vector
          double Rn2c_a[9];
          quat2R(&quat[0],Rn2c_a);
          
          cv::Vec3f  t_c;
          t_c(0) = KeyFDATA[i].CameraPos(0);
          t_c(1) = -KeyFDATA[i].CameraPos(1);
          t_c(2) = -KeyFDATA[i].CameraPos(2);
          //cv::Matx33f R = cv::Mat(3,3,CV_64FC1,Rn2c_a);
          cv::Mat R = cv::Mat(3,3,CV_64FC1,Rn2c_a);
          cv::Mat Rb = (Mat_<double>(3,3) << 1, 0, 0, 0, -1, 0, 0, 0, -1);

          //cv::Affine3d T(R, t_c);  
          store.path.push_back(cv::Affine3d(Rb*R, t_c));   
    
    }        







locks.ReadGlobalMAP_mtx.unlock();




}