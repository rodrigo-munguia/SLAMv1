#include "store.h"
#include "ekf_types.h"




void STORE_data_for_plot(arma::vec& x, parameters &par,LOCKS &locks,STORE &store, vectorFeat& FeatsDATA, vectorFeat& AnchorsDATA,FRAME *frame)
{


  static int count = 0;

    store.EKFmap.release();
    store.EKFmap_color.release();
    store.ANCHORSmap.release();
    store.ANCHORSmap_color.release();
    store.pose.clear();
    //path.clear();
    
    locks.ekf_run_mtx.lock(); 
        
        store.current_frame = *frame;

        store.FeatsDATA = &FeatsDATA; // copy pointer to struct
        store.AnchorsDATA = &AnchorsDATA; // copy pointer to struct

        for(int i = 0;i < FeatsDATA.size();  i++)
            {
            cv::Mat pt(1, 3, CV_64FC3);    
        
            int idx = FeatsDATA[i].idx_i_state;
            pt.at<double>(0,0) = x(idx);
            pt.at<double>(0,1) = -x(idx+1);
            pt.at<double>(0,2) = -x(idx+2);
            store.EKFmap.push_back(pt);

            cv::Mat col(1, 3, CV_8UC3);           
            if (FeatsDATA[i].matched == true)
            { 
               col.at<char>(0,0) = 0;  // B
               col.at<char>(0,1) = 0;  // G
               col.at<char>(0,2) = 255;  // R              
            } 
            else
            {                
              col.at<char>(0,0) = 204;  // B
              col.at<char>(0,1) = 204;  // G
              col.at<char>(0,2) = 255;  // R              
            }
            store.EKFmap_color.push_back(col);          
        }
        
         //cv::Mat t(1, 3, CV_64FC(16));
         if(count > 10)
         { 
         cv::Point3f t;   
          t.x = x(7);
          t.y = -x(8);
          t.z = -x(9);
          store.EKFtrajectory.push_back(t);
          count = 0;
          
          

         }
         else
         {
            count++;
         }
         
          for(int i = 0;i < AnchorsDATA.size();  i++)
            {
                cv::Mat pt(1, 3, CV_64FC3);               
                
                pt.at<double>(0,0) = AnchorsDATA[i].AnchorState(0);
                pt.at<double>(0,1) = -AnchorsDATA[i].AnchorState(1);
                pt.at<double>(0,2) = -AnchorsDATA[i].AnchorState(2);
                store.ANCHORSmap.push_back(pt);

                cv::Mat col(1, 3, CV_8UC3);           
                if (AnchorsDATA[i].matched == true)
                { 
                col.at<char>(0,0) = 0;  // B
                col.at<char>(0,1) = 255;  // G
                col.at<char>(0,2) = 255;  // R              
                } 
                else
                {                
                col.at<char>(0,0) = 153;  // B
                col.at<char>(0,1) = 255;  // G
                col.at<char>(0,2) = 255;  // R              
                }
                store.ANCHORSmap_color.push_back(col);          
            } 
         
          std::vector<double> quat;
          quat = arma::conv_to<vec_t>::from(x.subvec(0,3)); // convert from armadillo vector to c++ vector
          double Rn2c_a[9];
          quat2R(&quat[0],Rn2c_a);
          
          cv::Vec3f  t_c;
          t_c(0) = x(7);
          t_c(1) = -x(8);
          t_c(2) = -x(9);
          //cv::Matx33f R = cv::Mat(3,3,CV_64FC1,Rn2c_a);
          cv::Mat R = cv::Mat(3,3,CV_64FC1,Rn2c_a);
          cv::Mat Rb = (Mat_<double>(3,3) << 1, 0, 0, 0, -1, 0, 0, 0, -1);

          //cv::Affine3d T(R, t_c);            
          store.pose.push_back(cv::Affine3d(Rb*R, t_c));    


     locks.ekf_run_mtx.unlock();       



}

//-----------------------------------------------------------------------

void Store_gps(GPS &gps,parameters &par,STORE &store)
{

  static bool f_ini = false;
   static arma::vec::fixed<3> xe_ini;
   static arma::mat::fixed<3,3> R_e2t;
   
   double lat = gps.lat/10e6;
   double lon = gps.lon/10e6;
   double h =   gps.alt/10e2;   
   double Gx,Gy,Gz;
   Geo2ECEF(lat,lon,h,Gx,Gy,Gz);

   if(f_ini == false) // initialization
   {
      xe_ini(0) = Gx - par.ekf.AntGPSoffset[0];
      xe_ini(1) = Gy - par.ekf.AntGPSoffset[1];
      xe_ini(2) = Gz - par.ekf.AntGPSoffset[2];
      R_e2t = { {-sin(lat*M_PI/180.0)*cos(lon*M_PI/180.0), -sin(lat*M_PI/180.0)*sin(lon*M_PI/180.0),  cos(lat*M_PI/180.0)},
                {-sin(lon*M_PI/180.0)   ,           cos(lon*M_PI/180.0),       0  },                        
                {-cos(lat*M_PI/180.0)*cos(lon*M_PI/180.0), -cos(lat*M_PI/180.0)*sin(lon*M_PI/180.0), -sin(lat*M_PI/180.0) }};
     // R_e2t.print();
      f_ini = true;
      return; 
   }

   arma::vec::fixed<3> Ant_offset;
   Ant_offset(0) = par.ekf.AntGPSoffset[0];
   Ant_offset(1) = par.ekf.AntGPSoffset[1];
   Ant_offset(2) = par.ekf.AntGPSoffset[2];
   
   arma::vec::fixed<3> Gxyz;
   Gxyz(0) = Gx;
   Gxyz(1) = Gy;
   Gxyz(2) = Gz;

   arma::vec::fixed<3> efcp = (Gxyz-Ant_offset)-xe_ini;
  
    
  double Ra2b[9];
  Euler_to_Ra2b(0, 0, par.init.GPS_init_yaw, Ra2b);
  arma::mat Rn2c(Ra2b,3,3); // convert from arrat to armadillo
  
  
  arma::vec::fixed<3> gps_xyz =  Rn2c*R_e2t*efcp;

  store.x_gps.push_back(gps_xyz(0));
  store.y_gps.push_back(-gps_xyz(1));
  store.z_gps.push_back(-gps_xyz(2));
  
  //gps_xyz.print();

 






}