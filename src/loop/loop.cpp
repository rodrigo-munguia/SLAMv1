#include "loop.h"
#include "cost_function_cl.h"
#include "../matplotlib/matplotlibcpp.h"
#include "../Transforms/quat2R.h"
#include "../Transforms/Euler_to_Ra2b.h"
namespace plt = matplotlibcpp;

struct MATCH
{
  int number_matchs;
  int KeyFrame_idx;
};

void check_visibiliy_graph(int &idx_direct_link, int &idx_indirect_link, arma::mat &Vgraph );
void matchFeatures4(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches,double ratio); 


bool check_matches(KEYFRAME Current_KF,vectorKeyF &KeyFDATA,int idx_indirect_link, std::vector<MATCH> &match_idx,parameters &par);

bool get_measured_position_of_current_KF(KEYFRAME Current_KF, std::vector<MATCH> &match_idx,vectorFeat AnchorsDATA,parameters &par, arma::vec::fixed<3>  &z_pos,GMAP &gmap );

void update_gmap(KEYFRAME Current_KF, std::vector<MATCH> &match_idx,GMAP &gmap,parameters &par, arma::vec::fixed<3> z_pos_w ,LOCKS &locks);


//------------------------------------------------------------------------------
void LOOP::update(GMAP &gmap,LOCKS &locks)
{
   //static int num_f_since_last_close = 10000; // initial big number
   static auto last_time_close = std::chrono::high_resolution_clock::now(); // only executes the first time

   auto current_time = std::chrono::high_resolution_clock::now();

   auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_time_close);

   if (elapsed.count() > PAR.close_loop.min_time_since_last_close )
   {

        this->newFrame = false; // if a newFrame is received from the EKF then prevent the close loop thread from calling again the Update funtion until the newframe has been proccesed
        
        int idx_direct_link;
        int idx_indirect_link;
        
        
        check_visibiliy_graph(idx_direct_link, idx_indirect_link, gmap.Vgraph );
        
        if(idx_indirect_link > 0 )
        {
            // Loop detection: Search for visual matches between the current KF and previous (old) KFs 
            std::vector<MATCH> matches_idx;
            bool loop_detected = check_matches(this->Current_KF,gmap.KeyFDATA,idx_indirect_link,matches_idx,PAR);
          
          if ( loop_detected == true ) // there is a closing loop match?
          { 
            // Stop EKF from adding new Key Frames
            closing_loop_active = true;
            // Stop Global map procces from updating the global map
            gmap.closing_loop_active = true;
            gmap.loop_detected = true;


            // Based on the visual matches, compute the "actual" position of the current KF     
            bool measurement_available;
            arma::vec::fixed<3>  xyz_pos; 
            measurement_available = get_measured_position_of_current_KF(this->Current_KF, matches_idx,gmap.AnchorsDATA,PAR,xyz_pos,gmap);
            if (measurement_available == true)
              {  
                new_pos_measurement(0) = xyz_pos(0);
                new_pos_measurement(1) = xyz_pos(1);
                new_xy_position_available = true; // flag for EKF to use position measurement

                
                locks.update_anchors_mtx.lock();
                // update global map
                    update_gmap(this->Current_KF, matches_idx,gmap,PAR, xyz_pos ,locks);
                   
                    
                    gmap.Full_update_VG = true; // tell to the global map procces to fully update the visibility graph
               
                   //gmap.Full_Update_Visibility_Graph2(); // update visibility map after global map update
                locks.update_anchors_mtx.unlock();
                
                last_time_close = std::chrono::high_resolution_clock::now();

              } 
              // Resume EKF to add new Key Frames
                closing_loop_active = false;          
                // Resume updating of the global map by the global map proccess 
                gmap.closing_loop_active = false;
          }           
        }


    }
    
   

}
//---------------------------------------------------------------------------------------
 arma::vec::fixed<2> LOOP::get_new_xy_pos()
 {
  new_xy_position_available = false;
  return new_pos_measurement;
 }
//-------------------------------------------------------------------------------------
void update_gmap(KEYFRAME Current_KF, std::vector<MATCH> &match_idx,GMAP &gmap,parameters &par, arma::vec::fixed<3> z_pos_w ,LOCKS &locks)
{
  
  // get the KeyFrame index that has the most matches
    int close_kf_idx = match_idx[0].KeyFrame_idx;
    int nm = 0;
    for(int i = 0; i < match_idx.size(); i++)
    {
      if( match_idx[i].number_matchs >=  nm)
      {
        close_kf_idx = match_idx[i].KeyFrame_idx;
        nm = match_idx[i].number_matchs;
      }
    }

 // Build the problem.
  ceres::Problem problem;

  // create input arrays to the ceres optimizer ------------------------------------------
  std::vector<double> z_ab;
  std::vector<double> xy_ab;
  
   // fill first parameter vector to actual position
   xy_ab.push_back(gmap.KeyFDATA[0].CameraPos(0));  // x_a
   xy_ab.push_back(gmap.KeyFDATA[0].CameraPos(1));  // y_a
  
  // fill rest parameter vector with the previous position
  for(int i = 1; i < gmap.KeyFDATA.size();i++ )
  {    
    xy_ab.push_back(gmap.KeyFDATA[i].CameraPos(0));  // x_a
    xy_ab.push_back(gmap.KeyFDATA[i].CameraPos(1));  // y_a   
  }
    
  // fill observations
  for(int i = 0; i < gmap.KeyFDATA.size()-1; i++)  
  {  
    // observations are equal to xy_b
    z_ab.push_back(gmap.KeyFDATA[i+1].CameraPos(0)-gmap.KeyFDATA[i].CameraPos(0));
    z_ab.push_back(gmap.KeyFDATA[i+1].CameraPos(1)-gmap.KeyFDATA[i].CameraPos(1));
  } 
     
  // add current KeyFrame to paramters 
  //xy_ab.push_back(gmap.KeyFDATA[gmap.KeyFDATA.size()-1].CameraPos(0));  // x_a
  //xy_ab.push_back(gmap.KeyFDATA[gmap.KeyFDATA.size()-1].CameraPos(1));  // y_a 
  xy_ab.push_back(Current_KF.CameraPos(0));  // x_a
  xy_ab.push_back(Current_KF.CameraPos(1));  // y_a  
  
  //std::cout <<  gmap.KeyFDATA[gmap.KeyFDATA.size()-1].CameraPos(0) << std::endl;
  //std::cout <<  Current_KF.CameraPos(0) << std::endl;

  z_ab.push_back(Current_KF.CameraPos(0)-gmap.KeyFDATA[gmap.KeyFDATA.size()-1].CameraPos(0));
  z_ab.push_back(Current_KF.CameraPos(1)-gmap.KeyFDATA[gmap.KeyFDATA.size()-1].CameraPos(1));

    
  // add "visual-odometry" observation to the problem
  double* z = z_ab.data();
  double* pos =  xy_ab.data();
  
  const double std_vo = par.close_loop.std_ab_vo;  // standard deviation "visual odometry" observation
  const double std_cl = par.close_loop.std_ab_cl;  // standard deviation "close loop" observation
  
  Eigen::Matrix<double, 2, 2> sqrt_information_vo;
  sqrt_information_vo(0,0) = 1/std_vo;
  sqrt_information_vo(0,1) = 0;
  sqrt_information_vo(1,0) = 0;
  sqrt_information_vo(1,1) = 1/std_vo;
  Eigen::Matrix<double, 2, 2> sqrt_information_cl;
  sqrt_information_cl(0,0) = 1/std_cl;
  sqrt_information_cl(0,1) = 0;
  sqrt_information_cl(1,0) = 0;
  sqrt_information_cl(1,1) = 1/std_cl;

  
  for(int i = 0; i < ((z_ab.size()+1)/2); i++)
  {
    double *xy_a = pos + 2*i;
    
    double *xy_b = pos + 2*i +2;

    //ceres::LossFunction* loss_function = new ceres::HuberLoss(5);
    ceres::LossFunction* loss_function = NULL;     
     /*
     CostFunctionCL* cost_function = new CostFunctionCL(z[2*i  + 0],z[2*i  + 1],par,0);
          problem.AddResidualBlock(cost_function,loss_function,xy_a,xy_b); 
    */
    ceres::CostFunction* cost_function =
          CostFunctionCL2::Create(z[2*i  + 0],z[2*i  + 1],sqrt_information_vo,0);  
          problem.AddResidualBlock(cost_function,loss_function,xy_a,xy_b);  
  }

  // add "closing-loop" observation to the problem

  // obtaind relative position observation respect to the last KeyFrame
    double z_x = z_pos_w(0) - gmap.KeyFDATA[close_kf_idx].CameraPos(0); 
    double z_y = z_pos_w(1) - gmap.KeyFDATA[close_kf_idx].CameraPos(1);
    //double z_x = z_pos_w(0) - Current_KF.CameraPos(0); 
    //double z_y = z_pos_w(1) - Current_KF.CameraPos(1);
    double z_xy[2] = {z_x,z_y};
    double *zc = z_xy;

    double *xy_a = pos + 2*close_kf_idx;
    //double *xy_a = pos + xy_ab.size()-4;
    double *xy_b = pos + xy_ab.size()-2;

   //ceres::LossFunction* loss_function = new ceres::HuberLoss(5);
   ceres::LossFunction* loss_function = NULL;
   /*
     CostFunctionCL* cost_function = new CostFunctionCL(zc[0],zc[1],par,1);
          problem.AddResidualBlock(cost_function,loss_function,xy_a,xy_b); 
   */  
   
   ceres::CostFunction* cost_function =
          CostFunctionCL2::Create(zc[0],zc[1],sqrt_information_cl,1);  
          problem.AddResidualBlock(cost_function,loss_function,xy_a,xy_b);       

  //- set constant the initial position parameter block  
     double *c1 = pos;
   problem.SetParameterBlockConstant(c1);

  
  
  
  
  // --- Solve graph-SLAM optimization problem 
  ceres::Solver::Options options;   
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = false;
  options.function_tolerance = 1e-6; // default 1e-6
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);  
  // cout << summary.FullReport() << endl;

  
  // Plot closing loop
   std::vector<double> x_i, y_i,x_ie,y_ie,x_ie2,y_ie2;
    
    for(int i=0; i< gmap.KeyFDATA.size();i++)
    { 
        x_i.push_back(gmap.KeyFDATA[i].CameraPos(0));
        y_i.push_back(-gmap.KeyFDATA[i].CameraPos(1));
    } 
    x_i.push_back(Current_KF.CameraPos(0));
    y_i.push_back(-Current_KF.CameraPos(1)); 

    for(int i=0; i< xy_ab.size();i=i+2)
    {   
        if (i < 100)
        {
        x_ie.push_back(xy_ab[i]);
        y_ie.push_back(-xy_ab[i+1]);
        }
        else
        {
          x_ie2.push_back(xy_ab[i]);
        y_ie2.push_back(-xy_ab[i+1]);
        }        
    }    
   /*
    plt::figure(4);      
    plt::plot(x_i,y_i,"y."); 
    plt::plot(x_ie,y_ie,"g.");
    plt::plot(x_ie2,y_ie2,"b.");         
    plt::axis("equal");    
    plt::show();
  */
  // Add new Currrent_KF to the global map

  locks.add_keyframe_mtx.lock();  
       // gmap.NewKeyFDATA.push_back(Current_KF);                
  

  // Update KeyFrames x-y position in global map
  std::vector<arma::vec::fixed<2>> position_corr;

  
  for (int i = 0; i < gmap.KeyFDATA.size();i++)
  { 
    arma::vec::fixed<2> p_c;
    p_c(0) = xy_ab[i*2] - gmap.KeyFDATA[i].CameraPos(0);
    p_c(1) = xy_ab[i*2+1] - gmap.KeyFDATA[i].CameraPos(1);
    gmap.KeyFDATA[i].CameraPos(0) = xy_ab[i*2];
    gmap.KeyFDATA[i].CameraPos(1) = xy_ab[i*2+1];
    position_corr.push_back(p_c) ; // store position correction for each KeyFrame   
  }
  
  // Update Anchors x-y position, and make them availables to the EKF
      // search for index of visually linked KF to close loop point
      std::vector<int> vkf_idx;
      for(int i = 0; i < gmap.Vgraph.n_cols;i++ )
      {
        int ne = gmap.Vgraph(close_kf_idx,i);
        if (ne != 0)
        {
          vkf_idx.push_back(i);
        }
      }
  //close_kf_idx

  for(int i = 0; i < gmap.AnchorsDATA.size();i++)
  {
    
    if (gmap.AnchorsDATA[i].iKeyFrame.size()> 0)
    {
      int mkf = gmap.AnchorsDATA[i].iKeyFrame.size()/2;
      int kf_idx = gmap.AnchorsDATA[i].iKeyFrame[mkf].KeyF_idx;
      
      gmap.AnchorsDATA[i].AnchorState(0) = gmap.AnchorsDATA[i].AnchorState(0) + position_corr[kf_idx](0);
      gmap.AnchorsDATA[i].AnchorState(1) = gmap.AnchorsDATA[i].AnchorState(1) + position_corr[kf_idx](1);

     std::vector<int>::iterator it;

      it = find (vkf_idx.begin(), vkf_idx.end(), kf_idx);
      if (it != vkf_idx.end())
      {
         /// optimize this to only consider anchors "near" to the current camera positiopn
        // gmap.Pull_NewAnchorsDATA.push_back(gmap.AnchorsDATA[i]); // "send" corrected Anchors to EKF
      } 
     
      int q = 10;
    }
  }
  locks.add_keyframe_mtx.unlock();
  

 int q = 10;

}



//-------------------------------------------------------------------------------------
bool get_measured_position_of_current_KF(KEYFRAME Current_KF, std::vector<MATCH> &match_idx,vectorFeat AnchorsDATA,parameters &par , arma::vec::fixed<3>& z_pos ,GMAP &gmap  )
{
  
  // get index and descriptors of anchors associated with the keyframes associated with the loop closure
  std::vector<int> Anchors_predicted_idxs;
  cv::Mat Anchors_Descriptors;

   // search for index of visually linked KF to the matched (old) KeyFrame
   /*
      std::vector<int> vkf_idx;
      for(int i = 0; i < gmap.Vgraph.n_cols;i++ )
      {
        int ne = gmap.Vgraph(close_kf_idx,i);
        if (ne != 0)
        {
          vkf_idx.push_back(i);
        }
      }
  */
  // get the KeyFrame index that has the most matches
    int close_kf_idx = match_idx[0].KeyFrame_idx;
    int nm = 0;
    for(int i = 0; i < match_idx.size(); i++)
    {
      if( match_idx[i].number_matchs >=  nm)
      {
        close_kf_idx = match_idx[i].KeyFrame_idx;
        nm = match_idx[i].number_matchs;
      }
    }
    //----------
  
  for(int i = 0;i < AnchorsDATA.size();i++)
  {
    
    for(int j = 0;j < match_idx.size();j++)
    {
       for(int k = 0;k < AnchorsDATA[i].iKeyFrame.size();k++)
       {
           if(AnchorsDATA[i].iKeyFrame[k].KeyF_idx == match_idx[j].KeyFrame_idx)
           {
             Anchors_predicted_idxs.push_back(i);
             Anchors_Descriptors.push_back(AnchorsDATA[i].Descriptor);
             break;   
           }

       }
        
      // if(AnchorsDATA[i].init_KF == match_idx[j].KeyFrame_idx)
      // {
      //    Anchors_predicted_idxs.push_back(i);
      //    Anchors_Descriptors.push_back(AnchorsDATA[i].Descriptor);          
      // }
    }
  }
  // search the anchors in the current Keyframe
  std::vector<cv::DMatch> matches_p;
  matchFeatures4(Anchors_Descriptors,Current_KF.Descriptors, matches_p,0.6);
   //match (queryDescriptors, trainDescriptors, matches)
  // Use RANSAC (Find Homograpy) to discard outliers
  // matchFeatures4(Current_KF.Descriptors,KeyFDATA[i].Descriptors, matches,0.6);

  std::vector<cv::DMatch> matches;   // Store correct matches in "inliers"
  if (matches_p.size() >par.close_loop.min_matches_for_computing_pos )
  {
   std::vector<cv::Point2f> Points_m2;
   std::vector<cv::Point2f> Points_m1;
        for(unsigned int j =0 ; j< matches_p.size();j++ ) // for each match, update Feature table
        {  
          int Descriptor_idx_m1 = matches_p[j].queryIdx; // get index of the descriptor in 
          int Descriptor_idx_m2 = matches_p[j].trainIdx; // get index of the descriptor in 
          cv::Point2f point_idx_m1 =  Current_KF.keyPoints[Descriptor_idx_m2].pt;
          cv::Point2f point_idx_m2 = AnchorsDATA[Descriptor_idx_m1].Keypoint.pt;
          
          Points_m2.push_back(point_idx_m1);
          Points_m1.push_back(point_idx_m2); 
        }  
        cv::Mat status;
        cv::Mat H = cv::findHomography(Points_m2,Points_m1,cv::RANSAC,10,status);        
        // obtain matches
        
        for(size_t j = 0; j < Points_m2.size(); j++) 
            {
                if(status.at<char>(j) != 0) 
                {
                    matches.push_back(matches_p[j]);
                }
            }
        //    
        //----------------------------------------- 
  }
  else
  {
    return false;
  }      


  std::vector<double> P3D;
  std::vector<double> P2D;
  
  cout << "n of anchors matched: " << matches.size() << endl;
  if (matches.size()>= par.close_loop.min_matches_for_computing_pos) // Minimun number of matches for computing the updated pos in function "get_measured_position_of_current_KF")
  { 
      //Feature point world coordinates
        vector<cv::Point3f> Points3D;
      //The pixel coordinates of the feature point
        vector<cv::Point2f> Points2D;
      for (int i = 0; i < matches.size(); i++)
      {
        int Anchors_predicted_matched_idxs = matches[i].queryIdx;
        int Anchors_matched_idx = Anchors_predicted_idxs[Anchors_predicted_matched_idxs];

            
        double x = AnchorsDATA[Anchors_matched_idx].AnchorState[0]; // used by ceres
        double y = AnchorsDATA[Anchors_matched_idx].AnchorState[1];
        double z = AnchorsDATA[Anchors_matched_idx].AnchorState[2];

        Points3D.push_back(cv::Point3f(x, y, z)); 

        P3D.push_back(x);
        P3D.push_back(y);
        P3D.push_back(z); 
        
        int Current_KF_idx = matches[i].trainIdx;

        //double ud = Current_KF.keyPoints[Current_KF_idx].pt.x;
        //double vd = Current_KF.keyPoints[Current_KF_idx].pt.y;

        cv::Point2d uv;
        uv = Undistort_a_point(Current_KF.keyPoints[Current_KF_idx].pt,par.img.cam_parameters,1);

        Points2D.push_back(cv::Point2f(uv.x, uv.y)); 

        P2D.push_back(uv.x); // used by ceres
        P2D.push_back(uv.y);

      }
      //--------------------------      
      //  Use PnP method from OpenCV for computing camera pose after loop clusure
      
          //Distortion parameter
            double distCoeffD[5] = { 0, 0, 0, 0, 0 }; // The pixel coordinates were previously undistorted
            cv::Mat distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distCoeffD);

          //Initialize the output matrix
            cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
            
            double tvec_ini[3] = {Current_KF.CameraPos[0],-Current_KF.CameraPos[1],-Current_KF.CameraPos[2] }; // consider ned to xyz convertion
            cv::Mat tvec = cv::Mat(3, 1, CV_64FC1,tvec_ini);  


            //Initialize camera parameters Opencv
            const double f_x = par.img.cam_parameters.fc[0];
            const double f_y = par.img.cam_parameters.fc[1];
            const double cx = par.img.cam_parameters.cc[0];
            const double cy = par.img.cam_parameters.cc[1];

            double camD[9] = {
                    f_x, 0,    cx,
                    0,    f_y,  cy,
                    0,      0,      1 };
            cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, camD);
            
            cout << "Computing corrected camera pos" << endl;
            cout << "from:  x:" << tvec.at<double>(0) << "  y:" << tvec.at<double>(1) << endl;
            cv::solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_EPNP); //This method can be used for N-point pose estimation;
            
            // Use solvePnPRansac
            //bool useExtrinsicGuess = false;
            //int iterationsCount = 500;        // number of Ransac iterations.
            //float reprojectionError = 2.0;    // maximum allowed distance to consider it an inlier.
            //float confidence = 0.95;          // RANSAC successful confidence.
            //cv::Mat inliers_idx;
            //cv::solvePnPRansac( Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec,
            //                useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
            //                inliers_idx, cv::SOLVEPNP_ITERATIVE );
            
            cout << "  to:  x:" << tvec.at<double>(0) << "  y:" << tvec.at<double>(1) << endl;
            // consider xyz to ned convertion
           // arma::vec::fixed<3> estimated_position = {tvec.at<double>(0),-tvec.at<double>(1),-tvec.at<double>(2)}; 
        arma::vec::fixed<3> estimated_position2 = {tvec.at<double>(0),-tvec.at<double>(1),Current_KF.CameraPos[2]}; 
        
        //---------------------------------------
        //---- compute x-y  camera pose, after loop closere, with ceres
        /*
            // Build the problem.
          ceres::Problem problem;
          std::vector<double> xy_pos; 
          
          // set initial conditions
          xy_pos.push_back(Current_KF.CameraPos[0]);
          xy_pos.push_back(Current_KF.CameraPos[1]);
         // xy_pos.push_back(-Current_KF.CameraPos[2]);
          

          cout << "Updated camera pose computed" << endl;
          cout << "from: " << xy_pos[0] << " " << xy_pos[1] << " " << xy_pos[2] << endl;

          double* uv = P2D.data();
          double* xyz =  P3D.data();
          double* c_pos = xy_pos.data();
          
          std::vector<double> quat2;
          quat2 =  arma::conv_to<vec_t>::from(Current_KF.CameraAtt); // convert from armadillo vector to c++ vector
          double Rn2c_a2[9];
          quat2R(&quat2[0],Rn2c_a2);
          arma::mat Rn2c2(Rn2c_a2,3,3); // convert from arrat to armadillo
          
          //double Ra2b[9];
          //Euler_to_Ra2b(0, 0, 0, Ra2b);
          //arma::mat Rn2c2(Ra2b,3,3); // convert from arrat to armadillo
          
          
          for (int i = 0; i < ((P2D.size()+1)/2); i++)
          {
              
             //ceres::LossFunction* loss_function = new ceres::HuberLoss(5);
             ceres::LossFunction* loss_function = NULL;     
     
              // x,y
              ceres::CostFunction* cost_function =
                  CostFunctionPNPr::Create(uv[2*i  + 0],uv[2*i  + 1],xyz[3*i  + 0],xyz[3*i  + 1],xyz[3*i  + 2],Rn2c2,Current_KF.CameraPos[2],par);  
                  problem.AddResidualBlock(cost_function,loss_function,c_pos,c_pos+1); 
           
              // x,y,z
             // ceres::CostFunction* cost_function =
             //    CostFunctionPNPr2::Create(uv[2*i  + 0],uv[2*i  + 1],xyz[3*i  + 0],xyz[3*i  + 1],xyz[3*i  + 2],Rn2c2,par);  
             //     problem.AddResidualBlock(cost_function,loss_function,c_pos,c_pos+1,c_pos+2);

             // x,y,x,R   
             // ceres::CostFunction* cost_function =
             //      CostFunctionPNPr3::Create(uv[2*i  + 0],uv[2*i  + 1],xyz[3*i  + 0],xyz[3*i  + 1],xyz[3*i  + 2],par);  
             //      problem.AddResidualBlock(cost_function,loss_function,c_pos,c_pos+1,c_pos+2,Rn2c_a2);         

          }
          ceres::Solver::Options options;   
          //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
          options.minimizer_progress_to_stdout = false;
          //options.function_tolerance = 1e-6; // default 1e-6
          ceres::Solver::Summary summary;
          ceres::Solve(options, &problem, &summary); 
         
         // consider xyz to ned convertion
        arma::vec::fixed<3> estimated_position = {xy_pos[0],xy_pos[1],Current_KF.CameraPos[2]}; 
        
        cout << "to: " << xy_pos[0] << " " << xy_pos[1] << " " << xy_pos[2] << endl;
        */
        //------------------------------
         z_pos = estimated_position2;

         
         //-----------------------------------------
         // test
        bool test_flag_bad = false; 
        for(int i= 0; i < P3D.size() ; i = i+3)
        {
            std::vector<double> quat;
            
            double Rn2c_a[9];
            quat2R(&Current_KF.CameraAtt[0],Rn2c_a);
            arma::mat Rn2c(Rn2c_a,3,3); // convert from arrat to armadillo
            Current_KF.CameraAtt;
            
            //double Ra2b[9];
            //Euler_to_Ra2b(par.init.roll_init, par.init.pitch_init, par.init.GPS_init_yaw, Ra2b);
            //arma::mat Rn2c(Ra2b,3,3); // convert from arrat to armadillo
            
            arma::vec::fixed<3> Tn2c = z_pos;
            arma::vec::fixed<3> pxyz;
            pxyz[0] = P3D[i];
            pxyz[1] = P3D[i+1];
            pxyz[2] = P3D[i+2];            

            arma::vec::fixed<3> pc = Rn2c*(pxyz - Tn2c); // feature point expressed in the camera frame
            arma::mat::fixed<2,3> duv_dPc;
            cv::Point2d uvd = Projection_model(pc,1,false,par.img.cam_parameters,duv_dPc );
            int sm =  0;

            // check if the feat is predicted to appear in the image 
            if((uvd.x > sm)&&(uvd.x < par.img.image_cols - sm)&&(uvd.y > sm)&&(uvd.y < par.img.image_rows - sm))
            {

            }
            else
            {
              test_flag_bad = true;
              break;
            }
        }    




        //------------------------------------------
        // plot results
    /*    
    if(test_flag_bad)
    {
        std::vector<double> x_i,y_i,z_i; 
        for(int i=0;i< gmap.AnchorsDATA.size();i++)
         {             
          x_i.push_back(gmap.AnchorsDATA[i].AnchorState(0));
          y_i.push_back(-gmap.AnchorsDATA[i].AnchorState(1));
          z_i.push_back(-gmap.AnchorsDATA[i].AnchorState(2));            
         }
        std::vector<double> x_m,y_m,z_m;   
        for(int i= 0; i < P3D.size() ; i = i+3)
        {
          x_m.push_back(P3D[i]);
          y_m.push_back(-P3D[i+1]);
          z_m.push_back(-P3D[i+2]);
        } 
        std::vector<double> x_K, y_K, z_K;
        for(int i=0; i < gmap.KeyFDATA.size();i++)
        {
          x_K.push_back(gmap.KeyFDATA[i].CameraPos(0));
          y_K.push_back(-gmap.KeyFDATA[i].CameraPos(1));
          z_K.push_back(-gmap.KeyFDATA[i].CameraPos(2));
        }
        std::vector<double> x_c, y_c, z_c;
        x_c.push_back(Current_KF.CameraPos[0]);
        y_c.push_back(-Current_KF.CameraPos[1]);
        z_c.push_back(-Current_KF.CameraPos[2]);
        //std::vector<double> x_cc, y_cc, z_cc;
        //x_cc.push_back(estimated_position[0]);
        //y_cc.push_back(-estimated_position[1]);
        //z_cc.push_back(-estimated_position[2]);
        std::vector<double> x_cc2, y_cc2, z_cc2;
        x_cc2.push_back(estimated_position2[0]);
        y_cc2.push_back(-estimated_position2[1]);
        z_cc2.push_back(-estimated_position2[2]);
        std::vector<double> x_km, y_km, z_km;
        x_km.push_back(gmap.KeyFDATA[close_kf_idx].CameraPos(0));
        y_km.push_back(-gmap.KeyFDATA[close_kf_idx].CameraPos(1));
        z_km.push_back(-gmap.KeyFDATA[close_kf_idx].CameraPos(2));
        
        plt::figure(5);        
        plt::scatter(x_i,y_i,1,{{"color", "g"}});
        plt::scatter(x_K,y_K,2,{{"color", "r"}});
        plt::scatter(x_km,y_km,8,{{"color", "r"}});
        plt::plot(x_m,y_m,"b.");
        plt::plot(x_c,y_c,"y.");  // current camera 
       // plt::plot(x_cc,y_cc,"k."); // corrected camera
        plt::plot(x_cc2,y_cc2,"k+"); // corrected camera 2        
        plt::axis("equal");

        plt::figure(6);        
        plt::scatter(x_i,z_i,1,{{"color", "g"}});
        plt::scatter(x_K,z_K,2,{{"color", "r"}});
        plt::scatter(x_km,z_km,6,{{"color", "r"}});
        plt::plot(x_m,z_m,"b.");
        plt::plot(x_c,z_c,"y.");  // current camera 
       // plt::plot(x_cc,z_cc,"k."); // corrected camera 
        plt::plot(x_cc2,z_cc2,"k+"); // corrected cameraot been passed        
        plt::axis("equal"); 

         plt::show(); 

    }
    */
        //--------------------------------------------- 
      if(test_flag_bad == true)
      { 
        std::cout << "Estimated camera pos Test failed!" << std::endl; 
        return false; // test has failed
        
      }
      else
      { 
        std::cout << "Estimated camera pos Test succes:  Closing Loop!!" << std::endl; 
        return true; // test has been passed
      } 
        
  }// if (matches.size()>= 4)
  else
  {
    return false;
  }
  


}


//------------------------------------------------------------------
bool check_matches(KEYFRAME Current_KF,vectorKeyF &KeyFDATA,int idx_indirect_link, std::vector<MATCH> &match_idx,parameters &par)
{
  
  //std::vector<MATCH> mts;


  for(int i = 0; i < idx_indirect_link; i++ )
  {

    std::vector<cv::DMatch> matches;

     matchFeatures4(Current_KF.Descriptors,KeyFDATA[i].Descriptors, matches,0.6);
       //match (queryDescriptors, trainDescriptors, matches)
     

      //cout << "matches: "<< matches.size() << endl;
     if(matches.size()> par.close_loop.min_matches_for_potential_loop)
     {
       // Draw matches.
       std::vector<cv::Point2f> Points_m2;
       std::vector<cv::Point2f> Points_m1;

        // Use RANSAC (Find Homograpy) to discard outliers
        for(unsigned int j =0 ; j< matches.size();j++ ) // for each match, update Feature table
        {  
          int Descriptor_idx_m2 = matches[j].queryIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-2]
          int Descriptor_idx_m1 = matches[j].trainIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-1]
          cv::Point2f point_idx_m2 =  Current_KF.keyPoints[Descriptor_idx_m2].pt;
          cv::Point2f point_idx_m1 = KeyFDATA[i].keyPoints[Descriptor_idx_m1].pt;
          Points_m2.push_back(point_idx_m2);
          Points_m1.push_back(point_idx_m1); 
        }  
        cv::Mat status;
        cv::Mat H = cv::findHomography(Points_m2,Points_m1,cv::RANSAC,10,status);
        
        // obtain matches
        std::vector<cv::DMatch> inliers;   // Store correct matches in "inliers"
        for(size_t j = 0; j < Points_m2.size(); j++) 
            {
                if(status.at<char>(j) != 0) 
                {
                    inliers.push_back(matches[j]);
                }
            }
        //    
        //-----------------------------------------
       
       if(inliers.size()> par.close_loop.min_inliers_for_potential_loop)
       {
         cout << "Potential loop detected with KeyFrame: "<< i << "  inliers: " << inliers.size() << endl;
         MATCH mt;
         mt.number_matchs = inliers.size();
         mt.KeyFrame_idx = i;
         match_idx.push_back(mt);
       }

       /*
       cv::Mat img_matches;
        cv::drawMatches( Current_KF.frame, Current_KF.keyPoints, KeyFDATA[i].frame, KeyFDATA[i].keyPoints, inliers, img_matches, cv::Scalar::all(-1),cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
           
        cv::imshow("Good Matches", img_matches );
       cv::waitKey();
       */
      
        

     }
    

  }
  

  if (match_idx.size() >= par.close_loop.min_KeyFrames_with_potential_loop )
  { 
    /*
    // get the KeyFrame index that has the most matches
    int best_kf = mts[0].KeyFrame_idx;
    int nm = 0;
    for(int i = 0; i < mts.size(); i++)
    {
      if( mts[i].number_matchs >=  nm)
      {
        best_kf = mts[i].KeyFrame_idx;
        nm = mts[i].number_matchs;
      }
    }

    return best_kf;
    */
    return true;

  }
  else
  {
    return false;
  }
  


}











//-----------------------------------------------------------------------------------------------------

void check_visibiliy_graph(int &idx_direct_link, int &idx_indirect_link, arma::mat &Vgraph )
{
  idx_direct_link = 0;
  if (Vgraph.n_cols < 4)
  {
    idx_indirect_link = 0;    

  }
  else
  { 

    // check for row of all colums
    if (Vgraph(Vgraph.n_rows-1,Vgraph.n_cols-2) == 0)
    {
       idx_indirect_link = 0;
       return;
    }


    //Vgraph.print();
    bool flag_zero = false;
    //int idx_i;
    for(int i = Vgraph.n_cols-2; i >= 0 ;  i-- )
    {      
      int vlink =  Vgraph(Vgraph.n_rows-1,i);    
      if (vlink == 0)
      {
       flag_zero = true;
       idx_direct_link = i + 1;
       break;        
      }    
    } 
    if (flag_zero == false)
    {
      idx_indirect_link = 0;
    }
    else
    { 
      bool flag_zero2 = false;
      //int idx_j;
      for(int j = idx_direct_link-1; j >= 0 ; j--)
      {
        int vlink =  Vgraph(idx_direct_link,j);
        if (vlink == 0)
          {
          flag_zero2 = true;
          idx_indirect_link  = j + 1;
          break;        
          }            
      }
      if (flag_zero2 == false)
      {
        idx_indirect_link = 0;
      }
      else
      { 
//        int q = 10;
        //return idx_j;
      }
        
    } // else 2
      
  } // else 1
 
}


//-----------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------
//#define RATIO    0.6
//#define RATIO    0.85
void matchFeatures4(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches, double ratio) 
{
    std::vector<std::vector<cv::DMatch>> matches;
    
    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    
    matcher.knnMatch(query, target, matches, 2);
    // Second neighbor ratio test.
    for (unsigned int i = 0; i < matches.size(); ++i) 
    {   
        if (!matches[i].empty())
            if (matches[i][0].distance < matches[i][1].distance * ratio)
                goodMatches.push_back(matches[i][0]);
    }
}