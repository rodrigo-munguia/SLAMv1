#include "map.h"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "../vision/vision.h"
#include "../parameters.h"
#include "cost_function.h"


//------------------------------------------------------------------
void GMAP::local_bundle_ajustment2(LOCKS &locks)
{
  static int lastKFoptimized = 0;

  int idx_indirect;
  int idx_direct; 
  check_visibiliy_graph(idx_direct,idx_indirect);
  
  idx_indirect = lastKFoptimized - 1;
  if(idx_indirect < 0)
  { 
    idx_indirect = 0;
    int q = 10;
  }

  //cout << idx_indirect << endl;

  // create input arrays to the ceres optimizer ------------------------------------------
  std::vector<double> obs_vec;
  std::vector<double> points_vec;
  std::vector<double> cams_vec;
  std::vector<double> cam_par_vec;
  
  std::vector<int> obs_anchor_idx;
  std::vector<int> obs_kf_idx;
  std::vector<int> cams_idx;
  std::vector<int> points_idx;
  for(int i = AnchorsDATA.size()-1; i >=0; i-- )
  {  
     bool f_addAnchor = false;
     for(int j = 0; j < AnchorsDATA[i].iKeyFrame.size();j++)
      {
        int idx_kf = AnchorsDATA[i].iKeyFrame[j].KeyF_idx;

        if ((idx_kf >= idx_indirect)&&(AnchorsDATA[i].iKeyFrame[j].matched == true)&&(AnchorsDATA[i].init_type == 1) ) // If idx_kf-KeyFrame must be checked for matches
        {
          obs_vec.push_back(AnchorsDATA[i].iKeyFrame[j].MatchedPoint.x);
          obs_vec.push_back(AnchorsDATA[i].iKeyFrame[j].MatchedPoint.y); 
          f_addAnchor = true;
          obs_anchor_idx.push_back(i);
          obs_kf_idx.push_back(idx_kf);           
        }            
      } // for(int j = 0; j < AnchorsDATA[i].iKeyFrame.size();j++)
      
      if(f_addAnchor==true)
      {
         points_vec.push_back(AnchorsDATA[i].AnchorState(0));
         points_vec.push_back(AnchorsDATA[i].AnchorState(1));     
         points_vec.push_back(AnchorsDATA[i].AnchorState(2));
         points_idx.push_back(i);
      }

  }


  for(int i = idx_indirect; i < KeyFDATA.size(); i++)
  {  
     std::vector<int>::iterator it;
     it = std::find(obs_kf_idx.begin(), obs_kf_idx.end(), i); // check if the KeyFrame is asociated to an observationit 
     
     if (it != obs_kf_idx.end())
     {   
        cams_vec.push_back(KeyFDATA[i].CameraAtt(0));
        cams_vec.push_back(KeyFDATA[i].CameraAtt(1));
        cams_vec.push_back(KeyFDATA[i].CameraAtt(2));
        cams_vec.push_back(KeyFDATA[i].CameraAtt(3));
        cams_vec.push_back(KeyFDATA[i].CameraPos(0));
        cams_vec.push_back(KeyFDATA[i].CameraPos(1));
        cams_vec.push_back(KeyFDATA[i].CameraPos(2));
        cams_vec.push_back(0);  // dummy parameter!!  Ceres can be set constant the whole parameter block!!
        cams_idx.push_back(i);
     }
  }

  // create camera internal parameter block
   cam_par_vec.push_back(PAR.img.cam_parameters.fc[0]);
   cam_par_vec.push_back(PAR.img.cam_parameters.fc[1]);
   cam_par_vec.push_back(PAR.img.cam_parameters.cc[0]);
   cam_par_vec.push_back(PAR.img.cam_parameters.cc[1]);
   cam_par_vec.push_back(PAR.img.cam_parameters.distortions[0]);
   cam_par_vec.push_back(PAR.img.cam_parameters.distortions[1]);
   cam_par_vec.push_back(PAR.img.cam_parameters.alpha_c);

   double* observations = obs_vec.data();
   double* points = points_vec.data();
   double* cameras = cams_vec.data();
   double* cam_par = cam_par_vec.data();

  //--------------------------------------------------------------------
  // Add residuals
  // Build the problem.
  ceres::Problem problem; 

   
  for(int i = 0; i < obs_anchor_idx.size(); i++)
  {
     
    int idx_anchor = obs_anchor_idx[i];
    std::vector<int>::iterator it; 
    it = std::find(points_idx.begin(), points_idx.end(), idx_anchor); // find index of anchor
    int idx_p = it - points_idx.begin();

    int idx_cam = obs_kf_idx[i];
    std::vector<int>::iterator it2; 
    it2 = std::find(cams_idx.begin(), cams_idx.end(), idx_cam); // find index of cam (KF)
    int idx_c = it2 - cams_idx.begin();

    double *p = points + 3*idx_p;
    double *c = cameras + 8*idx_c;

   ceres::LossFunction* loss_function = new ceres::HuberLoss(5);
   //ceres::LossFunction* loss_function = new ceres::CauchyLoss(20);

   // ceres::LossFunction* loss_function = NULL;
    
    CostFunction* cost_function = new CostFunction(observations[2*i  + 0],observations[2*i  + 1],PAR);
          problem.AddResidualBlock(cost_function,loss_function,c,p);
    /*
           ceres::CostFunction* cost_function =
          ReprojectionErrorWithQuaternions::Create(observations[2*i  + 0],
                                                        observations[2*i  + 1]);    
          problem.AddResidualBlock(cost_function,loss_function,c,p,cam_par);  
    */
    

  } 
  //----------------------------------------------------------------------------
  // Set Parameters constants/quaternion
  for(int i = 0; i < cams_idx.size(); i++)
  {
    int c_idx = cams_idx[i];
    std::vector<int>::iterator it2; 
    it2 = std::find(cams_idx.begin(), cams_idx.end(), c_idx); // find index of cam (KF)
    int idx_c = it2 - cams_idx.begin();

    if (i == 0) // always set constant the first (oldest KF)
    {
        // Set Constant KeyFrames+ 8*(idx_c)
         double *c1 = cameras;
        problem.SetParameterization(c1, new ceres::SubsetParameterization(8, {0, 1,2,3,4,5,6})); 
    }
    else
    {
      if (c_idx < idx_direct)
      {
        // Set Constant KeyFrames
        double *c1 = cameras + 8*(idx_c); 
        problem.SetParameterization(c1, new ceres::SubsetParameterization(8, {0, 1,2,3,4,5,6})); 
      }
      else
      {
        // Set quaternion paramterization for non constant KeyFrames
        /*
        double *q = cameras + 8*(idx_c);
        ceres::LocalParameterization* camera_parameterization =
            new ceres::ProductParameterization(
                new ceres::QuaternionParameterization(),
                new ceres::IdentityParameterization(4));
          problem.SetParameterization(q, camera_parameterization); 
          */
         double *c1 = cameras + 8*(idx_c);
        
        if( PAR.sys.GMAP_optimize_Keyframe_pos == true)
        {
          problem.SetParameterization(c1, new ceres::SubsetParameterization(8, {0, 1,2,3}));  
        } 
        else
        {
          problem.SetParameterization(c1, new ceres::SubsetParameterization(8, {0, 1,2,3,4,5,6}));  
        }
        

      }
      
    }
  
  } 
  //---------------------------------------------------------------------------------

  //  Solve minimization problem
       
    /*
    std::cout << "cams" << "\n";
    for(int i=0 ; i < cams_vec.size();i++)
    std::cout << cams_vec.at(i) << "\n";
    */     

    ceres::Solver::Options options;
    
    //options.linear_solver_type = ceres::DENSE_SCHUR;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    //options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    //options.gradient_tolerance = 1e-3;
    options.function_tolerance = 1e-3; // default 1e-6
   // options.max_num_iterations = 100;
   //options.num_threads = 1;
    
   
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);      
    //std::cout << summary.FullReport() << "\n";
    cout << "Optimization!!" << endl;
   
   // Vgraph.print();

  //--------------------------------------------------------------------------------------------
  

  if(closing_loop_active == false)
  {
        
        int idx_indirect_c;
        int idx_direct_c; 
        // search for index of visually linked KF to the last KF
         std::vector<int> vkf_idx;
        for(int i = 0; i < this->Vgraph.n_cols;i++ )
        {
          int ne = this->Vgraph(this->Vgraph.n_cols-1,i);
          if (ne != 0)
          {
          vkf_idx.push_back(i);
          }
        }

        locks.Pull_NewAnchorsDATA_mtx.lock();

          

        for(int i = 0 ; i < points_idx.size();i++)
        {
          int idx_a = points_idx[i];
          arma::vec::fixed<3> pt;
          pt(0) = points_vec[i*3];
          pt(1) = points_vec[i*3 +1]; 
          pt(2) = points_vec[i*3 +2];

          if ((arma::norm(pt) < 100)&&(abs(pt(2)) < 20))
          {  
            if(PAR.sys.GMAP_update_optimized_anchors == true)  
                AnchorsDATA[idx_a].AnchorState = pt;              
            
          }

        }  

        // add anchors, that are visually linked to the last KF, to the buffer (Pull_NewAnchorsDATA) used by the EKF   
        Pull_NewAnchorsDATA.clear();
        int n_anchors_Added = 0; 
        for(int i =0; i < AnchorsDATA.size();i++)
        {
            bool f_add_to_ekf = false;
            for(int j = 0; j < AnchorsDATA[i].iKeyFrame.size();j++)
            {              
              int idx_kf = AnchorsDATA[i].iKeyFrame[j].KeyF_idx;
              std::vector<int>::iterator it;
              it = find (vkf_idx.begin(), vkf_idx.end(), idx_kf);
              if (it != vkf_idx.end())
              {
                  f_add_to_ekf = true;
                  break;
              }  
            } 
            if ((f_add_to_ekf == true)&&(PAR.sys.EKF_use_anchors_from_GMAP==true)&&(loop_detected==true)&&(n_anchors_Added<200))
            {  
              Pull_NewAnchorsDATA.push_back(AnchorsDATA[i]); // "send"optimized  Anchors to EKF
              n_anchors_Added++;
            } 
        }    
        locks.Pull_NewAnchorsDATA_mtx.unlock();


        for(int i = 0; i < cams_idx.size();i++)
        {
          int idx_c = cams_idx[i];
          /* 
            KeyFDATA[idx_c].CameraAtt(0) = cams_vec[i*8];
            KeyFDATA[idx_c].CameraAtt(1) = cams_vec[i*8+1];
            KeyFDATA[idx_c].CameraAtt(2) = cams_vec[i*8+2];
            KeyFDATA[idx_c].CameraAtt(3) = cams_vec[i*8+3];
            */
            arma::vec::fixed<3> pos;
            pos(0) = cams_vec[i*8+4];
            pos(1) = cams_vec[i*8+5];
            pos(2) = cams_vec[i*8+6];
            //pos(2) = KeyFDATA[idx_c].CameraPos(2);
            
            if(PAR.sys.GMAP_update_optimized_Keyframe_pos == true)
            {        
                KeyFDATA[idx_c].CameraPos = pos;
            }
            
            
            if(i==cams_idx.size()-1)
            lastKFoptimized = idx_c;
        }

  }// if(closing_loop_active == false)

        int q = 10;

}    