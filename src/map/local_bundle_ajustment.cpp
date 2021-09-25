#include "map.h"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "../vision/vision.h"
#include "../parameters.h"
#include "cost_function.h"



//-----------------------------------------------------------------------------------------------------------
std::vector<double> get_index_of_visually_linked_feats(int idx_indirect_vlink,vectorKeyF &KeyFDATA,vectorFeat &AnchorsDATA)
{
  
   std::vector<double> idx;
  for(int i = KeyFDATA.size()-1; i >= idx_indirect_vlink ; i--) // from the last (newest) KeyFrame to the oldest
    {
      
     
      for(int j = 0; j < KeyFDATA[i].MatchedFeats_idx.size();j++)
      {
        int idx_feat = KeyFDATA[i].MatchedFeats_idx[j];        
        
        bool add = true;
        for (int k = 0; k < idx.size(); k++)
        {
           if(idx_feat == idx[k])
           {
             add = false;
             break;
           }
        }
        if (add == true)
        {
          idx.push_back(idx_feat); 
        }                   
         

      } //for(int j = 0; j < KeyFDATA[i].MatchedFeats_idx.size();j++)
     

    }  // for(int i = KeyFDATA.si

    return idx;

}



//------------------------------------------------------------------
void GMAP::local_bundle_ajustment(LOCKS &locks)
{
 
  int idx_indirect_vlink;
  int idx_direct_vlink; 
  check_visibiliy_graph(idx_direct_vlink,idx_indirect_vlink);

  std::vector<double>  vlfeats = get_index_of_visually_linked_feats(idx_indirect_vlink,KeyFDATA,AnchorsDATA);
  
  // create input arrays to the ceres optimazer ------------------------------------------
  std::vector<double> obs_vec;
  std::vector<double> points_vec;
  std::vector<double> cams_vec;
  std::vector<double> cam_par_vec;
  
  std::vector<int> vlKF;
  for(int i =0; i < vlfeats.size(); i++)
  {
     int idx_feat = vlfeats[i];
     points_vec.push_back(AnchorsDATA[idx_feat].AnchorState(0));
     points_vec.push_back(AnchorsDATA[idx_feat].AnchorState(1));     
     points_vec.push_back(AnchorsDATA[idx_feat].AnchorState(2));

     for(int j = 0;j < AnchorsDATA[idx_feat].iKeyFrame.size();j++)
     {  
       int iKF = AnchorsDATA[idx_feat].iKeyFrame[j].KeyF_idx; //  get absolute KF index in KeyFrame

       if((iKF >= idx_indirect_vlink)&&(AnchorsDATA[idx_feat].iKeyFrame[j].MatchedPoint.x > 0)&&(AnchorsDATA[idx_feat].iKeyFrame[j].MatchedPoint.y > 0))
       {
         obs_vec.push_back(AnchorsDATA[idx_feat].iKeyFrame[j].MatchedPoint.x);
         obs_vec.push_back(AnchorsDATA[idx_feat].iKeyFrame[j].MatchedPoint.y); 
         
         bool fvlkf = false;
         if(vlKF.size()==0)
         {
            vlKF.push_back(iKF);
         }
         else
         { 
            for(int k = 0;k < vlKF.size();k++)
            { 
              if(vlKF[k]==iKF)
                fvlkf = true;
            }
            if (fvlkf == false)
              vlKF.push_back(iKF);
         }

       }       

     }

  }  
  // create camera parameter blocks
  std::vector<int> cams_vec_idx;
  for(int i = idx_indirect_vlink; i < KeyFDATA.size(); i++ )
  {   
      bool fKFc = false;  
      for(int k = 0;k < vlKF.size();k++)
         { 
           if(vlKF[k]==i)
           {
                fKFc = true;
                break;
           }     
         }   
      if (fKFc == true)
      {      
        cams_vec.push_back(KeyFDATA[i].CameraAtt(0));
        cams_vec.push_back(KeyFDATA[i].CameraAtt(1));
        cams_vec.push_back(KeyFDATA[i].CameraAtt(2));
        cams_vec.push_back(KeyFDATA[i].CameraAtt(3));
        cams_vec.push_back(KeyFDATA[i].CameraPos(0));
        cams_vec.push_back(KeyFDATA[i].CameraPos(1));
        cams_vec.push_back(KeyFDATA[i].CameraPos(2));
        cams_vec.push_back(0);  // dummy parameter!!  Ceres can be set constant the whole parameter block!!
        cams_vec_idx.push_back(i);
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
  int obs_idx = 0;  
  int fidx = KeyFDATA.size()-(cams_vec.size()/8);
  
  for(int i =0; i < vlfeats.size(); i++)
  {
    int idx_feat = vlfeats[i];
    
    for(int j = 0;j < AnchorsDATA[idx_feat].iKeyFrame.size();j++)
    {
       int iKF = AnchorsDATA[idx_feat].iKeyFrame[j].KeyF_idx; //  get absolute KF index in KeyFrame

       if((iKF >= idx_indirect_vlink)&&(AnchorsDATA[idx_feat].iKeyFrame[j].MatchedPoint.x > 0)&&(AnchorsDATA[idx_feat].iKeyFrame[j].MatchedPoint.y > 0))
       {  
          
          int idx_kf2;
          for (int k=0; k < cams_vec_idx.size(); k++)
          {
            if(cams_vec_idx[k]==iKF)
            {
               idx_kf2 = k; // get relative KF index in cameras
            }
          } 
         // int idx_kf = iKF - idx_indirect_vlink; // get relative KF index in cameras
         

          double *p = points + 3*i;
          double *c = cameras + 8*idx_kf2;
          ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
          //ceres::LossFunction* loss_function = NULL;
        
          /*
          ceres::CostFunction* cost_function =
          ReprojectionErrorWithQuaternions::Create(observations[2*obs_idx  + 0],
                                                        observations[2*obs_idx  + 1]);    
          problem.AddResidualBlock(cost_function,loss_function,c,p,cam_par);  
          */
          
          CostFunction* cost_function = new CostFunction(observations[2*obs_idx  + 0],observations[2*obs_idx  + 1],PAR);
          problem.AddResidualBlock(cost_function,loss_function,c,p);         
        
          obs_idx++;
       } // if(AnchorsDATA[idx_feat].iKeyFrame[j].MatchedPoint.x != 0)                          
    } // for(int j = 0;j < AnchorsDATA[idx_feat].iKeyFrame.size();j++) 
  }  // for(int i =0; i < vlfeats.size(); i++)

//----------------------------------  
   // Set Parameters constants/quaternion
    int n_kf_bundle = (cams_vec.size()/8);    
     
    int q_idx;
    if (idx_direct_vlink == 0)
    {
      q_idx  = idx_direct_vlink - fidx;
    }
    else
    {
      q_idx  = idx_direct_vlink - fidx -1;
    }          

    for (int i = 0; i < n_kf_bundle ;i++ )  
    {

      //if ( i <= n_kf_bundle)  
      if ( i <= q_idx)
      { 
        // Set Constant KeyFrames
        double *c1 = cameras + 8*(i); 
        problem.SetParameterization(c1, new ceres::SubsetParameterization(8, {0, 1,2,3,4,5,6}));  
      }
      else
      { 
        // Set quaternion paramterization for non constant KeyFrames
        double *q = cameras + 8*(i);
        ceres::LocalParameterization* camera_parameterization =
            new ceres::ProductParameterization(
                new ceres::QuaternionParameterization(),
                new ceres::IdentityParameterization(4));
          problem.SetParameterization(q, camera_parameterization); 
      }

    }      
   // problem.SetParameterization(cam_par, new ceres::SubsetParameterization(7, {0, 1,2,3,4,5}));

   //-------------------------------------------------------------------------------------------- 
   //  Solve minimization problem
       
    /*
    std::cout << "cams" << "\n";
    for(int i=0 ; i < cams_vec.size();i++)
    std::cout << cams_vec.at(i) << "\n";
    */     

    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::DENSE_SCHUR;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    //options.gradient_tolerance = 1e-3;
    options.function_tolerance = 1e-4; // default 1e-6
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);      
    std::cout << summary.FullReport() << "\n";
   
   // Vgraph.print();

   //--------------------------------------------------------------------------------------------
   // Update Anchors
   
  locks.Pull_NewAnchorsDATA_mtx.lock();

   Pull_NewAnchorsDATA.clear();    
  
  for(int i =0; i < vlfeats.size(); i++)
  {
     int idx_feat = vlfeats[i];
     arma::vec::fixed<3> pt;
     pt(0) = points_vec[i*3];
     pt(1) = points_vec[i*3 +1]; 
     pt(2) = points_vec[i*3 +2]; 

     if(arma::norm(pt)< 100)
     {
      AnchorsDATA[idx_feat].AnchorState = pt; 
     
        // Put the anchor in the pile to be used by the EKF-SLAM
        Pull_NewAnchorsDATA.push_back(AnchorsDATA[idx_feat]);
     }   
  
  }

    locks.Pull_NewAnchorsDATA_mtx.unlock();

  // Update Keyframe orientation/position 
  
  for(int i = 0; i < cams_vec_idx.size(); i++ )
  {   
     int KFidx = cams_vec_idx[i];
     
     // if (i == (cams_vec.size()/8)-1)
     // {
     /*
       KeyFDATA[KFidx].CameraAtt(0) = cams_vec[i*8];
       KeyFDATA[KFidx].CameraAtt(1) = cams_vec[i*8+1];
       KeyFDATA[KFidx].CameraAtt(2) = cams_vec[i*8+2];
       KeyFDATA[KFidx].CameraAtt(3) = cams_vec[i*8+3];
       KeyFDATA[KFidx].CameraPos(0) = cams_vec[i*8+4];
       KeyFDATA[KFidx].CameraPos(1) = cams_vec[i*8+5];
      KeyFDATA[KFidx].CameraPos(2) = cams_vec[i*8+6];
     */ 
    // }
     // cams_vec.push_back(0);  // dummy parameter!!  Ceres can be set constant the whole parameter block!!
  }
  
  
    int q = 10;
  
 }
 
//-----------------------------------------------------------------------------------------------------------

