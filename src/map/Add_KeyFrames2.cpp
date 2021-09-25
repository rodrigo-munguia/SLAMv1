#include "map.h"
#include "../Transforms/quat2R.h"
#include "../vision/vision.h"
#include <math.h>

void matchFeatures2b(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches );
// convert an Armadillo matrix to OpenCV matrix. NOTE: no copy is made
template <typename T>
cv::Mat_<T> to_cvmat(const arma::Mat<T> &src)
{
  return cv::Mat_<double>{int(src.n_cols), int(src.n_rows), const_cast<T*>(src.memptr())};
}

//---------------------------------------------------------------------------------------------------
// Rodrigo M. 2020
//-----------------------------------------------------------------------------------------------------
bool GMAP::Add_KeyFrames2()
{
    bool new_keyFrame = false;

    // Add new Keyframe to KeyFrame structure 
    
    
     bool ft = false;
     NewKeyFDATAtmp.clear();
    for(int k = 0; k < NewKeyFDATA.size();k++  )
        {   
          if(ft == false)
          {
              cout << NewKeyFDATA.size() << endl;
              ft = true;  
          }
            

            NewKeyFDATAtmp.push_back(NewKeyFDATA[k]);
            NewKeyFDATA.erase(NewKeyFDATA.begin() + k);
            k--;
            new_keyFrame = true;
            
        }

        for(int k = 0; k < NewKeyFDATAtmp.size();k++  )
        { 
            KeyFDATA.push_back(NewKeyFDATAtmp[k]);
            if (KeyFDATA.size()>1)
                {  
                                      
                    Init_new_anchors2(); // Initialize new features
                   
                    Update_Visibility_Graph2(); // Update visibility Graph

                    //visual_match2();
                    
                }
           
        }
       

    
    

    



    return new_keyFrame;
}
//-------------------------------------------------------------------------------
void GMAP::Init_new_anchors2()
{
        std::vector<cv::DMatch> matches;
       // matchFeatures2(KeyFDATA[KeyFDATA.size()-2].Descriptors, descriptors_sscKP, matches);
        
        matchFeatures2b(KeyFDATA[KeyFDATA.size()-2].Descriptors, KeyFDATA[KeyFDATA.size()-1].Descriptors, matches);
        
        std::vector<cv::Point2f> Points_m2;
        std::vector<cv::Point2f> Points_m1;

        
        // Use RANSAC (Find Homograpy) to discard outliers
        for(unsigned int i =0 ; i< matches.size();i++ ) // for each match, update Feature table
        {  
          int Descriptor_idx_m2 = matches[i].queryIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-2]
          int Descriptor_idx_m1 = matches[i].trainIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-1]
          cv::Point2f point_idx_m2 = KeyFDATA[KeyFDATA.size()-2].keyPoints[Descriptor_idx_m2].pt;
          cv::Point2f point_idx_m1 = KeyFDATA[KeyFDATA.size()-1].keyPoints[Descriptor_idx_m1].pt;
          Points_m2.push_back(point_idx_m2);
          Points_m1.push_back(point_idx_m1); 
        }  
        
        cv::Mat status;
        cv::Mat H = cv::findHomography(Points_m2,Points_m1,cv::RANSAC,10,status);
        
        // obtain matches
        std::vector<cv::DMatch> inliers;   // Store correct matches in "inliers"
        for(size_t i = 0; i < Points_m2.size(); i++) 
            {
                if(status.at<char>(i) != 0) 
                {
                    inliers.push_back(matches[i]);
                }
            }
        //    
        //-----------------------------------------
        std::vector<cv::Point2f> P_m2;
        std::vector<cv::Point2f> P_m1;
        // obtain matches
        std::vector<cv::DMatch> f_matches;

        // check minimun distance
          int idx_indirect;
          int idx_direct; 
          check_visibiliy_graph(idx_direct,idx_indirect);
          
          
          int n_anchors_in_v = 0;
          for(int i = AnchorsDATA.size()-1; i >= 0; i--)
          {            
            for(int j =0 ; j < AnchorsDATA[i].iKeyFrame.size();j++)
            {
              int KF_idx = AnchorsDATA[i].iKeyFrame[j].KeyF_idx;
              if ((KF_idx >= idx_indirect)&&(AnchorsDATA[i].iKeyFrame[j].matched == true) )
              {
                n_anchors_in_v++;
                break;
              }

            }

          }
          
          
          int n_VG = Vgraph.n_rows;
          
          int n_vlinks = 0;
          for(int i = 0; i < n_VG;i++)
          {
            n_vlinks = n_vlinks + Vgraph(n_VG-1,i);   
          }
          
         // cout << n_anchors_in_v << endl;
          
          double m_dis;
          if(n_vlinks > 200) // 200
          {
             m_dis= .1*n_vlinks + 1; //   .05  minumun distance
          }   
          else
          {
             m_dis = 1;
          }
          
         // m_dis = 5;


        for(unsigned int i =0 ; i< inliers.size();i++ ) // for each match, update Feature table
        { 
          int Descriptor_idx_m2 = inliers[i].queryIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-2]
          int Descriptor_idx_m1 = inliers[i].trainIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-1]
          cv::Point2f point_idx_m2 = KeyFDATA[KeyFDATA.size()-2].keyPoints[Descriptor_idx_m2].pt;
          cv::Point2f point_idx_m1 = KeyFDATA[KeyFDATA.size()-1].keyPoints[Descriptor_idx_m1].pt;
          
          cv::Point2d uv1,uv2;
          int ditortion_model = 1;
          uv1 = Undistort_a_point( point_idx_m1,PAR.img.cam_parameters,ditortion_model );  // undistort point
          uv2 = Undistort_a_point( point_idx_m2,PAR.img.cam_parameters,ditortion_model );  // undistort point
          
          
          bool md_flag = true;
          for (int j = 0; j < P_m1.size();j++)
          {
            double d = sqrt( pow(uv1.x - P_m1[j].x,2) + pow(uv1.y - P_m1[j].y,2) );
            if (d < m_dis)
            {
              md_flag = false;
              break;
            }   

          }

          if (md_flag == true)
          {
            P_m2.push_back(uv2);  // Undistorted Points in KetFDATA[KeyFDATA.size()-2]
            P_m1.push_back(uv1);   // Undistorted Points in KetFDATA[KeyFDATA.size()-1]
            f_matches.push_back(inliers[i]);
          }
        
        }
        //-------compute projection matrices
        double fc1 = PAR.img.cam_parameters.fc[0];
        double fc2 = PAR.img.cam_parameters.fc[1];
        double cc1 = PAR.img.cam_parameters.cc[0];
        double cc2 = PAR.img.cam_parameters.cc[1];
        double alpha_c = PAR.img.cam_parameters.alpha_c;

        arma::mat::fixed<3,3>  KK = {{fc1, alpha_c*fc2, cc1},
                                   { 0 ,    fc2  ,    cc2},
                                   { 0 ,      0  ,    1  }};

        arma::mat::fixed<3,3>  Rn = {{1, 0, 0},
                                   { 0 ,    -1  ,    0},
                                   { 0 ,      0  ,    -1  }};                           


        std::vector<double> quat1;
        quat1 =  arma::conv_to<vec_t>::from(KeyFDATA[KeyFDATA.size()-1].CameraAtt); // convert from armadillo vector to c++ vector
        double Rn2c_a1[9];
        quat2R(&quat1[0],Rn2c_a1);
        arma::mat Rn2c1(Rn2c_a1,3,3); // convert from arrat to armadillo
        arma::vec::fixed<3> Tn2c1 = KeyFDATA[KeyFDATA.size()-1].CameraPos;
       
        
        arma::mat::fixed<3,4> RT1;
        RT1(arma::span(0,2),arma::span(0,2)) = Rn2c1;
        RT1.col(3) = Tn2c1;

        std::vector<double> quat2;
        quat2 =  arma::conv_to<vec_t>::from(KeyFDATA[KeyFDATA.size()-2].CameraAtt); // convert from armadillo vector to c++ vector
        double Rn2c_a2[9];
        quat2R(&quat2[0],Rn2c_a2);
        arma::mat Rn2c2(Rn2c_a2,3,3); // convert from arrat to armadillo
        arma::vec::fixed<3> Tn2c2 = KeyFDATA[KeyFDATA.size()-2].CameraPos;      
        
        arma::mat::fixed<3,4> RT2;
        RT2(arma::span(0,2),arma::span(0,2)) = Rn2c2;
        RT2.col(3) = Tn2c2;
                
        arma::mat Pm1a = (KK*RT1).t();     // transpose to convert to OpenCV mat   
        arma::mat Pm2a = (KK*RT2).t();

        cv::Mat Pm1 = to_cvmat(Pm1a);  // projection matrix 1
        cv::Mat Pm2 = to_cvmat(Pm2a);  // projection matrix 1
        //-----------------------------------------------------
        

        cv::Mat pnts3D(4,P_m1.size(),CV_32F);

        cv::triangulatePoints(Pm1,Pm2,P_m1,P_m2,pnts3D);

        
       
        for(int i = 0; i < pnts3D.cols; i++ )
        {   
            int Descriptor_idx_m2 = f_matches[i].queryIdx; // get index of the descriptor in KetFDATA[KeyFDATA.size()-2]

            FEAT Anchor;
            arma::vec::fixed<3> pt;
            pt(0) = -pnts3D.at<float>(0,i)/pnts3D.at<float>(3,i);
            pt(1) = -pnts3D.at<float>(1,i)/pnts3D.at<float>(3,i); // convert to NED coordinates
            pt(2) = -pnts3D.at<float>(2,i)/pnts3D.at<float>(3,i); // convert to NED coordinates
            
            Anchor.AnchorState = Rn2c1*pt;
            
            Anchor.idx_i_state = -1;
            Anchor.idx_f_state = -1;
            Anchor.ekf_to_gmap_status = 1;
            Anchor.init_type = 1;   // initialized by the global map
            Anchor.id_anchor = -1;  // to be set later by the EKF
           Anchor.Descriptor = KeyFDATA[KeyFDATA.size()-2].Descriptors.row(Descriptor_idx_m2);
           Anchor.Keypoint = KeyFDATA[KeyFDATA.size()-2].keyPoints[Descriptor_idx_m2];
           Anchor.Initial_KeyPoint  = Anchor.Keypoint;
           Anchor.times_mathed = 1;
           Anchor.times_not_considered = 0;
           Anchor.times_not_mathed = 0;
           Anchor.CameraState = KeyFDATA[KeyFDATA.size()-2].CameraPos;
           Anchor.init_KF = KeyFDATA.size()-2;

           AnchorsDATA.push_back(Anchor); 
        

        }


       /*
        // Draw matches.
        cv::Mat img_matches;
        cv::drawMatches( KeyFDATA[KeyFDATA.size()-2].frame, KeyFDATA[KeyFDATA.size()-2].keyPoints, KeyFDATA[KeyFDATA.size()-1].frame, KeyFDATA[KeyFDATA.size()-1].keyPoints, f_matches, img_matches, cv::Scalar::all(-1),
        cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
             
       //-- Show detected matches
        cv::imshow("Good Matches", img_matches );
        cv::waitKey();
      */

      // Add new anchors obtained from the EKF-SLAM
    if(PAR.sys.GMAP_use_anchors_from_EKF == true)
    {  
      for(unsigned int j= 0; j < Push_NewAnchorsDATA.size(); j++ )
        {
          NewAnchorsDATA.push_back(Push_NewAnchorsDATA[j]);  // copy new anchor to  main anchors (map) structure        
          Push_NewAnchorsDATA.erase(Push_NewAnchorsDATA.begin() + j);        
          j--;             
        }
    }
    // Add new anchors obtained from the EKF-SLAM
    for(unsigned int j= 0; j < NewAnchorsDATA.size(); j++ )
      {
        AnchorsDATA.push_back(NewAnchorsDATA[j]);  // copy new anchor to  main anchors (map) structure
        NewAnchorsDATA.erase(NewAnchorsDATA.begin() + j);
        j--;             
      }         


}

//-------------------------------------------------------------------------------
void GMAP::Update_Visibility_Graph2()
{

    //int vg_size = Vgraph.n_rows; // size of visibility graph
    int vg_size = KeyFDATA.size();
    Vgraph.resize(vg_size,vg_size); // increase size of visibility graph by one
    Vgraph(vg_size-1,vg_size-1) = 0;

    arma::rowvec vg_row;
    vg_row.zeros(vg_size);

    for(int i = AnchorsDATA.size()-1; i >= 0; i--)
    {
        AnchorsDATA[i].iKeyFrame.clear();    
        bool Anchor_linked_to_newKF = false;
        
        for(int j = KeyFDATA.size()-1; j >=0; j--)
            {   
                // for each i-anchor check its "visibility" with the j-keyframe
                std::vector<double> quat;
                quat = arma::conv_to<vec_t>::from(KeyFDATA[j].CameraAtt); // convert from armadillo vector to c++ vector
                double Rn2c_a[9];
                quat2R(&quat[0],Rn2c_a);
                arma::mat Rn2c(Rn2c_a,3,3); // convert from arrat to armadillo
                arma::vec::fixed<3> Tn2c = KeyFDATA[j].CameraPos;
                arma::vec::fixed<3> pxyz = AnchorsDATA[i].AnchorState;
                arma::vec::fixed<3> pc = Rn2c*(pxyz - Tn2c); // feature point expressed in the camera frame
                arma::mat::fixed<2,3> duv_dPc;
                cv::Point2d uvd = Projection_model(pc,0,false,PAR.img.cam_parameters,duv_dPc );
                // check if the feature is predicted to appear in the image
                int sm =  PAR.img.search_margin; 
                if((uvd.x > sm)&&(uvd.x < PAR.img.image_cols - sm)&&(uvd.y > sm)&&(uvd.y < PAR.img.image_rows - sm))
                {
                    // The anchor is predicted to appear in the (j) i-KeyFrame
                    KeyFramesData kfd;
                    kfd.KeyF_idx = j;
                    kfd.PredictedPoint = uvd;
                    kfd.matched = false;
                    AnchorsDATA[i].iKeyFrame.push_back(kfd);

                    if(Anchor_linked_to_newKF == true) 
                    {
                    // if the anchor is linked to the last (new) KeyFrame then update new row of the visibility graph                    
                        vg_row[j]++;
                    } 
                    
                    if(j == KeyFDATA.size()-1) // check if the anchor is linked to the last (new) KeyFrame
                    {
                       Anchor_linked_to_newKF = true;
                    }

                   
                }    //  if((uvd.x > sm)&&(uvd.x < PAR.img.image_cols - sm)&&(uvd.y > sm)&&(uvd.y < PAR.img.image_rows - sm)) 


            } // for(int j = KeyFDATA.size()-1; j >=0; j--)
     
    } //  for(int i = AnchorsDATA.size()-1; i >= 0; i--)
    
    // update visibility graph
     arma::vec vg_col = vg_row.t();      
        Vgraph.row(vg_size-1)  = vg_row;
        Vgraph.col(vg_size-1) = vg_col;

   

}


//---------------------------------------------------------------------------------------------
void GMAP::Full_Update_Visibility_Graph3()
{
    // for testing purposes ---------------------------------
    cv::Mat vg_old(Vgraph.n_rows, Vgraph.n_cols, CV_8UC1, cv::Scalar(255));
    for (int i =0 ; i < Vgraph.n_rows; i++)
    {
      for (int j =0 ; j < Vgraph.n_cols; j++)
        {  
           if (Vgraph(i,j) != 0) 
             {                   
                   vg_old.at<uchar>(i,j) = 0;
             }          
        }
    }
    //-------------------------------------------------
    auto begin = std::chrono::high_resolution_clock::now();
   
    Vgraph.clear();  // Reset visibility graph
    Vgraph.zeros();

    int vg_size = KeyFDATA.size();
    Vgraph.resize(vg_size,vg_size); // increase size of visibility graph by one
    //Vgraph(vg_size-1,vg_size-1) = 0;
    
    
    
    for(int i=0 ; i < AnchorsDATA.size();i++)
    {
      AnchorsDATA[i].iKeyFrame.clear();
    
      for(int j = 0; j < KeyFDATA.size(); j++)
        { 
              arma::vec::fixed<3> pxyz = AnchorsDATA[i].AnchorState;
             // for each i-anchor check its "visibility" with the j-keyframe
              std::vector<double> quat;
              quat = arma::conv_to<vec_t>::from(KeyFDATA[j].CameraAtt); // convert from armadillo vector to c++ vector
              double Rn2c_a[9];
              quat2R(&quat[0],Rn2c_a);
              arma::mat Rn2c(Rn2c_a,3,3); // convert from arrat to armadillo
              arma::vec::fixed<3> Tn2c = KeyFDATA[j].CameraPos;                    
              arma::vec::fixed<3> pc = Rn2c*(pxyz - Tn2c); // feature point expressed in the camera frame
              arma::mat::fixed<2,3> duv_dPc;
              cv::Point2d uvd = Projection_model(pc,0,false,PAR.img.cam_parameters,duv_dPc );
              int sm =  PAR.img.search_margin; 
              if((uvd.x > sm)&&(uvd.x < PAR.img.image_cols - sm)&&(uvd.y > sm)&&(uvd.y < PAR.img.image_rows - sm))
                {
                          KeyFramesData kfd;
                          kfd.KeyF_idx = j;
                          kfd.PredictedPoint = uvd;
                          kfd.matched = false;
                          AnchorsDATA[i].iKeyFrame.push_back(kfd);
                }      
        } // for(int j = 0; j < KeyFDATA.size(); j++)
        
        for (int k = 0; k< AnchorsDATA[i].iKeyFrame.size();k++)
        {
          int i_idx = AnchorsDATA[i].iKeyFrame[k].KeyF_idx;
          
          for(int l = k+1; l < AnchorsDATA[i].iKeyFrame.size();l++)
          {
              int j_idx = AnchorsDATA[i].iKeyFrame[l].KeyF_idx;
              
              Vgraph(i_idx,j_idx)++; 
          }


        }

    
    } // for(int i=0 ; i < AnchorsDATA.size();i++)
    
    Vgraph = arma::symmatu(Vgraph);
    
   auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    
    std::cout << " Visibility graph updated: ";    
    std::printf("Execution time: %.3f seconds.\n", elapsed.count() * 1e-9);

  // for testing purposes ---------------------------------
    cv::Mat vg_new(Vgraph.n_rows, Vgraph.n_cols, CV_8UC1, cv::Scalar(255));
    for (int i =0 ; i < Vgraph.n_rows; i++)
    {
      for (int j =0 ; j < Vgraph.n_cols; j++)
        {  
           if (Vgraph(i,j) != 0) 
             {                   
                   vg_new.at<uchar>(i,j) = 0;
             }          
        }
    }
    //------------------------------------------------- 
     imwrite("vg_old.png" ,vg_old);
      imwrite("vg_new.png" ,vg_new);


    int q = 10; 
    //cv::imshow("Old", vg_old);
    //cv::waitKey(0); // Wait for a keystroke in the window
    //cv::imshow("New", vg_new);
    //cv::waitKey(0); // Wait for a keystroke in the window




}





//----------------------------------------------------------------------------------------------------------------
#define RATIO    0.75
//#define RATIO    0.85
void matchFeatures2b(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches) 
{
    std::vector<std::vector<cv::DMatch>> matches;
    
    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
    
    matcher.knnMatch(query, target, matches, 2);
    // Second neighbor ratio test.
    for (unsigned int i = 0; i < matches.size(); ++i) 
    {   
        if (!matches[i].empty())
            if (matches[i][0].distance < matches[i][1].distance * RATIO)
                goodMatches.push_back(matches[i][0]);
    }
}