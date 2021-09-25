
#include "map.h"
#include "../Transforms/quat2R.h"
#include "../vision/vision.h"
#include <math.h>
#include <opencv2/sfm/triangulation.hpp>



void Init_new_anchors(vectorKeyF &KeyFDATA, vectorFeat &NewAnchorsDATA,parameters &PAR );
void matchFeatures2(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches );

void Update_Visibility_Graph(vectorKeyF &KeyFDATA, vectorFeat &NewAnchorsDATA, vectorFeat &AnchorsDATA,arma::mat &Vgraph,parameters &PAR );

// convert an OpenCV matrix to Armadillo matrix. NOTE: a copy is made
template <typename T>
arma::Mat<T> to_arma(const cv::Mat_<T> &src)
{
  arma::Mat<T> dst(src.cols, src.rows);
  src.copyTo({src.rows, src.cols, dst.memptr()});
  return dst;
}

// convert an Armadillo matrix to OpenCV matrix. NOTE: no copy is made
template <typename T>
cv::Mat_<T> to_cvmat(const arma::Mat<T> &src)
{
  return cv::Mat_<double>{int(src.n_cols), int(src.n_rows), const_cast<T*>(src.memptr())};
}

//---------------------------------------------------------------------------------------------------
// Rodrigo M. 2020
//-----------------------------------------------------------------------------------------------------
bool GMAP::Add_KeyFrames()
{
  bool new_keyFrame = false;

  for(int k = 0; k < NewKeyFDATA.size();k++  )
  {

    // Add new Keyframe to KeyFrame structure 
    KeyFDATA.push_back(NewKeyFDATA[k]);
    NewKeyFDATA.erase(NewKeyFDATA.begin() + k);
    k--;
    
    

    
    // Stereo initialization of new anchors from the last two KeyFrames
    Init_new_anchors(KeyFDATA,NewAnchorsDATA,PAR);

    // Add new anchors obtained from the EKF-SLAM
    for(unsigned int j= 0; j < Push_NewAnchorsDATA.size(); j++ )
      {
        NewAnchorsDATA.push_back(Push_NewAnchorsDATA[j]);  // copy new anchor to  main anchors (map) structure
        
        
          Push_NewAnchorsDATA.erase(Push_NewAnchorsDATA.begin() + j);
        
        j--;             
      }   

    // Update visibility Graph with new keyFrame and new Anchors obtained from the EKF-SLAM 
    Update_Visibility_Graph(KeyFDATA,NewAnchorsDATA,AnchorsDATA,Vgraph,PAR );
     
    // Vgraph.print();

     // Add new anchors obtained from the EKF-SLAM
    for(unsigned int j= 0; j < NewAnchorsDATA.size(); j++ )
      {
        AnchorsDATA.push_back(NewAnchorsDATA[j]);  // copy new anchor to  main anchors (map) structure
        NewAnchorsDATA.erase(NewAnchorsDATA.begin() + j);
        j--;             
      }   

    new_keyFrame = true;  

    break;

  }

  return new_keyFrame;

}


//---------------------------------------------------------------------------------------------------
// Rodrigo M. 2020
//-----------------------------------------------------------------------------------------------------
void Init_new_anchors(vectorKeyF &KeyFDATA, vectorFeat &NewAnchorsDATA,parameters &PAR )
{
  if(KeyFDATA.size()>1)
  {
        /*
        //Sorting keypoints of last KeyFrame by deacreasing order of strength
        vector<float> responseVector;
        for (unsigned int i =0 ; i<KeyFDATA[KeyFDATA.size()-1].keyPoints.size(); i++) responseVector.push_back(KeyFDATA[KeyFDATA.size()-1].keyPoints[i].response);
        vector<int> Indx(responseVector.size()); 
        std::iota (std::begin(Indx), std::end(Indx), 0);
        cv::sortIdx(responseVector, Indx, cv::SORT_DESCENDING);
        vector<cv::KeyPoint> keyPointsSorted;
        for (unsigned int i = 0; i < KeyFDATA[KeyFDATA.size()-1].keyPoints.size(); i++) keyPointsSorted.push_back(KeyFDATA[KeyFDATA.size()-1].keyPoints[Indx[i]]);

        int numRetPoints = 200;
        float tolerance = 0.1; // tolerance of the number of return points

        vector<cv::KeyPoint> sscKP = Ssc(keyPointsSorted,numRetPoints,tolerance,KeyFDATA[KeyFDATA.size()-1].frame.cols,KeyFDATA[KeyFDATA.size()-1].frame.rows);
         
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(1000, 1.2f, 8, 16,0,2, cv::ORB::FAST_SCORE,31, 5);
        cv::Mat descriptors_sscKP; // descriptors of last KeyFrame
        detector->compute(KeyFDATA[KeyFDATA.size()-1].frame, sscKP,descriptors_sscKP); // compute descriptors for Keypoints
        
        */
        // obtain matches
        std::vector<cv::DMatch> matches;
       // matchFeatures2(KeyFDATA[KeyFDATA.size()-2].Descriptors, descriptors_sscKP, matches);
        
        matchFeatures2(KeyFDATA[KeyFDATA.size()-2].Descriptors, KeyFDATA[KeyFDATA.size()-1].Descriptors, matches);
        
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
          
          // check minimun distance
          double m_dis = 20; // minumun distance
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
       //Tn2c1(1) = -Tn2c1(1);
       // Tn2c1(2) = -Tn2c1(2);
        
        arma::mat::fixed<3,4> RT1;
        RT1(arma::span(0,2),arma::span(0,2)) = Rn2c1;
        RT1.col(3) = Tn2c1;

        std::vector<double> quat2;
        quat2 =  arma::conv_to<vec_t>::from(KeyFDATA[KeyFDATA.size()-2].CameraAtt); // convert from armadillo vector to c++ vector
        double Rn2c_a2[9];
        quat2R(&quat2[0],Rn2c_a2);
        arma::mat Rn2c2(Rn2c_a2,3,3); // convert from arrat to armadillo
        arma::vec::fixed<3> Tn2c2 = KeyFDATA[KeyFDATA.size()-2].CameraPos;
       // Tn2c2(1) = -Tn2c2(1);
       // Tn2c2(2) = -Tn2c2(2);
        
        arma::mat::fixed<3,4> RT2;
        RT2(arma::span(0,2),arma::span(0,2)) = Rn2c2;
        RT2.col(3) = Tn2c2;
        
        //arma::mat::fixed<3,4> Pm1at = KK*RT1;        
        //arma::mat::fixed<3,4> Pm2a = KK*RT2;
        arma::mat Pm1a = (KK*RT1).t();     // transpose to convert to OpenCV mat   
        arma::mat Pm2a = (KK*RT2).t();

        cv::Mat Pm1 = to_cvmat(Pm1a);  // projection matrix 1
        cv::Mat Pm2 = to_cvmat(Pm2a);  // projection matrix 1
        //-----------------------------------------------------
        //Pm1at.print();
        //cout << "Pm1 = " << endl << " "  << Pm1 << endl << endl;

        cv::Mat pnts3D(4,P_m1.size(),CV_32F);

        cv::triangulatePoints(Pm1,Pm2,P_m1,P_m2,pnts3D);

        //------------------------------------------------------------------
        /*
        arma::mat::fixed<3,4> Pma = (KK*RT1);
        arma::mat::fixed<3,4> Pmb = (KK*RT2);
        const double ua = P_m1[1].x;
        const double va = P_m1[1].y;
        const double ub = P_m2[1].x;
        const double vb = P_m2[1].y;

        arma::mat::fixed<4,4> A;
        A.row(0) =  ua*Pma.row(2)-Pma.row(0);
        A.row(1) =  va*Pma.row(2)-Pma.row(1);
        A.row(2) =  ub*Pmb.row(2)-Pmb.row(0);
        A.row(3) =  vb*Pmb.row(2)-Pmb.row(1);
       // A.print();
        arma::mat U;
        arma::vec s;
        arma::mat V;
        arma::svd(U,s,V,A);
        
        //arma::vec::fixed<4> b = {0,0,0,0};
        //arma::vec x1 = arma::solve(A, b);

        V.print();
        */

        //x1.print();

        
        /*
        vector<std::vector<cv::Point2f>> sfmPoints2d;
        sfmPoints2d.push_back(P_m1);
        sfmPoints2d.push_back(P_m2); 
        
        vector<cv::Mat> sfmProjMats;
        sfmProjMats.push_back(Pm1);
        sfmProjMats.push_back(Pm2);
        cv::Mat points3d;
        
        cv::sfm::triangulatePoints(sfmPoints2d, sfmProjMats, points3d);
        */
        //-------------------------------------------------------------------------



        //cout << "points = " << endl << " "  << pnts3D << endl << endl;
        //cout << pnts3D.at<float>(0,0) << endl;
       
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
            
           NewAnchorsDATA.push_back(Anchor); 
         //    Anchor.AnchorState.print();
        //    int q = 10;

        }


        int q = 10;

        
        

       /*
        // Draw matches.
        cv::Mat img_matches;
        cv::drawMatches( KeyFDATA[KeyFDATA.size()-2].frame, KeyFDATA[KeyFDATA.size()-2].keyPoints, KeyFDATA[KeyFDATA.size()-1].frame, KeyFDATA[KeyFDATA.size()-1].keyPoints, f_matches, img_matches, cv::Scalar::all(-1),
        cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
             
       //-- Show detected matches
        cv::imshow("Good Matches", img_matches );
        cv::waitKey();
      */

  }

}
//----------------------------------------------------------------------------------------------------------------
#define RATIO    0.75
//#define RATIO    0.85
void matchFeatures2(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches) 
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

//---------------------------------------------------------------------------------------------------
// Rodrigo M. 2020
//-----------------------------------------------------------------------------------------------------
void Update_Visibility_Graph(vectorKeyF &KeyFDATA, vectorFeat &NewAnchorsDATA, vectorFeat &AnchorsDATA,arma::mat &Vgraph,parameters &PAR )
{


      
      int vg_size = Vgraph.n_rows; // size of visibility graph
      Vgraph.resize(vg_size+1,vg_size+1); // increase size of visibility graph by one
      Vgraph(vg_size,vg_size) = 0;

      arma::rowvec vg_row;
      vg_row.zeros(vg_size);

      //--- Update visual relations with NewAnchordsDATA
       int n_zeros = 0;
       bool link_iKeyF;
       int first_kf_linked = 0;
      for(int i = KeyFDATA.size()-1; i >= 0 ; i--)
      {
          link_iKeyF = false;
          for(unsigned int j= 0; j < NewAnchorsDATA.size(); j++ )
          {             
            // for each new anchor check its "visibility" with the i-keyframe
            std::vector<double> quat;
            quat = arma::conv_to<vec_t>::from(KeyFDATA[i].CameraAtt); // convert from armadillo vector to c++ vector
            double Rn2c_a[9];
            quat2R(&quat[0],Rn2c_a);
            arma::mat Rn2c(Rn2c_a,3,3); // convert from arrat to armadillo
            arma::vec::fixed<3> Tn2c = KeyFDATA[i].CameraPos;
            arma::vec::fixed<3> pxyz = NewAnchorsDATA[j].AnchorState;
            arma::vec::fixed<3> pc = Rn2c*(pxyz - Tn2c); // feature point expressed in the camera frame
            arma::mat::fixed<2,3> duv_dPc;
            cv::Point2d uvd = Projection_model(pc,0,false,PAR.img.cam_parameters,duv_dPc );
            // check if the feature is predicted to appear in the image
            int sm =  PAR.img.search_margin; 
            if((uvd.x > sm)&&(uvd.x < PAR.img.image_cols - sm)&&(uvd.y > sm)&&(uvd.y < PAR.img.image_rows - sm))
            {
              // The anchor is predicted to appear in the i-KeyFrame
              KeyFramesData kfd;
              kfd.KeyF_idx = i;
              kfd.PredictedPoint = uvd;
              NewAnchorsDATA[j].iKeyFrame.push_back(kfd);

              // and index of anchor to the vector of index of features (anchors) predicted to appear in the i-keyframe
              int newAnchor_idx = AnchorsDATA.size() + j;
              KeyFDATA[i].PredictedFeats_idx.push_back(newAnchor_idx);

              if((KeyFDATA.size()>1)&&(i<KeyFDATA.size()-1))
              {
                vg_row[i]++;
              }

              link_iKeyF = true;  
            }  
           
          } // for(unsigned int j= 0; j < NewAnchorsDATA.size(); j++ )
          if (link_iKeyF == false)
          {
            n_zeros++;
          }
          if (n_zeros > PAR.map.n_consec_kf_wo_link) // If there is not visual link for n consecutive Keyframes, break the loop
          { 
            first_kf_linked = i;
           // break;
          }

      }//for(int i = 0; i < KeyFDATA.size(); i++)
      
      //--- Update visual relations with AnchordsDATA
      
      //for(unsigned int j= 0; j < AnchorsDATA.size(); j++ )
      int n_not_linked = 0;
        for( int j = AnchorsDATA.size()-1; j >= 0 ; j--)
        { 
           
           if(AnchorsDATA[j].iKeyFrame.size()>0)
           {
             int iKF = AnchorsDATA[j].iKeyFrame[0].KeyF_idx; // For each anchor get its fisrt KeyFrame associate
             if(iKF >= first_kf_linked) 
             { 
               int idx_last_KF = KeyFDATA.size()-1;
               std::vector<double> quat;
               quat = arma::conv_to<vec_t>::from(KeyFDATA[idx_last_KF].CameraAtt); // convert from armadillo vector to c++ vector
               double Rn2c_a[9];
               quat2R(&quat[0],Rn2c_a);
               arma::mat Rn2c(Rn2c_a,3,3); // convert from arrat to armadillo
               arma::vec::fixed<3> Tn2c = KeyFDATA[idx_last_KF].CameraPos;
               arma::vec::fixed<3> pxyz = AnchorsDATA[j].AnchorState;
               arma::vec::fixed<3> pc = Rn2c*(pxyz - Tn2c); // feature point expressed in the camera frame
               arma::mat::fixed<2,3> duv_dPc;
               cv::Point2d uvd = Projection_model(pc,0,false,PAR.img.cam_parameters,duv_dPc );
               // check if the feature is predicted to appear in the image
              int sm =  PAR.img.search_margin; 
               if((uvd.x > sm)&&(uvd.x < PAR.img.image_cols - sm)&&(uvd.y > sm)&&(uvd.y < PAR.img.image_rows - sm))
                {
                  // The anchor is predicted to appear in the i-KeyFrame
                  KeyFramesData kfd;
                  kfd.KeyF_idx = idx_last_KF;
                  kfd.PredictedPoint = uvd;
                  AnchorsDATA[j].iKeyFrame.push_back(kfd);

                   // and index of anchor to the vector of index of features (anchors) predicted to appear in the last i-keyframe
                  
                  KeyFDATA[idx_last_KF].PredictedFeats_idx.push_back(j);
                  
                  vg_row[iKF]++; // visual graph row update

                  n_not_linked = 0;
                } 
               
             }
             else
             {
               n_not_linked++;
             }
             
           }
           // if n Anchors have not been visualy linked with the last Key frame, then break the loop..
           if(n_not_linked > PAR.map.c_consec_anchor_wo_link) 
           {
             //break;
           }
           

          
        }    

      // Update visibility graph
      vg_size = Vgraph.n_rows; // size of visibility graph
      vg_row.resize(vg_size);
      vg_row(vg_size-1) = 0;
      arma::vec vg_col = vg_row.t();
      if(vg_size > 1)
      {
        Vgraph.row(vg_size-1)  = vg_row;
        Vgraph.col(vg_size-1) = vg_col;

      }





}