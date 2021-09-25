#include "visual_match_feats.h"
#include "../Transforms/quat2R.h"
#include "../Jacs/Jac_uv_XYZ.h"



MatchInfo visual_match_feats(arma::vec& x, arma::mat& P, parameters &par,FRAME *frame, vectorFeat& FeatsDATA ,vectorFeat& AnchorsDATA,GMAP &gmap,LOOP &cloop,LOCKS &locks)
{

    MatchInfo Mi;

        
        /*
        cv::Mat fastDetectionResults; //draw FAST detections
        cv::drawKeypoints(frame->image,keyPoints,fastDetectionResults, cv::Scalar(94.0, 206.0, 165.0, 0.0));
        cv::namedWindow("FAST Detections", cv::WINDOW_AUTOSIZE); 
        cv::imshow( "FAST Detections", fastDetectionResults);
        waitKey(0);
        */
        
        cv::Mat Descriptors;
        //std::vector<KeyPoint> keypoints;
        std::vector<int> FeatIndex;
        std::vector<arma::mat::fixed<2,2>> FeatInnov;
        int n_p_feats;   // number of predicted features 
        int n_p_anchors = 0; // bumber of predicted anchors
        int n_m_feats = 0; // number of matched features
        int n_m_anchors = 0; // number of matched anchors


        // --- check if Features are predicted to appear in the current frame   
        for (int i= 0; i< FeatsDATA.size();i++)
        {
            int idx_i = FeatsDATA[i].idx_i_state;
            int idx_f = FeatsDATA[i].idx_f_state;

            std::vector<double> quat;
            quat = arma::conv_to<vec_t>::from(x.subvec(0,3)); // convert from armadillo vector to c++ vector
            double Rn2c_a[9];
            quat2R(&quat[0],Rn2c_a);
            arma::mat Rn2c(Rn2c_a,3,3); // convert from arrat to armadillo
            arma::vec::fixed<3> Tn2c = x.subvec(7,9);
            arma::vec::fixed<3> pxyz = x.subvec(idx_i,idx_f);
            arma::vec::fixed<3> pc = Rn2c*(pxyz - Tn2c); // feature point expressed in the camera frame
            arma::mat::fixed<2,3> duv_dPc;
            cv::Point2d uvd = Projection_model(pc,1,false,par.img.cam_parameters,duv_dPc );
            int sm =  par.img.search_margin;

            // check if the feature is predicted to appear in the image 
            if((uvd.x > sm)&&(uvd.x < par.img.image_cols - sm)&&(uvd.y > sm)&&(uvd.y < par.img.image_rows - sm))
            {
                FeatsDATA[i].PredictedPoint = uvd;   
                
                if (par.img.check_innovation_for_mathing == true) // compute potential match innovation
                {   
                    arma::mat::fixed<2,13> duv_dx;
                    arma::mat::fixed<2,3> duv_dy;
                    Jac_uv_XYZ(pxyz,x,par,duv_dx,duv_dy);
                    int x_len = x.size();

                    arma::mat Hi;
                    Hi.zeros(2,x_len);
                    Hi(arma::span(0,1),arma::span(0,12)) = duv_dx;
                    Hi(arma::span(0,1),arma::span(idx_i,idx_f)) = duv_dy;

                    arma::mat::fixed<2,2> Ri;
                    Ri(0,0) = pow(par.ekf.sigma_uv,2);
                    Ri(1,1) = pow(par.ekf.sigma_uv,2);

                    arma::mat::fixed<2,2> Si;
                    Si = Hi*P*Hi.t() + Ri;

                    FeatsDATA[i].Si = Si;
                    FeatsDATA[i].duv_dx = duv_dx;
                    FeatsDATA[i].duv_dy = duv_dy;
                    FeatInnov.push_back(Si);    
                }                

                Descriptors.push_back(FeatsDATA[i].Descriptor);
                //keypoints.push_back(FeatsDATA[i].Keypoint); 
                FeatIndex.push_back(i);

                FeatsDATA[i].predicted = true; 
                FeatsDATA[i].matched = false;   // set by default 
                FeatsDATA[i].times_not_mathed++; // increment by defualt, if later teh feat is matched then this value is decremented              
                
            }
            else // the feature is predicted out of the image
            {
                FeatsDATA[i].predicted = false;
                FeatsDATA[i].times_not_considered++; // increment
                FeatsDATA[i].matched = false;
            }
        

        } // for (int i= 0; i< FeatsDATA.size();i++)
        n_p_feats = FeatIndex.size();

        // check if anchors are predicted to appear in the current frame
        std::vector<int> AnchorIndex;
        for(int i = 0; i< AnchorsDATA.size(); i++)
        {

            std::vector<double> quat;
            quat = arma::conv_to<vec_t>::from(x.subvec(0,3)); // convert from armadillo vector to c++ vector
            double Rn2c_a[9];
            quat2R(&quat[0],Rn2c_a);
            arma::mat Rn2c(Rn2c_a,3,3); // convert from arrat to armadillo
            arma::vec::fixed<3> Tn2c = x.subvec(7,9);
            arma::vec::fixed<3> pxyz = AnchorsDATA[i].AnchorState;            

            arma::vec::fixed<3> pc = Rn2c*(pxyz - Tn2c); // feature point expressed in the camera frame
            arma::mat::fixed<2,3> duv_dPc;
            cv::Point2d uvd = Projection_model(pc,1,false,par.img.cam_parameters,duv_dPc );
            int sm =  par.img.search_margin;

            // check if the Anchor is predicted to appear in the image 
            if((uvd.x > sm)&&(uvd.x < par.img.image_cols - sm)&&(uvd.y > sm)&&(uvd.y < par.img.image_rows - sm))
            {
                AnchorsDATA[i].PredictedPoint = uvd;  

                Descriptors.push_back(AnchorsDATA[i].Descriptor);
                AnchorIndex.push_back(i);

                AnchorsDATA[i].predicted = true; 
                AnchorsDATA[i].matched = false;   // set by default 
                AnchorsDATA[i].times_not_mathed++; // increment by defualt, if later teh feat is matched then this value is decremented              

                n_p_anchors++;
            }    
            else // the feature is predicted out of the image
            {
                AnchorsDATA[i].predicted = false;
                AnchorsDATA[i].times_not_considered++; // increment
                AnchorsDATA[i].matched = false;
            }



        }


        // obtain candidate Keypoint to be matched
        vector<cv::KeyPoint> Candidate_keyPoints;
        cv::Mat CandidateDescriptors;
        Ptr<FeatureDetector> detector = ORB::create(1000, 1.2f, 8, 16,0,2, ORB::FAST_SCORE,31, 5);
        detector->detectAndCompute(frame->image,cv::Mat(), Candidate_keyPoints,CandidateDescriptors);
        
        // obtain matches
        std::vector<cv::DMatch> matches;
        matchFeatures(Descriptors, CandidateDescriptors, matches);


        // add key frames 
        if (cloop.closing_loop_active == false)
        {   
            // if conditions are acomplished
            add_key_frame_to_global_map(x,par,gmap,locks,Candidate_keyPoints,CandidateDescriptors,matches,AnchorsDATA,frame->image,FeatsDATA);
            
            // at each frame
            add_key_frame_to_close_loop(x,par,cloop,locks,Candidate_keyPoints,CandidateDescriptors,matches,AnchorsDATA,frame->image,FeatsDATA);
        }
        

        // Innovation check and Update features table 
        for(int i =0 ; i< matches.size();i++ ) // for each match, update Feature table
        {         
                 
            int Descriptor_idx = matches[i].queryIdx; // get index of the descriptor matched

            if (Descriptor_idx < n_p_feats)  // A feature has been matched
            {

                    int Feat_idx = FeatIndex[Descriptor_idx]; // get index of the feature matched
                    int CandidateDescriptor_idx = matches[i].trainIdx; // get index of the candidate keypoint matched
                    cv::Point2f measured_point = Candidate_keyPoints[CandidateDescriptor_idx].pt;
                    cv::Point2f predicted_point = FeatsDATA[Feat_idx].PredictedPoint;

                    if (par.img.check_innovation_for_mathing == true) // compute potential match innovation
                    {
                        
                        // distance from the 
                        double Sigma_s = 2; // number of sigmas to consider
                        double d = sqrt(pow(measured_point.x - predicted_point.x,2) + pow(measured_point.y - predicted_point.y,2)); 
                        double innov =  sqrt(Sigma_s*Sigma_s*FeatsDATA[Feat_idx].Si(0,0) + Sigma_s*Sigma_s*FeatsDATA[Feat_idx].Si(1,1));

                        if( d < innov)
                        { 
                            FeatsDATA[Feat_idx].matched = true;
                            FeatsDATA[Feat_idx].times_not_mathed--;
                            FeatsDATA[Feat_idx].times_mathed++;             
                            FeatsDATA[Feat_idx].Keypoint = Candidate_keyPoints[CandidateDescriptor_idx]; // Update Feature Keypoint
                            FeatsDATA[Feat_idx].MatchedPoint = Candidate_keyPoints[CandidateDescriptor_idx].pt; // Update position of the feature
                            n_m_feats++;
                        }

                    }    
                    else
                    {   
                    // Simple extra check for matching
                        
                        double d = sqrt(pow(measured_point.x - predicted_point.x,2) + pow(measured_point.y - predicted_point.y,2)); 

                        if( d < par.img.max_innov_pixels )
                        {    
                            FeatsDATA[Feat_idx].matched = true;
                            FeatsDATA[Feat_idx].times_not_mathed--;
                            FeatsDATA[Feat_idx].times_mathed++;             
                            FeatsDATA[Feat_idx].Keypoint = Candidate_keyPoints[CandidateDescriptor_idx]; // Update Feature Keypoint
                            FeatsDATA[Feat_idx].MatchedPoint = Candidate_keyPoints[CandidateDescriptor_idx].pt; // Update position of the feature
                            n_m_feats++;
                        }
                    }

            }
            else // An anchor has been matched
            {   
                int Anchor_idx = AnchorIndex[Descriptor_idx-n_p_feats]; // get index of the feature matched
                int CandidateDescriptor_idx = matches[i].trainIdx; // get index of the candidate keypoint matched
                cv::Point2f measured_point = Candidate_keyPoints[CandidateDescriptor_idx].pt;
                cv::Point2f predicted_point = AnchorsDATA[Anchor_idx].PredictedPoint;

                double d = sqrt(pow(measured_point.x - predicted_point.x,2) + pow(measured_point.y - predicted_point.y,2)); 

                    if( d < par.img.max_innov_pixels ) // Simple extra check for matching
                    { 
                        AnchorsDATA[Anchor_idx].matched = true;
                        AnchorsDATA[Anchor_idx].times_not_mathed--;
                        AnchorsDATA[Anchor_idx].times_mathed++;             
                        AnchorsDATA[Anchor_idx].Keypoint = Candidate_keyPoints[CandidateDescriptor_idx]; // Update Feature Keypoint
                        AnchorsDATA[Anchor_idx].MatchedPoint = Candidate_keyPoints[CandidateDescriptor_idx].pt; // Update position of the feature    
                        n_m_anchors++;
                    }
              
            }
            
            
           
        }
        //----------------------------------------------------------------------

       // testing matches
       /*      
       for(int i =0 ; i< FeatsDATA.size();i++ )
       {
           if(FeatsDATA[i].matched == true)
           {
               cv::circle(frame->image,FeatsDATA[i].Keypoint.pt, 5, (0,0,255), -1);

           }

       }
        // Draw matches.
        cv::Mat img_matches;
        cv::drawMatches( frame->image, Featkeypoints, frame->image, Candidate_keyPoints, matches, img_matches, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
       //-- Show detected matches
        cv::imshow("Good Matches", img_matches );
        cv::waitKey();

        int q = 10;  
        */
        Mi.n_predicted_img_feats = n_p_feats;
        Mi.n_predicted_img_anchors = n_p_anchors;
        Mi.n_matched_feats = n_m_feats;
        Mi.n_matched_anchors = n_m_anchors;
   
    return Mi;
}

//---------------------------------------------------------------------------------------------------------------
void add_key_frame_to_global_map(arma::vec& x,parameters &par,GMAP &gmap,LOCKS &locks,vector<cv::KeyPoint> &keyPoints,cv::Mat &Descriptors,std::vector<cv::DMatch> &Matches,vectorFeat& AnchorsDATA,cv::Mat &frame,vectorFeat& FeatsDATA)
{
        static int count_f=0;
        static arma::vec::fixed<3> last_cam_pos = {0,0,0};
        count_f++;
        
        double s_d = 0;
        for(int i=0;i<FeatsDATA.size();i++)
        {
            double d =  arma::norm(x.subvec(FeatsDATA[i].idx_i_state,FeatsDATA[i].idx_f_state) - FeatsDATA[i].CameraState  ) ;
           s_d = s_d + d; 
        }

        double p_d = s_d/FeatsDATA.size(); // average feature depth
        double dist = arma::norm(last_cam_pos - x.subvec(7,9) ) ; // distance from last KeyFrame

        double ratio_dist_pd = dist/p_d;
        //if ((count_f > 20)&&(AnchorsDATA.size() > 4)&&(Matches.size() > 10))
        
        //cout << Matches.size() << endl;
        if ((count_f > 5)&&(Matches.size() > 5)&&(ratio_dist_pd > .15))
        {   
               // cout << dist << endl;

              // add anchors to the pile of new anchors of the global map

            /*
            for(int i = 0; i < AnchorsDATA.size(); i++)
            {
                if (AnchorsDATA[i].ekf_to_gmap_status == 0)
                {
                    locks.NewEKFAnchorsDATA_mtx.lock();
                            gmap.Push_NewAnchorsDATA.push_back(AnchorsDATA[i]);
                        locks.NewEKFAnchorsDATA_mtx.unlock(); 

                        AnchorsDATA[i].ekf_to_gmap_status = 1;
                }     

            } 
            */     

            // Add KeyPoint 
            KEYFRAME keyframe;
            keyframe.frame = frame;
            keyframe.CameraAtt = x.subvec(0,3);
            keyframe.CameraPos = x.subvec(7,9);
            keyframe.Descriptors = Descriptors;
            keyframe.keyPoints = keyPoints;        
            //map.add_keyframe(keyframe);
            locks.add_keyframe_mtx.lock();  
                gmap.NewKeyFDATA.push_back(keyframe);                
            locks.add_keyframe_mtx.unlock();

            count_f = 0;
            last_cam_pos = x.subvec(7,9);
        
           
            //std::this_thread::sleep_for (std::chrono::milliseconds(50)); 
        
        }     
        
   
         

}
//---------------------------------------------------------------------------------------------------------------

void add_key_frame_to_close_loop(arma::vec& x,parameters &par,LOOP &cloop,LOCKS &locks,vector<cv::KeyPoint> &keyPoints,cv::Mat &Descriptors,std::vector<cv::DMatch> &Matches,vectorFeat& AnchorsDATA,cv::Mat &frame,vectorFeat& FeatsDATA)
{

            KEYFRAME keyframe;
            keyframe.frame = frame;
            keyframe.CameraAtt = x.subvec(0,3);
            keyframe.CameraPos = x.subvec(7,9);
            keyframe.Descriptors = Descriptors;
            keyframe.keyPoints = keyPoints;        
            //map.add_keyframe(keyframe);
            locks.add_keyframe_mtx_CL.lock();                             
                cloop.Current_KF = keyframe;
                cloop.newFrame = true;
            locks.add_keyframe_mtx_CL.unlock();

}




//----------------------------------------------------------------------------------------------------------------
#define RATIO    0.75
//#define RATIO    0.85
void matchFeatures(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches) 
{
    std::vector<std::vector<cv::DMatch>> matches;
    
    FlannBasedMatcher matcher = FlannBasedMatcher(makePtr<flann::LshIndexParams>(12, 20, 2));
    
    matcher.knnMatch(query, target, matches, 2);
    // Second neighbor ratio test.
    for (unsigned int i = 0; i < matches.size(); ++i) 
    {   
        if (!matches[i].empty())
            if (matches[i][0].distance < matches[i][1].distance * RATIO)
                goodMatches.push_back(matches[i][0]);
    }
}