#include "map.h"
#include "../Transforms/quat2R.h"
#include "../vision/vision.h"
#include <math.h>

void matchFeatures3(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches); 

int check_visibiliy_graph(arma::mat &Vgraph);
//---------------------------------------------------------------------------------------------------
// Rodrigo M. 2020
//-----------------------------------------------------------------------------------------------------
void GMAP::visual_match2()
{

    int idx_indirect;
    int idx_direct; 
    check_visibiliy_graph(idx_direct,idx_indirect);
    
    int n_extraKF_to_check = 0;

    int n_KF_to_check = KeyFDATA.size()-idx_indirect + n_extraKF_to_check; // number of keyFrames to check
    if (n_KF_to_check > KeyFDATA.size())
      n_KF_to_check = KeyFDATA.size();

    std::vector<cv::Mat> descriptors; 
    std::vector<std::vector<int>> idx_anchors;   
    descriptors.resize(n_KF_to_check);   // reserve space for the descriptors (matrices)
    idx_anchors.resize(n_KF_to_check);

    for(int i = AnchorsDATA.size()-1; i >=0 ; i--)
    {

      for(int j = 0; j < AnchorsDATA[i].iKeyFrame.size();j++)
      {
        int idx_kf = AnchorsDATA[i].iKeyFrame[j].KeyF_idx;
        
        int minKFtomatch = idx_indirect - n_extraKF_to_check;
        if(minKFtomatch < 0)
          minKFtomatch = 0;

        if (idx_kf >= minKFtomatch  ) // If idx_kf-KeyFrame must be checked for matches
        {
          int relative_idx_kf = idx_kf -( KeyFDATA.size()-n_KF_to_check)  ; // get the relative index

          descriptors[relative_idx_kf].push_back(AnchorsDATA[i].Descriptor);
          idx_anchors[relative_idx_kf].push_back(i); // store index of anchor
        }     
            
      } // for(int j = 0; j < AnchorsDATA[i].iKeyFrame.size();j++)

    } // for(int i = AnchorsDATA.size()-1; i >=0 ; i--)
    
     
    for(int i = idx_indirect,j = 0; i <  KeyFDATA.size(); i++,j++) // for each KeyFrame to check, ( from old to new)
    {

        std::vector<cv::DMatch> matches;
        matchFeatures3(descriptors[j],KeyFDATA[i].Descriptors, matches);
       //match (queryDescriptors, trainDescriptors, matches)
        for(unsigned int k =0 ; k< matches.size();k++ ) // for each match, update Feature table          
         { 
            int Descriptor_idx_q = matches[k].queryIdx; // get index of the descriptor in descriptors[j]      
            int Descriptor_idx_t = matches[k].trainIdx; // get index of the descriptor in KeyFDATA[i].Descriptors  
            
            int idx_anchor = idx_anchors[j][Descriptor_idx_q]; // get index of anchor
            
            cv::Point2f point_pred;
            int feat_iKey_idx;            
            for(int l = 0; l < AnchorsDATA[idx_anchor].iKeyFrame.size();l++)
            {
              if(AnchorsDATA[idx_anchor].iKeyFrame[l].KeyF_idx==i)
              {
                point_pred = AnchorsDATA[idx_anchor].iKeyFrame[l].PredictedPoint;
                feat_iKey_idx = l;                
                break;
              }
            }                        
            cv::Point2f point_match = KeyFDATA[i].keyPoints[Descriptor_idx_t].pt;
            double dis = sqrt( pow(point_pred.x - point_match.x,2) + pow(point_pred.y - point_match.y,2)  );

            // check innovation error  (norm(predicted-matched))
            double max_inov_error = 20;
            if(dis < max_inov_error)
            {
              
              AnchorsDATA[idx_anchor].iKeyFrame[feat_iKey_idx].MatchedPoint = point_match ;
              AnchorsDATA[idx_anchor].iKeyFrame[feat_iKey_idx].matched = true;
            }
          }  

       


    }

 



}








// 
void GMAP::visual_match()
{

    int idx_kf_match;
    int idx_direct; 
    check_visibiliy_graph(idx_direct,idx_kf_match);
      
    int n_zero_vlink = -1;
    for(int i = KeyFDATA.size()-1; i >= idx_kf_match ; i--)
    {   

        KeyFDATA[i].MatchedFeats_idx.clear();
        cv::Mat iKFdescriptors;  
        for(int j = 0; j < KeyFDATA[i].PredictedFeats_idx.size();j++ )
        {
          int idx_Anchor = KeyFDATA[i].PredictedFeats_idx[j];   // get index of the features predicted to appear in the Keyframe

          iKFdescriptors.push_back(AnchorsDATA[idx_Anchor].Descriptor);  
          
        }
        
       
          std::vector<cv::DMatch> matches;
          matchFeatures3(iKFdescriptors,KeyFDATA[i].Descriptors, matches);

          
          for(unsigned int j =0 ; j< matches.size();j++ ) // for each match, update Feature table
          { 
            int Descriptor_idx_q = matches[j].queryIdx; // get index of the descriptor in KeyFDATA[i].Descriptors        
            int Descriptor_idx_t = matches[j].trainIdx; // get index of the descriptor in iKFdescriptors
            int idx_feat = KeyFDATA[i].PredictedFeats_idx[Descriptor_idx_q];
            
            cv::Point2f point_pred;
            int feat_iKey_idx;
            for(int k = 0; k < AnchorsDATA[idx_feat].iKeyFrame.size();k++)
            {
              if(AnchorsDATA[idx_feat].iKeyFrame[k].KeyF_idx==i)
              {
                point_pred = AnchorsDATA[idx_feat].iKeyFrame[k].PredictedPoint;
                feat_iKey_idx = k;
                break;
              }
            }            
            cv::Point2f point_match = KeyFDATA[i].keyPoints[Descriptor_idx_t].pt;
            
            double dis = sqrt( pow(point_pred.x - point_match.x,2) + pow(point_pred.y - point_match.y,2)  );

            // check innovation error  (norm(predicted-matched))
            double max_inov_error = 25;
            if(dis < max_inov_error)
            {
              KeyFDATA[i].MatchedFeats_idx.push_back(idx_feat); // add index of matched feature to keyframe
              AnchorsDATA[idx_feat].iKeyFrame[feat_iKey_idx].MatchedPoint = point_match ;
            }

           
          }  // for(unsigned int j =0 ; j< matches.size();j++ )

          int vlink = Vgraph(KeyFDATA.size()-1,i); // get visual link strenght of the last keyFrame to the i-Keyframe from the visual graph 
  
          if (vlink == 0)
          {
            n_zero_vlink++;
          }
          // For local bundle-ajustment, only match features (anchors) for a sub-set of KeyFrames, in this case:
          // If there is no visual link from the last KFrame to the i-Kframe for n consecutive Kframes break (stop the matching)
          if (n_zero_vlink > PAR.map.n_consec_kf_wo_link_match)
          {
            break;
          }

    } // for(int i = KeyFDATA.size()-1; i >= 0 ; i--)

    

    // Remove KeyFrames with zero matches. ----------------------------------
    int FFD_size = KeyFDATA.size();    
    
    if ((FFD_size > 1)&&(idx_kf_match >= 0))
    { 
      for(int i = FFD_size-1; i >= idx_kf_match ; i--)
      { 
          if(KeyFDATA[i].MatchedFeats_idx.size()== 0 )
          {
            
            // First Remove Bad KeyFrame from anchors
            for(int j = 0; j < KeyFDATA[i].PredictedFeats_idx.size();j++)
            {
                int idx_f = KeyFDATA[i].PredictedFeats_idx[j];
                
                for(int k = 0;k < AnchorsDATA[idx_f].iKeyFrame.size();k++)
                {
                  if(AnchorsDATA[idx_f].iKeyFrame[k].KeyF_idx==i) // if index of deleted KeyFrame is associated to the Anchor
                  {                       
                      AnchorsDATA[idx_f].iKeyFrame.erase(AnchorsDATA[idx_f].iKeyFrame.begin()+k);
                  }
                }
            }
            // Delete KeyFrame
            KeyFDATA.erase(KeyFDATA.begin()+i);

            // Delete KeyFrame from Visual Graph
            Vgraph.shed_row(i);
            Vgraph.shed_col(i);  
                     
            
          }//if(KeyFDATA[i].MatchedFeats_idx.size()== 0 )

      } //for(int i = FFD_size-1; i >= idx_kf_match ; i--) 
    } // if ((FFD_size > 1)&&(idx_kf_match >= 0))
    //--------------------------------------------------------------------
   
   int q = 10;
}
//------------------------------------------------------------------------------------------------------
/*
int check_visibiliy_graph(arma::mat &Vgraph)
{
  
  if (Vgraph.n_cols < 4)
  {
    return 0;
  }
  else
  { 
    Vgraph.print();
    bool flag_zero = false;
    int idx_i;
    for(int i = Vgraph.n_cols-2; i >= 0 ;  i-- )
    {      
      int vlink =  Vgraph(Vgraph.n_rows-1,i);    
      if (vlink == 0)
      {
       flag_zero = true;
       idx_i = i + 1;
       break;        
      }    
    } 
    if (flag_zero == false)
    {
      return 0;
    }
    else
    { 
      bool flag_zero2 = false;
      int idx_j;
      for(int j = idx_i-1; j >= 0 ; j--)
      {
        int vlink =  Vgraph(idx_i,j);
        if (vlink == 0)
          {
          flag_zero2 = true;
          idx_j = j + 1;
          break;        
          }            
      }
      if (flag_zero2 == false)
      {
        return 0;
      }
      else
      {
        return idx_j;
      }
        
    } // else 2
      
  } // else 1
 
}
*/
//---------------------------------------------------------------------------------------------------------
#define RATIO    0.85
//#define RATIO    0.85
void matchFeatures3(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches) 
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