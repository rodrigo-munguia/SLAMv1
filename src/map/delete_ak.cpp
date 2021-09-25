#include "map.h"

void GMAP::delete_ak2()
{
    int idx_indirect;
    int idx_direct; 
    check_visibiliy_graph(idx_direct,idx_indirect);


    for(int i = AnchorsDATA.size()-1; i >=0 ; i--)
    {
      bool f_anchor_to_check = true;
      int n_matched = 0;      
      for(int j = 0; j < AnchorsDATA[i].iKeyFrame.size();j++)
      {
        int idx_kf = AnchorsDATA[i].iKeyFrame[j].KeyF_idx;
         
        if(idx_kf >= idx_indirect)
        {
                if (AnchorsDATA[i].iKeyFrame[j].matched == true ) // If idx_kf-KeyFrame must be checked for matches
                {
                    n_matched++;
                }
        }        
        else
        {
           f_anchor_to_check = false;
           break; 
        } 
            
      } // for(int j = 0; j < AnchorsDATA[i].iKeyFrame.size();j++)
      
      int minimun_n_of_kF_matched = PAR.map.min_kf_matched_for_keep_anchor;
       
      if  ((f_anchor_to_check == true)) 
      {   
        if((AnchorsDATA[i].init_type == 1)&&(n_matched<minimun_n_of_kF_matched)) 
          AnchorsDATA.erase(AnchorsDATA.begin()+i);

        if((AnchorsDATA[i].init_type == 0)&&(n_matched<2))
            AnchorsDATA.erase(AnchorsDATA.begin()+i);


          //i--;
          int q = 10;
      } 



    } // for(int i = AnchorsDATA.size()-1; i >=0 ; i--)







}   


void GMAP::delete_ak()
{
    
    if(KeyFDATA.size()>1)    
        for(int i =0 ;i < AnchorsDATA.size();i++)
        {   
            int n_matched = 0;
            for(int j= 0; j < AnchorsDATA[i].iKeyFrame.size();j++)
            {            
                if((AnchorsDATA[i].iKeyFrame[j].MatchedPoint.x > 0)&&(AnchorsDATA[i].iKeyFrame[j].MatchedPoint.y > 0))
                {
                    n_matched++;
                }
            }
            if(n_matched < 2) 
            {
            // delete anchor
            // First Update Keyframes
               for(int j= 0; j < AnchorsDATA[i].iKeyFrame.size();j++)
                {
                    int kf_idx = AnchorsDATA[i].iKeyFrame[j].KeyF_idx;

                    for(int k = 0; k < KeyFDATA[kf_idx].PredictedFeats_idx.size();k++)
                    {   
                        if(KeyFDATA[kf_idx].PredictedFeats_idx[k] > i )
                        {
                            KeyFDATA[kf_idx].PredictedFeats_idx[k]--;
                        } 
                        if(KeyFDATA[kf_idx].PredictedFeats_idx[k] == i )
                        {   
                            KeyFDATA[kf_idx].PredictedFeats_idx.erase(KeyFDATA[kf_idx].PredictedFeats_idx.begin()+k);
                            k--;                                                         
                        }
                        
                    }
                    for(int k = 0; k < KeyFDATA[kf_idx].MatchedFeats_idx.size();k++)
                    {   
                        if(KeyFDATA[kf_idx].MatchedFeats_idx[k] > i )
                        {
                            KeyFDATA[kf_idx].MatchedFeats_idx[k]--;
                        } 
                        if(KeyFDATA[kf_idx].MatchedFeats_idx[k] == i )
                        {   
                            KeyFDATA[kf_idx].MatchedFeats_idx.erase(KeyFDATA[kf_idx].MatchedFeats_idx.begin()+k); 
                            k--;                            
                        }
                        
                    }
                    
                    
                }     

               AnchorsDATA.erase(AnchorsDATA.begin()+i);   
               i--;
            }    
               

        }




}