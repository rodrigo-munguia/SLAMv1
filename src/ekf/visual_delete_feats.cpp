#include "visual_delete_feats.h"





void visual_delete_feats(arma::vec& x, arma::mat& P, parameters &par, vectorFeat& FeatsDATA, vectorFeat& AnchorsDATA  )
{

    if (FeatsDATA.size() > 0)
    {

        for(int i = 0; i < FeatsDATA.size(); i++ )
        {

            int Ttimes = FeatsDATA[i].times_mathed + FeatsDATA[i].times_not_mathed;
            float RatioMatched = (float)FeatsDATA[i].times_mathed/(float)FeatsDATA[i].times_not_mathed;
                if((Ttimes > 10))
                    if((RatioMatched < .3)||(FeatsDATA[i].times_not_considered > 10))
                        {
                            
                            delete_i_feat_(i, x,  P, FeatsDATA);
                            i--;
                            
                        }
                


        }

        
    }

    //---------------------------------------------------------------
    
    
    if (AnchorsDATA.size() > 0)
    {

        for(int i = 0; i < AnchorsDATA.size(); i++ )
        {

            int Ttimes = AnchorsDATA[i].times_mathed + AnchorsDATA[i].times_not_mathed;
            float RatioMatched = (float)AnchorsDATA[i].times_mathed/(float)AnchorsDATA[i].times_not_mathed;
               // if((Ttimes > 5))
               //     if((RatioMatched < .3)||(AnchorsDATA[i].times_not_considered > 5))
               if((AnchorsDATA[i].times_not_considered > 5))
                        {
                            
                            delete_i_anchor_(i,  AnchorsDATA);
                            i--;
                            
                        }
                


        }

        
    }
   


}
//--------------------------------------------------------------------------------------------------
void delete_i_anchor_(int idx, vectorFeat& AnchorsDATA)
{
    // remove anchor from table
    AnchorsDATA.erase(AnchorsDATA.begin()+idx);

    
}


//------------------------------------------------------------------------------------------------

void delete_i_feat_(int idx, arma::vec& x, arma::mat& P, vectorFeat& FeatsDATA)
{

     int idx_i = FeatsDATA[idx].idx_i_state;
     int idx_f = FeatsDATA[idx].idx_f_state;
                            
    // remove feature from state
    x.shed_rows(idx_i,idx_f);
    P.shed_rows(idx_i,idx_f);
    P.shed_cols(idx_i,idx_f);
    FeatsDATA.erase(FeatsDATA.begin()+idx);
                            
    // Update FeatsData
    for(int j = idx; j<FeatsDATA.size(); j++)
    {
        FeatsDATA[j].idx_i_state = FeatsDATA[j].idx_i_state - 3;
        FeatsDATA[j].idx_f_state = FeatsDATA[j].idx_f_state - 3; 
    }



}