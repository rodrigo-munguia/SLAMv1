#include "visual_init_anchors.h"
#include "visual_delete_feats.h"


void visual_init_anchors(arma::vec& x, arma::mat& P, parameters &par, vectorFeat& FeatsDATA, vectorFeat& AnchorsDATA,GMAP &gmap,LOCKS &locks)
{
    static int id_anchor_count=0; 

    // Check state-features convergence to initialize nwe anchors
    if (FeatsDATA.size() > 0)
    {
        
        for(int i = 0; i < FeatsDATA.size(); i++ )
        {

            int idx_i = FeatsDATA[i].idx_i_state;
            int idx_f = FeatsDATA[i].idx_f_state;
                            
            double depth =   arma::norm(FeatsDATA[i].CameraState - x.subvec(idx_i,idx_f));

            double r_uncertanty = sqrt( 9*P(idx_i,idx_i)  + 9*P(idx_i+1,idx_i+1) + 9*P(idx_i+2,idx_i+2) );

            if (r_uncertanty/depth < par.ekf.Uncertainty_depth_anchor_init)
            {
                // Add anchor to table, an delete feature from the state
                FEAT Anchor;  // feature data structure 
                Anchor = FeatsDATA[i];
                Anchor.AnchorState = x.subvec(idx_i,idx_f);

                //Anchor.AnchorState(2) =  Anchor.AnchorState(2) - .2;

                Anchor.idx_i_state = -1;
                Anchor.idx_f_state = -1;
                Anchor.ekf_to_gmap_status = 1;
                Anchor.id_anchor = id_anchor_count;
                
                
                AnchorsDATA.push_back(Anchor);

                id_anchor_count++; 
                   
                delete_i_feat_(i, x,  P, FeatsDATA);
                i--;

                //Also add anchor to the pile of new anchors of the global map
                  
                locks.NewEKFAnchorsDATA_mtx.lock();
                    gmap.Push_NewAnchorsDATA.push_back(Anchor);
                locks.NewEKFAnchorsDATA_mtx.unlock();
                
            }               


        }
// Update anchors from the global map process

    
        // check for optimized/new anchors computed by the global map process         
        if (gmap.Pull_NewAnchorsDATA.size()>0) // if there is anchors initialized/optimized by the global map process        
        {    
            locks.Pull_NewAnchorsDATA_mtx.lock();

            for(int i =0; i < AnchorsDATA.size(); i++ )
            {
                int id_a = AnchorsDATA[i].id_anchor; // get id of i-anchor
                
                if (AnchorsDATA[i].init_type == 1) // if was initialized by the global map
                {
                  AnchorsDATA.erase (AnchorsDATA.begin()+i);
                       i--;  
                }

                for(int j= 0; j < gmap.Pull_NewAnchorsDATA.size();j++ )
                {
                   if(id_a == gmap.Pull_NewAnchorsDATA[j].id_anchor ) // if anchor has been optimized by the global map
                   {
                       AnchorsDATA.erase (AnchorsDATA.begin()+i);
                       i--;
                   }    

                }

            }    

            cout << AnchorsDATA.size() << "  " << gmap.Pull_NewAnchorsDATA.size() << endl;
             
            for(int i = 0 ; i < gmap.Pull_NewAnchorsDATA.size(); i++)
            { 
            if (gmap.Pull_NewAnchorsDATA[i].id_anchor < 0)
              {
                  gmap.Pull_NewAnchorsDATA[i].id_anchor = id_anchor_count;
                  id_anchor_count++;
              }  
              AnchorsDATA.push_back(gmap.Pull_NewAnchorsDATA[i]);
             

            }
            //cout << "new optimized anchors" << endl;
            
                gmap.Pull_NewAnchorsDATA.clear();
            locks.Pull_NewAnchorsDATA_mtx.unlock();    

            int q = 10;

        }
        
        
  
     




    }










}
