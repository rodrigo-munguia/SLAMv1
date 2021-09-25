#include "map.h"
#include "../Transforms/quat2R.h"
#include "../vision/vision.h"


//------------------------------------------------------------------------------
void GMAP::update(LOCKS &locks)
{
    
    if (Full_update_VG == true)
    {
     
      Full_Update_Visibility_Graph3();
      Full_update_VG = false;
    }

   // bool new_kf = Add_KeyFrames();
    bool new_kf = Add_KeyFrames2();

    if ((new_kf == true)&&(KeyFDATA.size()>1))
    {
        visual_match2();
    }
    
    if ((new_kf == true)&&(KeyFDATA.size()>2))
    {
        delete_ak2();

       
        local_bundle_ajustment2(locks); 


        GSTORE_data_for_plot(PAR,locks,store,AnchorsDATA,KeyFDATA  );                  
        
    }
    

}



//------------------------------------------------------------------------------------------------------
void GMAP::check_visibiliy_graph(int &idx_direct_link, int &idx_indirect_link )
{
  idx_direct_link = 0;
  if (Vgraph.n_cols < 4)
  {
    idx_indirect_link = 0;
  }
  else
  { 
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







