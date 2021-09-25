
#include "ekf.h"



void EKF::state_init()
{

    system_init(x,P,PAR);

}

void EKF::prediction(double delta_t)
{

    prediction_state(x,P,PAR,delta_t);

}

void EKF::visual_update(FRAME *frame,GMAP &gmap,LOOP &cloop,LOCKS &locks)
{

    
    if (FeatsDATA.size() > 0)
    {
        visual_delete_feats(x, P,PAR,FeatsDATA, AnchorsDATA  );
        
        if(PAR.sys.EKF_initialize_anchors == true)
            visual_init_anchors(x, P,PAR, FeatsDATA, AnchorsDATA,gmap,locks );

        MatchInfo Mi = visual_match_feats(x,P,PAR,frame,FeatsDATA,AnchorsDATA,gmap,cloop,locks);

        visual_update_f(x,P,PAR,FeatsDATA,AnchorsDATA);

    }

    if (frame->range > 0) // if range measurement is associated with the frame
    {
        visual_init_w_range(x,P,PAR,frame,FeatsDATA,AnchorsDATA);
        
    }
    
    

}

void EKF::closing_loop_position_update(arma::vec::fixed<2> xy_pos)
{
    //this->AnchorsDATA.clear(); 
    //cl_position_update(x, P,xy_pos,PAR);
    cl_position_update2(x, P,xy_pos,PAR,FeatsDATA,AnchorsDATA);
}


void EKF::altitude_update_bar(BAR &bar)
{

   Altitude_Update_bar(x,P,bar,PAR);
    
}
void EKF::altitude_update_alt(ALT &alt)
{

    Altitude_Update_alt(x,P,alt,PAR);
}

void EKF::attitude_update(ATT &att)
{
    Attitude_Update(x,P,att,PAR);
}


void EKF::store_data_for_plot(LOCKS &locks,FRAME *frame)
{   
    
   STORE_data_for_plot(x,PAR,locks,store,FeatsDATA,AnchorsDATA,frame);


}



void EKF::store_gps(GPS &gps)
{  
  
  Store_gps(gps,PAR,store);

}