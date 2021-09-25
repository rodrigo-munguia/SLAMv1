
#ifndef EKF_H
#define EKF_H

#include <armadillo>

#include "system_init.h"
#include "prediction.h"
#include "../getData.h"
#include "visual_init_w_range.h"
#include "opencv2/opencv.hpp"
#include "ekf_types.h"
#include "visual_match_feats.h"
#include "visual_update_f.h"
#include "visual_delete_feats.h"
#include "visual_init_anchors.h"
#include "../map/map.h"
#include "../loop/loop.h"
#include "altitude_update.h"
#include "cl_position_update.h"
#include <opencv2/viz.hpp>
#include "store.h"
#include "attitude_update.h"





using namespace arma;

class EKF
{

    public:
        
        bool run = false;
        
        arma::vec x;
        arma::mat P;       

        std::vector<double> x_c, y_c, z_c;
        
        EKF(parameters &par) // cosntructor
        {
            PAR = par;

            x.resize(13);
            P.resize(13,13); 
            
            
        }
        void state_init();

        void prediction(double delta_t);

        void visual_update(FRAME *frame,GMAP &gmap,LOOP &cloop,LOCKS &locks);

        void altitude_update_bar(BAR &bar);
        void altitude_update_alt(ALT &alt);

        void attitude_update(ATT &att);

        void closing_loop_position_update(arma::vec::fixed<2> xy_pos);

        void store_info()
        {
            x_c.push_back(x(7));
            y_c.push_back(-x(8));
            z_c.push_back(-x(9));
        }

        void store_gps(GPS &gps);

        void store_data_for_plot(LOCKS &locks,FRAME *frame); 

        STORE store;
     
    private:
        parameters PAR;

        vectorFeat FeatsDATA;

        vectorFeat AnchorsDATA;

        
          
 

    
};





#endif