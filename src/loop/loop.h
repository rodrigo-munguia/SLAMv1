#ifndef LOOP_H
#define LOOP_H

#include <armadillo>
#include "../parameters.h"
#include "opencv2/opencv.hpp"
#include "../locks.h"
#include "../map/map.h"
#include "../map/map_types.h"
#include "../vision/vision.h"
#include "cost_function_cl.h"




using namespace arma;


class LOOP
{

    public:
        
        
        LOOP(parameters &par) // constructor
        {
            PAR = par;
            newFrame = false;
            closing_loop_active = false;
            new_xy_position_available = false;          
            
        }

        
        void update(GMAP &gmap,LOCKS &locks);
       
        
        bool newFrame;
        KEYFRAME Current_KF;

        bool closing_loop_active;

        bool new_xy_position_available;

        arma::vec::fixed<2> get_new_xy_pos();
        
    private:

       arma::vec::fixed<2> new_pos_measurement;
       


        parameters PAR;    
        

        

    
};





#endif