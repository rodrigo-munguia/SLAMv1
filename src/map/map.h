#ifndef MAP_H
#define MAP_H

#include <armadillo>
#include "../parameters.h"
#include "opencv2/opencv.hpp"
#include "map_types.h"
#include "../ekf/ekf_types.h"
#include "../parameters.h"
#include "../locks.h"
#include "../anms/anms.h"
#include "g_store.h"
#include <chrono>  
#include <thread>  



using namespace arma;
typedef std::vector<double> vec_t;

class GMAP
{

    public:
        
        
        GMAP(parameters &par) // constructor
        {
            PAR = par;
            closing_loop_active = false;
            loop_detected = false;
           
            
            
        }

        
        void update(LOCKS &locks);
       
        vectorKeyF NewKeyFDATA;  // The EKF-SLAM puts here new KeyFrames

        vectorKeyF NewKeyFDATAtmp;  // temporal pile

        vectorFeat Push_NewAnchorsDATA; // The EKF-SLAM puts here new anchors computed by the EKF-SLAM

        vectorFeat Pull_NewAnchorsDATA; // The EKF-SLAM get from here new and optimized anchors computed by the global map process
    //private:

        
       
       vectorKeyF KeyFDATA;  // Structure for storing KeyFrames
        vectorFeat AnchorsDATA; // Structure for storing map points (anchors)

        arma::mat Vgraph; // visibility graph 

        bool closing_loop_active;
        bool loop_detected;

        bool Full_update_VG = false;       
        
        GSTORE store;

        
    private:

        void Full_Update_Visibility_Graph2();
        void Full_Update_Visibility_Graph3();
       

        vectorFeat NewAnchorsDATA; // The EKF-SLAM puts here new anchors

        


        bool Add_KeyFrames();
        bool Add_KeyFrames2();

        void visual_match();
        void visual_match2();

        void local_bundle_ajustment(LOCKS &locks);
        void local_bundle_ajustment2(LOCKS &locks);
        
        void delete_ak2();
        void delete_ak();
        

        void check_visibiliy_graph(int &idx_direct_link, int &idx_indirect_link  );

        void Update_Visibility_Graph2();

        void Init_new_anchors2();

        
        
        
        


        parameters PAR;    
        

        

    
};





#endif