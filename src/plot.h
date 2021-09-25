#ifndef PLOT_H
#define PLOT_H

#include "ekf/ekf.h"
#include "map/map.h"
#include "parameters.h"
#include <opencv2/viz.hpp>

using namespace cv;
using namespace std;


class PLOT
{



public:
  
  PLOT(parameters &par) // cosntructor
        {
            PAR = par;
        }    

  void init(EKF &ekf,GMAP &gmap,LOCKS &locks);
  
  void Update(EKF &ekf,GMAP &gmap);
 
  
  //viz::Viz3d *viewer;

private:
   
   static void KeyboardViz3d(const viz::KeyboardEvent &w, void *t);
   //viz::Viz3d viewer;
   //cv::viz::WCloud *cloud;
   //std::vector<cv::Point3d> EKFmap;   

   static parameters PAR;

};  
 



#endif 