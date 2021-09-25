#ifndef SYS_VISUALPROC_I_H
#define SYS_VISUALPROC_I_H



#include "ekf_types.h"
#include <armadillo>
#include "../parameters.h"
#include "../getData.h"
#include "opencv2/opencv.hpp"
#include "../vision/vision.h"







//-----------------------------------------------------------------------------
//   Initialize visual features with range measurements
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
using namespace arma;

typedef std::vector<double> vec_t;


void visual_init_w_range(arma::vec& x, arma::mat& P, parameters &par,FRAME *frame, vectorFeat& FeatsDATA , vectorFeat& AnchorsDATA   );

void init_XYZ_feat_point(arma::vec& x, arma::mat& P, parameters &par,FRAME *frame,cv::KeyPoint kp,cv::Mat Descriptor, vectorFeat& FeatsDATA);

void remove_close_points(vector<cv::KeyPoint>& Kp_init, vector<cv::KeyPoint>&  sscKP, parameters &par, vectorFeat& FeatsDATA , vectorFeat& AnchorsDATA );


#endif