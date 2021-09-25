#ifndef SYS_VISUAL_update_I_H
#define SYS_VISUAL_update_I_H



#include "ekf_types.h"
#include <armadillo>
#include "../parameters.h"
#include "../getData.h"
#include "opencv2/opencv.hpp"
#include "../vision/vision.h"







//-----------------------------------------------------------------------------
//   Update EKF with visual measurements
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
using namespace arma;
typedef std::vector<double> vec_t;


void visual_update_f(arma::vec& x, arma::mat& P, parameters &par, vectorFeat& FeatsDATA, vectorFeat& AnchorsDATA );

void visual_update_feats(arma::vec& x, arma::mat& P, parameters &par, vectorFeat& FeatsDATA);

void visual_update_anchors(arma::vec& x, arma::mat& P, parameters &par, vectorFeat& AnchorsDATA );

#endif