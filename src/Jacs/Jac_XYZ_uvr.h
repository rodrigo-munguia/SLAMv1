

#ifndef SYS_JACxyz_H
#define SYS_JACxyz_H




#include <armadillo>
#include "../parameters.h"
#include "../vision/vision.h"
#include "opencv2/opencv.hpp"

/*
%**************************************************************************
% R. Munguia 2020
% nf = [xi yi zi]';
% Jacobian of "nf" with respect to system state "x_v" and image point "uv" 
*/

using namespace arma;

typedef std::vector<double> vec_t;

void Jac_XYZ_uvr(cv::Point2d uvd, arma::vec& x, parameters &par,double depth,arma::mat::fixed<3,13>& dy_dx, arma::mat::fixed<3,3>& dy_duvr);


#endif