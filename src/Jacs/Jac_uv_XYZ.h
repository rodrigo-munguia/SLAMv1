#ifndef SYS_JAC_uv_xyz_H
#define SYS_JAC_uv_xyz_H




#include <armadillo>
#include "../parameters.h"
#include "../vision/vision.h"
#include "opencv2/opencv.hpp"


/*
%**************************************************************************
% R. Munguia 2020
% nf = [xi yi zi]';
% Jacobian of of the image point uv with respect to the euclidena point nf
*/

using namespace arma;

typedef std::vector<double> vec_t;


void Jac_uv_XYZ(arma::vec::fixed<3> pxyz, arma::vec& x, parameters &par,arma::mat::fixed<2,13>& duv_dx, arma::mat::fixed<2,3>& duv_dy);



#endif