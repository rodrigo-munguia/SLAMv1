#ifndef PRED_H
#define PRED_H


#include <armadillo>
#include "../parameters.h"




//-----------------------------------------------------------------------------
//   State vector and Covariance matrix initialization
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
using namespace arma;

typedef std::vector<double> vec_t;

void prediction_state(arma::vec& x, arma::mat& P, parameters &par,double delta_t);


#endif