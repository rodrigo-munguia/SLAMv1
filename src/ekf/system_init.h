
#ifndef SYS_INT_H
#define SYS_INT_H




#include <armadillo>
#include "../parameters.h"
#include "../Transforms/Euler_to_Ra2b.h"
#include "../Transforms/Ra2b_TO_Quat_a2b.h"



//-----------------------------------------------------------------------------
//   State vector and Covariance matrix initialization
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
using namespace arma;

void system_init(arma::vec& x, arma::mat& P, parameters &par);


#endif