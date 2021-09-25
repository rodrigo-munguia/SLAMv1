#ifndef CL_U_H
#define CL_U_H


#include <armadillo>
#include "../parameters.h"
#include "ekf_types.h"


//-----------------------------------------------------------------------------
// 
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  

void cl_position_update(arma::vec& x, arma::mat& P,arma::vec::fixed<2> xy_pos,parameters &par);

void cl_position_update2(arma::vec& x, arma::mat& P,arma::vec::fixed<2> xy_pos,parameters &par, vectorFeat &FeatsDATA, vectorFeat &AnchorsDATA);





#endif