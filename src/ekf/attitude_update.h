#ifndef ATT_U_H
#define ATT_U_H


#include <armadillo>
#include "../parameters.h"
#include "../getData.h"

//-----------------------------------------------------------------------------
//   
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  

void Attitude_Update(arma::vec& x, arma::mat& P,ATT &att,parameters &par);





#endif