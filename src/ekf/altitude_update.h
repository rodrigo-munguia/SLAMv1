#ifndef ALT_U_H
#define ALT_U_H


#include <armadillo>
#include "../parameters.h"
#include "../getData.h"

//-----------------------------------------------------------------------------
//   
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  

void Altitude_Update_bar(arma::vec& x, arma::mat& P,BAR &bar,parameters &par);

void Altitude_Update_alt(arma::vec& x, arma::mat& P,ALT &alt,parameters &par);



#endif