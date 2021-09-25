
#ifndef SYS_VISUALDELETE_I_H
#define SYS_VISUALDELETE_I_H

#include "ekf_types.h"
#include "../parameters.h"
#include <armadillo>


//-----------------------------------------------------------------------------
//   Delete bad features
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
using namespace arma;

typedef std::vector<double> vec_t;


void visual_delete_feats(arma::vec& x, arma::mat& P, parameters &par, vectorFeat& FeatsDATA, vectorFeat& AnchorsDATA  );

void delete_i_feat_(int idx, arma::vec& x, arma::mat& P, vectorFeat& FeatsDATA);

void delete_i_anchor_(int idx, vectorFeat& AnchorsDATA);


#endif