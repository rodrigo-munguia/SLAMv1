#ifndef SYS_VISUAL_INIT_ANCHORS_I_H
#define SYS_VISUAL_INIT_ANCHORS_I_H

#include "ekf_types.h"
#include "../parameters.h"
#include <armadillo>
#include "../map/map.h"
#include "../map/map_types.h"
#include "../locks.h"


//-----------------------------------------------------------------------------
//   Initialize anchors
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
using namespace arma;

typedef std::vector<double> vec_t;


void visual_init_anchors(arma::vec& x, arma::mat& P, parameters &par, vectorFeat& FeatsDATA, vectorFeat& AnchorsDATA,GMAP &gmap,LOCKS &locks );


#endif