#ifndef STORE_H
#define STORE_H

#include "ekf_types.h"
#include "../getData.h"
#include "../Transforms/quat2R.h"
#include "../locks.h"
#include "../Transforms/Geo2ECEF.h"
#include "../Transforms/Euler_to_Ra2b.h"


typedef std::vector<double> vec_t;

void STORE_data_for_plot(arma::vec& x, parameters &par,LOCKS &locks,STORE &store, vectorFeat& FeatsDATA, vectorFeat& AnchorsDATA,FRAME *frame);


void Store_gps(GPS &gps,parameters &par,STORE &store);



#endif 



