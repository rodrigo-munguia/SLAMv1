#ifndef GSTORE_H
#define GSTORE_H

#include "../ekf/ekf_types.h"
#include "map_types.h"
#include "../getData.h"
#include "../Transforms/quat2R.h"
#include "../locks.h"
#include "../Transforms/Geo2ECEF.h"
#include "../Transforms/Euler_to_Ra2b.h"


typedef std::vector<double> vec_t;

void GSTORE_data_for_plot(parameters &par,LOCKS &locks,GSTORE &store,vectorFeat& AnchorsDATA, vectorKeyF& KeyFDATA  );




#endif 