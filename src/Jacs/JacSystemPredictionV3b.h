//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: JacSystemPredictionV3b.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Aug-2020 18:29:58
//
#ifndef JACSYSTEMPREDICTIONV3B_H
#define JACSYSTEMPREDICTIONV3B_H

// Include Files
#include <cstddef>
#include <cstdlib>
//#include "rtwtypes.h"
//#include "JacSystemPredictionV3b_types.h"

// Function Declarations
extern void JacSystemPredictionV3b(const double x[13], double delta_t, const
  double k[2], double JFx[169], double JFu[78]);

#endif

//
// File trailer for JacSystemPredictionV3b.h
//
// [EOF]
//
