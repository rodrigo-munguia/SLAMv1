//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: JacSystemPredictionV3b.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 31-Aug-2020 18:29:58
//

// Include Files
#include "JacSystemPredictionV3b.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// Initial States ***************************************************
//  x meaning
//  index  1  2  3  4  5   6   7   8  9  10  11  12  13
//        q1 q2 q3 q4 w_x w_y w_z  x  y  z  v_x v_y v_z
//  Attitude states
//  x(1:4)=   [q1 q2 q3 q4] -> quaternion orientation  (navigation to body rotation)
//  x(5:7)=   [w_x w_y w_z ] ->  vel rotation in the navigation frame
//  Position states
//  x(8:10)= [x  y  z]  ->  Position in the navigation coordinate frame.
//  x(11:13)= [v_x v_y v_z]  -> Velocity in navigation coordinate frame.
// Arguments    : const double x[13]
//                double delta_t
//                const double k[2]
//                double JFx[169]
//                double JFu[78]
// Return Type  : void
//
void JacSystemPredictionV3b(const double x[13], double delta_t, const double k[2],
  double JFx[169], double JFu[78])
{
  double scale;
  double b_x;
  double W_tmp;
  double W[3];
  double absxk;
  double t;
  double w;
  double c_x;
  double d_x;
  double sinwow;
  double W_mat[16];
  double qmult_tmp;
  double unnamed_idx_0;
  double unnamed_idx_1;
  double unnamed_idx_2;
  double unnamed_idx_3;
  double dcosw_dnw_tmp;
  double dFs_dw[3];
  int i;
  signed char dfp_dp[9];
  double b_W_mat[4];
  double b_sinwow[12];
  int i1;
  double dv[12];
  int JFx_tmp;
  static const signed char b[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  int b_JFx_tmp;
  signed char i2;
  static const signed char a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  // {
  //  Attitude equations
  //
  //           % x(1:4) = (cos(w)*eye(4,4) + W_mat*sinwow)*x(1:4); % eqn. D.36
  //           %(aided navigation)
  //           % where
  //
  //             % sinwow = sin(w)/w;
  //             % w   = norm(W);
  //             %
  //           x(5:7) =   (eye(3) - eye(3)*k.r*delta_t)*x(5:7);    +  ?W;
  //
  //    % Position Equations
  //          x(8:10) = x(8:10) + x(11:13)*delta_t;
  //          x(11:13) =  (eye(3) - eye(3)*k.t*delta_t)x(11:13)  +  VW ;
  //
  // where VW and ?W is an unknown linear and angular velocity,
  // with linear and angular acceleration with zero-mean and
  // known Gaussian process covariance, aW and ?W, respectively:
  //
  // VW = aW*delta_t;       aW = sigma_w
  // ?W = ?W*delta_t;       ?W  =  sigma_a
  //
  //    U = [ eye(3,3)*(sigma_w*delta_t)^2   zeros(3,3)
  //                 zeros(3,3)   eye(3,3)*(sigma_a*delta_t)^2];
  //
  //  %}
  //  JFx = [ dfq_dq  dfq_dw   0       0    ]
  //        [   0     dfw_dw   0       0    ]
  //        [   0       0    dfp_dp  dfp_dv ]
  //        [   0       0      0     dfv_dv ]
  //  JFu = [   0       0    ]
  //        [ duw_dsw   0    ]
  //        [   0       0    ]
  //        [   0    duv_dsv ]
  scale = 3.3121686421112381E-170;
  b_x = x[4] * delta_t;
  W_tmp = b_x / 2.0;
  W[0] = W_tmp;
  absxk = std::abs(W_tmp);
  if (absxk > 3.3121686421112381E-170) {
    w = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    w = t * t;
  }

  c_x = x[5] * delta_t;
  W_tmp = c_x / 2.0;
  W[1] = W_tmp;
  absxk = std::abs(W_tmp);
  if (absxk > scale) {
    t = scale / absxk;
    w = w * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    w += t * t;
  }

  d_x = x[6] * delta_t;
  W_tmp = d_x / 2.0;
  absxk = std::abs(W_tmp);
  if (absxk > scale) {
    t = scale / absxk;
    w = w * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    w += t * t;
  }

  w = scale * std::sqrt(w);
  if (w == 0.0) {
    sinwow = 1.0;
  } else {
    sinwow = std::sin(w) / w;
  }

  W_mat[0] = 0.0;
  W_mat[4] = -W[0];
  W_mat[8] = -W[1];
  W_mat[12] = -W_tmp;
  W_mat[1] = W[0];
  W_mat[5] = 0.0;
  W_mat[9] = -W_tmp;
  W_mat[13] = W[1];
  W_mat[2] = W[1];
  W_mat[6] = W_tmp;
  W_mat[10] = 0.0;
  W_mat[14] = -W[0];
  W_mat[3] = W_tmp;
  W_mat[7] = -W[1];
  W_mat[11] = W[0];
  W_mat[15] = 0.0;
  qmult_tmp = std::cos(w);

  //  eqn. D.36
  // ******************
  // ******************
  //   NOTA: falta "descomponer" dfq_dw utilizando la regla de la cadena.
  // -------------------------------
  unnamed_idx_0 = x[0];
  unnamed_idx_1 = x[1];
  unnamed_idx_2 = x[2];
  unnamed_idx_3 = x[3];

  // cw = cos(w);
  dcosw_dnw_tmp = std::sin(w);
  scale = std::abs(b_x);
  absxk = std::abs(c_x);
  t = std::abs(d_x);
  if (b_x < 0.0) {
    b_x = -1.0;
  } else if (b_x > 0.0) {
    b_x = 1.0;
  } else {
    if (b_x == 0.0) {
      b_x = 0.0;
    }
  }

  if (c_x < 0.0) {
    c_x = -1.0;
  } else if (c_x > 0.0) {
    c_x = 1.0;
  } else {
    if (c_x == 0.0) {
      c_x = 0.0;
    }
  }

  if (d_x < 0.0) {
    d_x = -1.0;
  } else if (d_x > 0.0) {
    d_x = 1.0;
  } else {
    if (d_x == 0.0) {
      d_x = 0.0;
    }
  }

  W_tmp = 4.0 * std::sqrt((scale * scale / 4.0 + absxk * absxk / 4.0) + t * t /
    4.0);
  W[0] = delta_t * scale * b_x / W_tmp;
  W[1] = delta_t * absxk * c_x / W_tmp;
  W[2] = delta_t * t * d_x / W_tmp;

  // --------------------------------
  if (w == 0.0) {
    dFs_dw[0] = 0.0;
    dFs_dw[1] = 0.0;
    dFs_dw[2] = 0.0;
  } else {
    scale = 1.0 / w * qmult_tmp + -dcosw_dnw_tmp / (w * w);
    dFs_dw[0] = scale * W[0];
    dFs_dw[1] = scale * W[1];
    dFs_dw[2] = scale * W[2];
  }

  // -------------------------------------------
  // ************************************************************************    
  for (i = 0; i < 9; i++) {
    dfp_dp[i] = 0;
  }

  dfp_dp[0] = 1;
  dfp_dp[4] = 1;
  dfp_dp[8] = 1;

  // **********************************************************
  //  derivatives position
  //  JFx = [ dfq_dq  dfq_dw   0       0    ]
  //        [   0     dfw_dw   0       0    ]
  //        [   0       0    dfp_dp  dfp_dv ]
  //        [   0       0      0     dfv_dv ]
  std::memset(&JFx[0], 0, 169U * sizeof(double));
  for (i = 0; i < 4; i++) {
    b_W_mat[i] = ((W_mat[i] * unnamed_idx_0 + W_mat[i + 4] * unnamed_idx_1) +
                  W_mat[i + 8] * unnamed_idx_2) + W_mat[i + 12] * unnamed_idx_3;
  }

  scale = delta_t * x[1];
  absxk = sinwow * (-scale / 2.0);
  b_sinwow[0] = absxk;
  t = delta_t * x[2];
  W_tmp = sinwow * (-t / 2.0);
  b_sinwow[4] = W_tmp;
  b_x = delta_t * x[3];
  w = sinwow * (-b_x / 2.0);
  b_sinwow[8] = w;
  c_x = sinwow * (delta_t * x[0] / 2.0);
  b_sinwow[1] = c_x;
  b_sinwow[5] = sinwow * (b_x / 2.0);
  b_sinwow[9] = W_tmp;
  b_sinwow[2] = w;
  b_sinwow[6] = c_x;
  b_sinwow[10] = sinwow * (scale / 2.0);
  b_sinwow[3] = sinwow * (t / 2.0);
  b_sinwow[7] = absxk;
  b_sinwow[11] = c_x;
  for (i = 0; i < 3; i++) {
    i1 = i << 2;
    dv[i1] = unnamed_idx_0 * -dcosw_dnw_tmp * W[i] + (b_W_mat[0] * dFs_dw[i] +
      b_sinwow[i1]);
    JFx_tmp = i1 + 1;
    dv[JFx_tmp] = unnamed_idx_1 * -dcosw_dnw_tmp * W[i] + (b_W_mat[1] * dFs_dw[i]
      + b_sinwow[JFx_tmp]);
    JFx_tmp = i1 + 2;
    dv[JFx_tmp] = unnamed_idx_2 * -dcosw_dnw_tmp * W[i] + (b_W_mat[2] * dFs_dw[i]
      + b_sinwow[JFx_tmp]);
    i1 += 3;
    dv[i1] = unnamed_idx_3 * -dcosw_dnw_tmp * W[i] + (b_W_mat[3] * dFs_dw[i] +
      b_sinwow[i1]);
  }

  for (i = 0; i < 4; i++) {
    JFx_tmp = i << 2;
    JFx[13 * i] = qmult_tmp * static_cast<double>(b[JFx_tmp]) + W_mat[JFx_tmp] *
      sinwow;
    b_JFx_tmp = JFx_tmp + 1;
    JFx[13 * i + 1] = qmult_tmp * static_cast<double>(b[b_JFx_tmp]) +
      W_mat[b_JFx_tmp] * sinwow;
    b_JFx_tmp = JFx_tmp + 2;
    JFx[13 * i + 2] = qmult_tmp * static_cast<double>(b[b_JFx_tmp]) +
      W_mat[b_JFx_tmp] * sinwow;
    JFx_tmp += 3;
    JFx[13 * i + 3] = qmult_tmp * static_cast<double>(b[JFx_tmp]) +
      W_mat[JFx_tmp] * sinwow;
  }

  W_tmp = k[0];
  scale = k[1];
  for (i = 0; i < 3; i++) {
    signed char i3;
    int c_JFx_tmp;
    JFx_tmp = i << 2;
    b_JFx_tmp = 13 * (i + 4);
    JFx[b_JFx_tmp] = dv[JFx_tmp];
    JFx[b_JFx_tmp + 1] = dv[JFx_tmp + 1];
    JFx[b_JFx_tmp + 2] = dv[JFx_tmp + 2];
    JFx[b_JFx_tmp + 3] = dv[JFx_tmp + 3];
    i2 = dfp_dp[3 * i];
    i3 = a[3 * i];
    JFx[b_JFx_tmp + 4] = static_cast<double>(i2) - static_cast<double>(i3) *
      W_tmp * delta_t;
    JFx_tmp = 13 * (i + 7);
    JFx[JFx_tmp + 7] = i2;
    c_JFx_tmp = 13 * (i + 10);
    JFx[c_JFx_tmp + 7] = static_cast<double>(i3) * delta_t;
    JFx[c_JFx_tmp + 10] = static_cast<double>(i2) - static_cast<double>(i3) *
      scale * delta_t;
    i1 = 3 * i + 1;
    JFx[b_JFx_tmp + 5] = static_cast<double>(dfp_dp[i1]) - static_cast<double>
      (a[i1]) * W_tmp * delta_t;
    JFx[JFx_tmp + 8] = dfp_dp[i1];
    JFx[c_JFx_tmp + 8] = static_cast<double>(a[i1]) * delta_t;
    JFx[c_JFx_tmp + 11] = static_cast<double>(dfp_dp[i1]) - static_cast<double>
      (a[i1]) * scale * delta_t;
    i1 = 3 * i + 2;
    JFx[b_JFx_tmp + 6] = static_cast<double>(dfp_dp[i1]) - static_cast<double>
      (a[i1]) * W_tmp * delta_t;
    JFx[JFx_tmp + 9] = dfp_dp[i1];
    JFx[c_JFx_tmp + 9] = static_cast<double>(a[i1]) * delta_t;
    JFx[c_JFx_tmp + 12] = static_cast<double>(dfp_dp[i1]) - static_cast<double>
      (a[i1]) * scale * delta_t;
  }

  // *********************************************************
  //  derivatives JFu
  //  JFu = [   0       0    ]
  //        [ duw_dsw   0    ]
  //        [   0       0    ]
  //        [   0    duv_dsv ]
  std::memset(&JFu[0], 0, 78U * sizeof(double));
  for (i = 0; i < 3; i++) {
    i2 = dfp_dp[3 * i];
    JFu[13 * i + 4] = i2;
    JFx_tmp = 13 * (i + 3);
    JFu[JFx_tmp + 10] = i2;
    i2 = dfp_dp[3 * i + 1];
    JFu[13 * i + 5] = i2;
    JFu[JFx_tmp + 11] = i2;
    i2 = dfp_dp[3 * i + 2];
    JFu[13 * i + 6] = i2;
    JFu[JFx_tmp + 12] = i2;
  }
}

//
// File trailer for JacSystemPredictionV3b.cpp
//
// [EOF]
//
