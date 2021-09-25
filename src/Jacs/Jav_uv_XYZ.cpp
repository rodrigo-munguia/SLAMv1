#include "Jac_uv_XYZ.h" 
#include "../Transforms/quat2R.h"
#include <math.h>
/*
% Initial States ***************************************************
% x meaning
% index  0  1  2  3  4   5   6   7  8  9  10  11  12  
%       q1 q2 q3 q4 w_x w_y w_z  x  y  z  v_x v_y v_z 
*/



void Jac_uv_XYZ(arma::vec::fixed<3> pxyz, arma::vec& x, parameters &par,arma::mat::fixed<2,13>& duv_dx, arma::mat::fixed<2,3>& duv_dy)
{

    std::vector<double> quat;
    quat = arma::conv_to<vec_t>::from(x.subvec(0,3)); // convert from armadillo vector to c++ vector
    double Rn2c_a[9];
    quat2R(&quat[0],Rn2c_a);
    arma::mat Rn2c(Rn2c_a,3,3); // convert from arrat to armadillo

    arma::vec::fixed<3> Tn2c = x.subvec(7,9);

    arma::vec::fixed<3> pn = (pxyz - Tn2c);

    arma::vec::fixed<3> pc = Rn2c*pn;

    arma::mat::fixed<2,3> duv_dPc;
    cv::Point2d uvd = Projection_model(pc,1,true,par.img.cam_parameters,duv_dPc );

     double q1 = x(0);
    double q2 = x(1);
    double q3 = x(2);
    double q4 = x(3);
    double Pn1 = pn(0);
    double Pn2 = pn(1);
    double Pn3 = pn(2);
     
    arma::mat::fixed<3,4> dPc_dq; 
    if (par.ekf.visual_attitude_update == true)
    {
        dPc_dq = {{ 2*Pn3*q1 + 2*Pn2*q2 - 2*Pn1*q3, 2*Pn2*q1 - 2*Pn3*q2 + 2*Pn1*q4, 2*Pn2*q4 - 2*Pn3*q3 - 2*Pn1*q1, 2*Pn1*q2 + 2*Pn2*q3 + 2*Pn3*q4},
                  { 2*Pn1*q1 + 2*Pn3*q3 - 2*Pn2*q4, 2*Pn1*q2 + 2*Pn2*q3 + 2*Pn3*q4, 2*Pn3*q1 + 2*Pn2*q2 - 2*Pn1*q3, 2*Pn3*q2 - 2*Pn2*q1 - 2*Pn1*q4},
                  { 2*Pn2*q1 - 2*Pn3*q2 + 2*Pn1*q4, 2*Pn1*q3 - 2*Pn2*q2 - 2*Pn3*q1, 2*Pn1*q2 + 2*Pn2*q3 + 2*Pn3*q4, 2*Pn1*q1 + 2*Pn3*q3 - 2*Pn2*q4}};
    }
    else
    {
        dPc_dq.zeros();
    }
    
    arma::mat::fixed<3,13> dPc_dx;
    dPc_dx.zeros();
    dPc_dx(arma::span(0,2),arma::span(0,3)) = dPc_dq;
    dPc_dx(arma::span(0,2),arma::span(7,9)) = -1*Rn2c;
    
    //----------
    duv_dx = duv_dPc*dPc_dx;
    //-----------

    duv_dy = duv_dPc*Rn2c;


}
