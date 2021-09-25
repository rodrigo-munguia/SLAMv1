#include "Jac_XYZ_uvr.h"

#include "../Transforms/quat2R.h"
#include <math.h>

/*
%**************************************************************************
% R. Munguia 2020
% nf = [xi yi zi]';
% Jacobian of "nf" with respect to system state "x_v" and image point "uv" 

% x meaning
% index  1  2  3  4  5   6   7   8  9  10  11  12  13  
%       q1 q2 q3 q4 w_x w_y w_z  x  y  z  v_x v_y v_z 

%dy_dx =  [ dx_dq     0(1x3)  1 0 0  0(1x3) ]
%         [ dy_dq     0(1x3)  0 1 0  0(1x3) ]
%         [ dz_dq     0(1x3)  0 0 1  0(1x3) ]

%dy_dR =  [   dx_duv   dx_dr  ]
%         [   dy_duv   dy_dr  ]
%         [   dz_duv   dz_dr  ]

%  %inverse projection model
%  [hc,dhc_duv] = Inverse_Projection_Model(uv,1,0,1,cam_parameters);
%  Rn2c  = quat2R(VAR.x(1:4)); % actual  navigation to camera rotation matrix
%  Rc2n = Rn2c';  %  actual camera to navigation rotation matrix
%  Tn2c = VAR.x(8:10);

%  hn = Rc2n*hc; % % vector h pointing in the direction of the undistorted image point, expressed in the navigation frame
%  m = hn/norm(hn);

%  new_yi = Tn2c + d*m; 
*/

void Jac_XYZ_uvr(cv::Point2d uvd, arma::vec& x, parameters &par,double depth,arma::mat::fixed<3,13>& dy_dx, arma::mat::fixed<3,3>& dy_duvr)
{
        
    arma::vec::fixed<3> hc;
    arma::mat::fixed<3,2> dhc_duvd;    

    hc = Inverse_projection_model(uvd,1,true, par.img.cam_parameters,dhc_duvd); // compute inverse projection model
   
     std::vector<double> quat;
    quat = arma::conv_to<vec_t>::from(x.subvec(0,3)); // convert from armadillo vector to c++ vector
    double Rn2c_a[9];

    quat2R(&quat[0],Rn2c_a);
    
    arma::mat Rn2c(Rn2c_a,3,3); // convert from arrat to armadillo

    arma::mat::fixed<3,3> Rc2n = Rn2c.t();

    arma::vec::fixed<3> hn = Rc2n*hc;

    arma::vec::fixed<3> m = hn/arma::norm(hn); // normalized vector (in the nav frame) pointing in the direction of the feature

    
    double hn1 = hn(0);
    double hn2 = hn(1);
    double hn3 = hn(2);
    
    double rhn = pow(hn1*hn1 + hn2*hn2 + hn3*hn3,1.5);

    arma::rowvec::fixed<3> dnormhn_dhn = { -hn1/rhn, -hn2/rhn, -hn3/rhn};

    arma::mat::fixed<3,3> dm_dhn = hn*dnormhn_dhn + arma::eye(3,3)*(1/arma::norm(hn));

    arma::mat::fixed<3,3> dP_dhn = depth*arma::eye(3,3)*dm_dhn;

    double q1 = x(0);
    double q2 = x(1);
    double q3 = x(2);
    double q4 = x(3);
    double hc1 = hc(0);
    double hc2 = hc(1);
    double hc3 = hc(2);

    arma::mat::fixed<3,4> dhn_dq = {{ 2*hc1*q1 - 2*hc3*q3 + 2*hc2*q4, 2*hc1*q2 + 2*hc2*q3 + 2*hc3*q4, 2*hc2*q2 - 2*hc3*q1 - 2*hc1*q3, 2*hc2*q1 + 2*hc3*q2 - 2*hc1*q4},
                                    { 2*hc2*q1 + 2*hc3*q2 - 2*hc1*q4, 2*hc3*q1 - 2*hc2*q2 + 2*hc1*q3, 2*hc1*q2 + 2*hc2*q3 + 2*hc3*q4, 2*hc3*q3 - 2*hc1*q1 - 2*hc2*q4},
                                    { 2*hc3*q1 - 2*hc2*q2 + 2*hc1*q3, 2*hc1*q4 - 2*hc3*q2 - 2*hc2*q1, 2*hc1*q1 - 2*hc3*q3 + 2*hc2*q4, 2*hc1*q2 + 2*hc2*q3 + 2*hc3*q4}};

    arma::mat::fixed<3,4> dP_dq = dP_dhn*dhn_dq;


    //arma::mat::fixed<3,13> dy_dx;
    dy_dx.zeros();

    dy_dx(arma::span(0,2),arma::span(0,3)) = dP_dq;
    dy_dx(arma::span(0,2),arma::span(7,9)) = arma::eye(3,3);
    
    //--------------------------------

    arma::mat::fixed<3,2> dP_duv = dP_dhn*Rc2n*dhc_duvd;

    //arma::mat::fixed<3,3> dy_duvr;
    dy_duvr(arma::span(0,2),arma::span(0,1)) = dP_duv;
    dy_duvr(arma::span(0,2),2) = m;


   // dy_duvr.print();

   // int q = 10;


}