
#include "prediction.h"
#include "../Jacs/JacSystemPredictionV3b.h"
//#include <cmath>
#include <math.h>
#include <vector>

//-----------------------------------------------------------------------------
//   State vector and Covariance matrix Prediction
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  

/*
% Initial States ***************************************************void STORE_data_for_plot(arma::vec& x, arma::mat& P, parameters &par,LOCKS &locks)
%       q1 q2 q3 q4 w_x w_y w_z  x  y  z  v_x v_y v_z 
% Attitude states
% x(1:4)=   [q1 q2 q3 q4] -> quaternion orientation  (navigation to camera)
% x(5:7)=   [w_x w_y w_z ] ->  vel rotation in the body frame
% Position states
% x(8:10)= [x  y  z]  ->  Position in the navigation coordinate frame
% (navigation to camera)
% x(11:13)= [v_x v_y v_z]  -> Velocity in navigation coordinate frame.
*/
using namespace arma;

void prediction_state(arma::vec& x, arma::mat& P, parameters &par,double delta_t)
{   
    //double delta_t = par.ekf.delta_t;
    double sigma_w = par.ekf.sigma_w;
    double sigma_a = par.ekf.sigma_a;
    
    //-- test data
    /*    arma::vec xin;
        xin.load("x_in.txt");
        //x = xin.subvec(0,12);
        x.set_size( size(xin) );
        x = xin;
        arma::mat Pin;
        Pin.load("P_in.txt");
        P.set_size(size(Pin)); 
        P = Pin;
        delta_t = .04;
        */
    //---
        
    double tau1 = par.ekf.Tau_r;
    double tau2 = par.ekf.Tau_p;

    double kr = 1/tau1;
    double kt = 1/tau2;
   
   
    // Attitude equations
    arma::vec w_u = x.subvec(4,6);
    arma::vec W = w_u*delta_t/2;
    double w = arma::norm(W);

    double sinwow;

    if (w==0)
    {
        sinwow = 1;
    }
    else
    {
        sinwow = sin(w)/w;
    }

    arma::mat W_mat = { {0, -W[0], -W[1], -W[2]},
                        {W[0], 0, -W[2], W[1]},
                        {W[1], W[2], 0, -W[0]},
                        {W[2], -W[1], W[0], 0} };
    

    arma::mat M = cos(w)*arma::eye(4,4) + W_mat*sinwow;
    
    x.subvec(0,3) = (cos(w)*arma::eye(4,4) + W_mat*sinwow)*x.subvec(0,3); // q_(k+1) = M*q_(k)
    x.subvec(0,3) = x.subvec(0,3)/arma::norm(x.subvec(0,3)); // normalize quaternion
    

    x.subvec(4,6) = (arma::eye(3,3)-arma::eye(3,3)*kr*delta_t)*x.subvec(4,6);  // w_(k+1) = (I - I*kr*delta_t)*w_(k)

    // position equations
    x.subvec(7,9) = x.subvec(7,9) + x.subvec(10,12)*delta_t;

    x.subvec(10,12) = (arma::eye(3,3)-arma::eye(3,3)*kt*delta_t)*x.subvec(10,12);  // v_(k+1) = (I - I*kt*delta_t)*v_(k)

    //x.subvec(10,12) = x.subvec(10,12);
    //------------------------------------
    // Update system covariance matrix P
    
    double JFx[169];
    double JFu[78];
    double k[2] = {kr,kt};    
    
    std::vector<double> x_t;

    x_t = arma::conv_to<vec_t>::from(x.subvec(0,12)); // convert from armadillo vector to c++ vector
   
    JacSystemPredictionV3b(&x_t[0], delta_t,k, JFx, JFu); // get Jacobians

    arma::mat Jfx(JFx,13,13); // initialize armadillo matrix from array.
    arma::mat Jfu(JFu,13,6);

    arma::mat U;
    U.zeros(6,6);

    U(arma::span(0,2),arma::span(0,2)) = arma::eye(3,3)*pow(sigma_w*delta_t,2);
    U(arma::span(3,5),arma::span(3,5)) = arma::eye(3,3)*pow(sigma_a*delta_t,2);
    
    int x_len = x.size();
    if(x_len > 13)
    {   
      //  arma::mat tp = P(arma::span(0,12),arma::span(13,x_len));
      //  arma::mat tj = Jfx*P(arma::span(0,12),arma::span(13,x_len));
        P(arma::span(0,12),arma::span(13,x_len-1)) = Jfx*P(arma::span(0,12),arma::span(13,x_len-1)); 
        P(arma::span(13,x_len-1),arma::span(0,12)) = P(arma::span(0,12),arma::span(13,x_len-1)).t();
        P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + Jfu*U*Jfu.t();
    }
    else
    {
        P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + Jfu*U*Jfu.t();
    }
    
    

    //-- test data
    /*
        arma::vec xout;
        xout.load("x_out.txt");
        arma::mat Pout;
        Pout.load("P_out.txt");
        //xout.subvec(0,12).print();
        //cout << "xxxx" << endl;
        //x.print();
        double dif = arma::norm(P-Pout, "inf");
    */
    //-----

    

}

