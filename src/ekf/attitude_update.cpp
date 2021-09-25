#include "attitude_update.h"
#include <cmath>


struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}




void Attitude_Update(arma::vec& x, arma::mat& P,ATT &att,parameters &par)
{
  
  static bool f_initialized = false;
   static double yaw_at_home;

   
   double measured_yaw = att.yaw;

    
   
   if(f_initialized == false)
    {
        yaw_at_home = att.yaw;
        f_initialized = true;
    }
    if(f_initialized == true)
    {
        double p_ini = par.sys.camera_init_pitch_angle;
            arma::vec::fixed<3> z;  // measurement
                // account for Bebop to camera coordinate transformation
                z(0) = (att.pitch + p_ini ); 
                z(1) = -att.roll;
                z(2) = att.yaw - yaw_at_home;
                
                z(2) = 0;

            Quaternion q;
            q.w = x(0);
            q.x = x(1);
            q.y = x(2);
            q.z = x(3);

            EulerAngles e;

            e = ToEulerAngles(q);
            
            arma::vec::fixed<3> h;  // measurement prediction
            h(0) = e.roll;
            h(1) = e.pitch;
            h(2) = e.yaw;

            //--------------- Measurement Jacobian
                int x_len = x.size();
                arma::mat H;
                // Form jacobian
                H.resize(3,x_len);
                arma::mat::fixed<3,4> de_dq; 
            double q0 = q.w;
            double q1 = q.x;
            double q2 = q.y;
            double q3 = q.z;
            
            double t1 = (2*q1*q1 + 2*q2*q2 - 1);
            double t2 = (2*q0*q1 + 2*q2*q3);
            double t3 = pow((1 - pow((2*q0*q2 - 2*q1*q3),2)),.5);
            double t4 = (2*q2*q2 + 2*q3*q3 - 1);
            double t5 =  (2*q0*q3 + 2*q1*q2);

            
            de_dq = {{-(2*q1*t1)/(t1*t1 + t2*t2), -(((2*q0)/t1 - (4*q1*t2)/t1*t1)*t1*t1)/(t1*t1 + t2*t2), -(((2*q3)/t1 - (4*q2*t2)/t1*t1)*t1*t1)/(t1*t1 + t2*t2), -(2*q2*t1)/(t1*t1 + t2*t2) },
                    {(2*q2)/t3, -(2*q3)/t3, (2*q0)/t3, -(2*q1)/t3 }, 
                    { -(2*q3*t4)/(t4*t4 + t5*t5), -(2*q2*t4)/(t4*t4 + t5*t5), -(((2*q1)/t4 - (4*q2*t5)/t4*t4)*t4*t4)/(t4*t4 + t5*t5), -(((2*q0)/t4 - (4*q3*t5)/t4*t4)*t4*t4)/(t4*t4 + t5*t5)  }};               
            
            H(arma::span(0,2),arma::span(0,3)) = de_dq;

            //--------------------------------------------------------------------------
            
           
            
            //---- ekf update

            double vs = par.ekf.sigma_att_update*par.ekf.sigma_att_update;  // variance
            arma::mat::fixed<3,3> R;
            R(0,0) = vs;
            R(1,1) = vs;
            R(2,2) = vs*2;


                arma::mat H_P = H*P;

                arma::mat S = H_P*H.t() + R; // Innovation matrix
                    //arma::mat K = P*H.t()*arma::inv_sympd(S);
                arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

                P = P - K*H_P;  // System Covariance matrix update 
            
                x = x + K*(z-h);  // System vector update
            
    }

  int ww = 10; 
  

 








}