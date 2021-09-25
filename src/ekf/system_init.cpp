
#include "system_init.h"


//-----------------------------------------------------------------------------
//   State vector and Covariance matrix initialization
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
using namespace arma;

/*
% Initial States ***************************************************
% x meaning
% index  0  1  2  3  4   5   6   7  8  9  10  11  12  
%       q1 q2 q3 q4 w_x w_y w_z  x  y  z  v_x v_y v_z 
% Attitude states
% x(1:4)=   [q1 q2 q3 q4] -> quaternion orientation  (navigation to camera)
% x(5:7)=   [w_x w_y w_z ] ->  vel rotation in the body frame
% Position states
% x(8:10)= [x  y  z]  ->  Position in the navigation coordinate frame
% (navigation to camera)
% x(11:13)= [v_x v_y v_z]  -> Velocity in navigation coordinate frame.
*/

void system_init(arma::vec& x, arma::mat& P, parameters &par)
{

double roll = par.init.roll_init;
double pitch = par.init.pitch_init;
double yaw = par.init.yaw_init;
double x_init = par.init.x_init;
double y_init = par.init.y_init;
double z_init = par.init.z_init;
double Ra2b[9];

Euler_to_Ra2b(roll, pitch, yaw, Ra2b);

double q_int[4];

Ra2b_TO_Quat_a2b(Ra2b,q_int);

x(0) = q_int[0];
x(1) = q_int[1];
x(2) = q_int[2];
x(3) = q_int[3];
x(4) = numeric_limits<double>::epsilon();
x(5) = numeric_limits<double>::epsilon();
x(6) = numeric_limits<double>::epsilon();
x(7) = x_init;
x(8) = y_init;
x(9) = z_init;
x(10) = numeric_limits<double>::epsilon();
x(11) = numeric_limits<double>::epsilon();
x(12) = numeric_limits<double>::epsilon();

P = eye(13,13)*numeric_limits<double>::epsilon();




}