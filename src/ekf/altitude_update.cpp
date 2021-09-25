#include "altitude_update.h"
#include "math.h"



void Altitude_Update_alt(arma::vec& x, arma::mat& P,ALT &alt,parameters &par)
{
   static bool f_initialized = false;
   static double alt_at_home;
   
   //double measured_alt = -alt.altitude;
   double measured_alt = alt.altitude;  // only for paper

   if(f_initialized == false)
    {
        alt_at_home = measured_alt;
        f_initialized = true;
    }
    if(f_initialized == true)
    {
        
        double z =  measured_alt -  alt_at_home;     

        int x_len = x.size();
        arma::mat H;
        // Form jacobian
        H.resize(1,x_len);
        H(0,9) = 1; 
        
        double h = x(9);
        double r = par.ekf.sigma_h;

        arma::mat H_P = H*P;

        arma::mat S = H_P*H.t() + r*r; // Innovation matrix
        //arma::mat K = P*H.t()*arma::inv_sympd(S);
       arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

        P = P - K*H_P;  // System Covariance matrix update 
 
        x = x + K*(z-h);  // System vector update

        int q = 10;

    }    


}


void Altitude_Update_bar(arma::vec& x, arma::mat& P,BAR &bar,parameters &par)
{
    
    static bool f_initialized = false;
    static double  P_f;  // N/m^2 barometric pressure at home position

    double M = par.sensors.barometer_sensor.M; // kg/mol standard molar mass of atmospheric air
    double R = par.sensors.barometer_sensor.R; // N-m/mol-K universal gas constant for air
    double a = par.sensors.barometer_sensor.a; // K/m
    double g = par.sensors.barometer_sensor.g; // constant gravity
    double T0 = par.sensors.barometer_sensor.T0; //  Kelvin standard temperature at sea level
    double P0 = par.sensors.barometer_sensor.P0; // N/m^2 standard pressure at sea level
    double L0 = par.sensors.barometer_sensor.L0; // K/m lapse rate of temperature deacrese in lower atmosphere
    //---------------------------------------------
    double la = par.sensors.barometer_sensor.la; // altitude over sea level (zapopan)
    double lt = par.sensors.barometer_sensor.lt; // Temperature at flight location (celcius)
    
    if(f_initialized == false)
    {
        P_f = bar.pressure;
        f_initialized = true;
    }
    
    if(f_initialized == true)
    {
        double T = lt + 273.15; // Temperature at flight location (kelvin)   
        // Equation 1 (in function of initial read pressure )
 
        double ee = (R*L0)/(M*g);
        double z = (1 - pow((bar.pressure/P_f),ee) )*(T/a); // relative altitute
        

        int x_len = x.size();
        arma::mat H;
        // Form jacobian
        H.resize(1,x_len);
        H(0,9) = 1; 
        
        double h = x(9);
        double r = par.ekf.sigma_h;

        arma::mat H_P = H*P;

        arma::mat S = H_P*H.t() + r*r; // Innovation matrix
        //arma::mat K = P*H.t()*arma::inv_sympd(S);
       arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

        P = P - K*H_P;  // System Covariance matrix update 
 
        x = x + K*(z-h);  // System vector update

        int q = 10;

    }    

}