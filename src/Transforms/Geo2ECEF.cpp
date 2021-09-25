#include "Geo2ECEF.h"
#include <stdio.h>      /* printf */
#include <math.h>       /* round, floor, ceil, trunc */
// Function to convert Geodetic to ECEF coordinates

void Geo2ECEF(double lat_d, double lng_d, double alt_m, double &Gx, double &Gy, double &Gz )
{

    const double aa = 6378137.0;
    const double bb = 6356752.314; 
    const double scl = 10e10;    // mm position accuracy reqs 10 decimal accuracy in radian lat and lng

    double lat_r = round(scl*lat_d*M_PI/180)/scl;
    double lng_r = round(scl*lng_d*M_PI/180)/scl;

    double ff = (aa-bb)/aa;
    double ee = sqrt((2-ff)*ff);

    double R_M = (aa*(1-ee*ee))/pow((1-ee*ee*sin(lat_r)*sin(lat_r)),3/2);
    double R_N = aa/sqrt(1-ee*ee*sin(lat_r)*sin(lat_r)); 
    
    //double lat_r = round(scl*lat_d*M_PI/180)/scl;
    //double lng_r = round(scl*lng_d*M_PI/180)/scl;

    Gx = (R_N + alt_m)*cos(lat_r)*cos(lng_r);
    Gy = (R_N + alt_m)*cos(lat_r)*sin(lng_r);
    Gz = (R_N*(1-ee*ee)+alt_m)*sin(lat_r);
    
    // round to nearest millimeter
    Gx = round(Gx*1000)/1000;
    Gy = round(Gy*1000)/1000;
    Gz = round(Gz*1000)/1000;


}



