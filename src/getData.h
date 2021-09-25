#ifndef GET_DATA_H
#define GET_DATA_H

/*---------------------------------------------------
Rodrigo Munguía 2020.

Get data from data base
-----------------------------------------------------
*/
#include <string>
#include "opencv2/opencv.hpp"
#include <fstream>
#include <algorithm>
#include <sstream>
#include "parameters.h"

using namespace std;
using namespace cv;



// gps data
struct GPS
{
   long int time;
   double lat;
   double lon;
   double alt;
   int sat;     
};

// barometer (raw) data
struct BAR 
{
    long int time;
    double pressure;
    double temp;
};
// Altitude  data
struct ALT
{
    long int time;
    double altitude; //  (meters)
};


// Range (sonar) data
struct RANGE
{
    long int time;
    double range;
    double volt;
};

// Visual data
struct FRAME
{
    long int time;
    string image_file;
    cv::Mat image;
    double range;
};

struct ATT
{
    long int time;
    float roll; //  
    float pitch;
    float yaw;
};


// struct for storing data from sensors
struct DATA
{   
    string data_type; 
    GPS gps;
    BAR bar;
    RANGE range;
    FRAME frame;
    ALT alt;
    ATT att;
};

    

DATA getData(parameters &PAR);
DATA getDataB(parameters &PAR);

#endif

