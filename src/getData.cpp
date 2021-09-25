#include "getData.h"

/*---------------------------------------------------
Rodrigo Munguía 2020.

Get data from data base
-----------------------------------------------------
*/


using namespace std;
using namespace cv;





string frame_list = "frame_list.txt";
string GPS_data = "GPS_raw.txt";
string Pressure_data = "PressureSensor.txt";
string Range_data = "RangeSensor.txt";
string Altitude_data = "Altitude.txt";
string Attitude_data = "Attitude.txt";



//-------------------------------------------------------------------------------------------------
// get data from Bebop dataset
DATA getDataB(parameters &PAR)
{

 string Dataset_dir = PAR.init.DataSetFile;
    
    static int flag_init = 0;
    static vector<FRAME> dat_F;
    static vector<GPS> dat_G;
    static vector<ALT> dat_A;
    static vector<RANGE> dat_R;
    static vector<ATT> dat_O;
    DATA dat;
    static double last_range;
    static long int last_range_time;
    static bool range_available = false;
    static bool range_available_alt = false;
    static double last_alt = 0;
    
    // load data from the text files ----------
    // Execute only one time
    if (flag_init == 0)
    {   
        ifstream data_file_F;
        ifstream data_file_G;
        ifstream data_file_A;
        ifstream data_file_R;
        ifstream data_file_O;    

        data_file_F.open(Dataset_dir + frame_list); // frames file
        data_file_G.open(Dataset_dir + GPS_data); // GPS file
        data_file_A.open(Dataset_dir + Altitude_data); // Altimeter file
        data_file_R.open(Dataset_dir + Range_data); // Range file
        data_file_O.open(Dataset_dir + Attitude_data);

        string lineF;
        vector<string> v;
        while ( getline(data_file_F,lineF) )
        {  
            //stringstream ss(lineF);
            FRAME frame;
           // frame.time = stoi(lineF.substr(0,10));
            frame.time = stol(lineF.substr(0,10), nullptr, 10);
            frame.image_file = lineF;
            //int q = 10;
            dat_F.push_back(frame);
        }
        
        string lineG;
        
        while ( getline(data_file_G,lineG) ) // read one line of the file each time
        {   
            stringstream ss(lineG); 
            GPS gps;
            vector<string> v_G;
            while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ',' );
                v_G.push_back( substr );
            }
           // gps.time = stoi(v_G[0]);
            gps.time = stol(v_G[0], nullptr, 10);
            gps.lat = stod(v_G[1]);
            gps.lon = stod(v_G[2]);
            gps.alt = stod(v_G[3]);
            gps.sat = stoi(v_G[6]);

            dat_G.push_back(gps);    
            
        }
        string lineO;
        while ( getline(data_file_O,lineO) )
        {
            stringstream ss(lineO); 
            ATT att;
            vector<string> v_O;
            while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ',' );
                v_O.push_back( substr );
            }
            //bar.time = stoi(v_P[0]);
            att.time = stol(v_O[0], nullptr, 10);
            att.roll = stod(v_O[1]);
            att.pitch = stod(v_O[2]);
            att.yaw = stod(v_O[3]);            
            dat_O.push_back(att);             
        }
        
        
        string lineA;
        while ( getline(data_file_A,lineA) )
        {
            stringstream ss(lineA); 
            ALT alt;
            vector<string> v_A;
            while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ',' );
                v_A.push_back( substr );
            }
            //bar.time = stoi(v_P[0]);
            alt.time = stol(v_A[0], nullptr, 10);
            alt.altitude = stod(v_A[1]);
            dat_A.push_back(alt); 
            
        }
        string lineR;
        
        while ( getline(data_file_R,lineR) )
        {
            stringstream ss(lineR); 
            RANGE range;
            vector<string> v_R;
            while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ',' );
                v_R.push_back( substr );
            }
            //range.time = stoi(v_R[0]);
            range.time = stol(v_R[0], nullptr, 10);
            range.range = stod(v_R[1]);
            range.volt = stod(v_R[2]);
            
          

            dat_R.push_back(range);
        }
        
        
        flag_init = 1;
        data_file_F.close();
        data_file_G.close();
        data_file_A.close();
        data_file_R.close(); 
        data_file_O.close(); 
        
    }
    //----------------------------------------------   
    int idx;

    if (dat_R.size() > 0)
    {
        long int times[5] = {dat_F[0].time,dat_G[0].time,dat_A[0].time,dat_O[0].time,dat_R[0].time,};
         idx = distance(begin(times),min_element(begin(times), end(times)));
    }         
    else
    {
        long int times[4] = {dat_F[0].time,dat_G[0].time,dat_A[0].time,dat_O[0].time};
        idx = distance(begin(times),min_element(begin(times), end(times)));
    }    
    //int tmp = *min_element(times, end(times));
    // find the index of the minimun time
   
     
    if (idx == 0)
    {  
        dat.data_type = "frame";
        dat.frame = dat_F[0];
        // read image:  it can be move up to the parse section
        cv::Mat image;
        image = imread(Dataset_dir + dat_F[0].image_file, IMREAD_GRAYSCALE  );
        dat.frame.image = image;
        if (range_available == true)
        {
            dat.frame.range = last_range; // if a range masurement has just "received" then associate it to the frame
            range_available = false;  
        }
        //if (range_available_alt == true)
        else
        {   
            
            long int dt = dat.frame.time - last_range_time;  // us

            if (dt > 250000 && last_alt != 0) // if there is not range measurement for a while, try to use alt measurements
            {
                dat.frame.range = last_alt;  
                
                //range_available_alt = false; 
                range_available = false;               
            }
            else 
            { 
                dat.frame.range = -1;
            }
        }        
        dat_F.erase(dat_F.begin());
    }
    else if(idx == 1)
    {   
       dat.data_type = "gps";
       dat.gps = dat_G[0];
       dat_G.erase(dat_G.begin());
    }
    else if(idx == 2)
    {   
        dat.data_type = "alt";
        dat.alt = dat_A[0];
        last_alt = dat.alt.altitude;
        dat_A.erase(dat_A.begin());
        range_available_alt = true;
         range_available = true;
    }
    else if(idx == 3)
    {   
        dat.data_type = "att";
        dat.att = dat_O[0];        
        dat_O.erase(dat_O.begin());        
    }
    else if(idx == 4)
    {  
       dat.data_type = "range";
       dat.range = dat_R[0]; 
       dat_R.erase(dat_R.begin());
       if (dat.range.range != 0)
       {
            last_range = dat.range.range;
            last_range_time = dat.range.time;
            range_available = true;
       }
    }
    else
    {
        
    }
       
    return dat;
}



//-------------------------------------------------------------------------------------------------
// get data from QUAD dataset
DATA getData(parameters &PAR)
{   
     
    string Dataset_dir = PAR.init.DataSetFile;
    
    static int flag_init = 0;
    static vector<FRAME> dat_F;
    static vector<GPS> dat_G;
    static vector<BAR> dat_P;
    static vector<RANGE> dat_R;
    DATA dat;
    static double last_range;
    static bool range_available;
    
    // load data from the text files ----------
    // Execute only one time
    if (flag_init == 0)
    {   
        ifstream data_file_F;
        ifstream data_file_G;
        ifstream data_file_P;
        ifstream data_file_R;    

        data_file_F.open(Dataset_dir + frame_list); // frames file
        data_file_G.open(Dataset_dir + GPS_data); // GPS file
        data_file_P.open(Dataset_dir + Pressure_data); // Barometer file
        data_file_R.open(Dataset_dir + Range_data); // Range file

        string lineF;
        vector<string> v;
        while ( getline(data_file_F,lineF) )
        {  
            //stringstream ss(lineF);
            FRAME frame;
           // frame.time = stoi(lineF.substr(0,10));
            frame.time = stol(lineF.substr(0,10), nullptr, 10);
            frame.image_file = lineF;
            //int q = 10;
            dat_F.push_back(frame);
        }
        
        string lineG;
        
        while ( getline(data_file_G,lineG) ) // read one line of the file each time
        {   
            stringstream ss(lineG); 
            GPS gps;
            vector<string> v_G;
            while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ',' );
                v_G.push_back( substr );
            }
           // gps.time = stoi(v_G[0]);
            gps.time = stol(v_G[0], nullptr, 10);
            gps.lat = stod(v_G[1]);
            gps.lon = stod(v_G[2]);
            gps.alt = stod(v_G[3]);
            gps.sat = stoi(v_G[6]);

            dat_G.push_back(gps);    
            
        }
        
         string lineP;
        while ( getline(data_file_P,lineP) )
        {
            stringstream ss(lineP); 
            BAR bar;
            vector<string> v_P;
            while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ',' );
                v_P.push_back( substr );
            }
            //bar.time = stoi(v_P[0]);
            bar.time = stol(v_P[0], nullptr, 10);
            bar.pressure = stod(v_P[1]);
            bar.temp = stod(v_P[2]);

            dat_P.push_back(bar); 
            
        }
        string lineR;
        
        while ( getline(data_file_R,lineR) )
        {
            stringstream ss(lineR); 
            RANGE range;
            vector<string> v_R;
            while( ss.good() ) // parse each comma-separated element into a vector of strings
            {
                string substr;
                getline( ss, substr, ',' );
                v_R.push_back( substr );
            }
            //range.time = stoi(v_R[0]);
            range.time = stol(v_R[0], nullptr, 10);
            range.range = stod(v_R[1]);
            range.volt = stod(v_R[2]);
            
          

            dat_R.push_back(range);
        }
        
        
        flag_init = 1;
        data_file_F.close();
        data_file_G.close();
        data_file_P.close();
        data_file_R.close();  
        
    }
    //----------------------------------------------
    
    

    long int times[4] = {dat_F[0].time,dat_G[0].time,dat_P[0].time,dat_R[0].time};
    
    //int tmp = *min_element(times, end(times));
    // find the index of the minimun time
    int idx = distance(begin(times),min_element(begin(times), end(times)));
     
    if (idx == 0)
    {  
        dat.data_type = "frame";
        dat.frame = dat_F[0];
        // read image:  it can be move up to the parse section
        cv::Mat image;
        image = imread(Dataset_dir + dat_F[0].image_file, IMREAD_GRAYSCALE  );
        dat.frame.image = image;
        if (range_available == true)
        {
            dat.frame.range = last_range; // if a range masurement has just "received" then associate it to the frame
            range_available = false;  
        }
        else
        {
            dat.frame.range = -1;
        }        
        dat_F.erase(dat_F.begin());
    }
    else if(idx == 1)
    {   
       dat.data_type = "gps";
       dat.gps = dat_G[0];
       dat_G.erase(dat_G.begin());
    }
    else if(idx == 2)
    {   
        dat.data_type = "bar";
        dat.bar = dat_P[0];
        dat_P.erase(dat_P.begin());
    }
    else if(idx == 3)
    {  
       dat.data_type = "range";
       dat.range = dat_R[0]; 
       dat_R.erase(dat_R.begin());
       last_range = dat.range.range;
       range_available = true;
    }
    else
    {
        
    }
    
    return dat;
}

