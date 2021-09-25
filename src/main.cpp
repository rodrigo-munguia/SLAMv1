
/* for debug/release (without debugging)
make clean
make debug/release
*/

#include <iostream>
#include "opencv2/opencv.hpp"
#include <armadillo>
#include <cmath>
#include <thread>
#include <chrono>
#include "parameters.h"
#include "getData.h"
#include "map/map.h"
#include "ekf/ekf.h"
#include "matplotlib/matplotlibcpp.h"
#include "locks.h"
#include "loop/loop.h"
#include "plot.h"

using namespace std;
using namespace cv;
using namespace arma;
namespace plt = matplotlibcpp;

//void mapping(GMAP &gmap,parameters &par,bool &ekf_run,LOCKS &locks);
void mapping(GMAP &gmap,parameters &par,EKF &ekf,LOCKS &locks);
//void ekf_slam(EKF &ekf,GMAP &gmap,LOOP &cloop,parameters &par, bool &ekf_run, double &run_time,LOCKS &locks);
void ekf_slam(EKF &ekf,GMAP &gmap,LOOP &cloop,parameters &par, double &run_time,LOCKS &locks);
//void loop(LOOP &cloop, GMAP &gmap,parameters &par,bool &ekf_run,LOCKS &locks);
void loop(LOOP &cloop, GMAP &gmap,parameters &par,EKF &ekf,LOCKS &locks);

void plot_f(PLOT &plot,EKF &ekf,GMAP &gmap,parameters &par,LOCKS &locks);

// Environment variables ----------------
// set openplas to run in a single thread
//export OPENBLAS_NUM_THREADS=1
// Avoid OpenCL initialization     
//export OPENCV_OPENCL_DEVICE=disabled  

int main() {
    

    cv::setNumThreads(1);

    parameters par;
    LOCKS locks;

    
    
    
    par = get_parameters(); // get parameters structure
    
    EKF ekf(par); // create EKF-SLAM object     
    //bool ekf_run = true;
    ekf.run = true;

    double run_time = par.init.run_time; 

    GMAP gmap(par); // create Global MAP object

    LOOP cloop(par); // create Closing loop opbject

    PLOT plot(par); // Plot object

    // init EKF-SLAM thread
    //thread th1(ekf_slam,std::ref(ekf),std::ref(gmap),std::ref(cloop),std::ref(par),std::ref(ekf_run),std::ref(run_time),std::ref(locks));
    thread th1(ekf_slam,std::ref(ekf),std::ref(gmap),std::ref(cloop),std::ref(par),std::ref(run_time),std::ref(locks));

    // init Mapping thread
    //thread th2(mapping,std::ref(gmap),std::ref(par),std::ref(ekf_run),std::ref(locks));
    thread th2(mapping,std::ref(gmap),std::ref(par),std::ref(ekf),std::ref(locks));

    // init Closing-loop thread
    //thread th3(loop,std::ref(cloop),std::ref(gmap),std::ref(par),std::ref(ekf_run),std::ref(locks));
    thread th3(loop,std::ref(cloop),std::ref(gmap),std::ref(par),std::ref(ekf),std::ref(locks));
    
     // init Plot thread     
    thread th4(plot_f,std::ref(plot),std::ref(ekf),std::ref(gmap),std::ref(par),std::ref(locks));
       
    
    //arma::arma_version ver;
    // std::cout << "ARMA version: "<< ver.as_string() << std::endl;
    
    
    int option;
    string Soption = "";
    do //do-while loop starts here.that display menu again and again until user select to exit program
    {
        
        cout << "1) Stop EKF-SLAM " << endl;    
        cout << "2) Display results " << endl;  
        cout << "Ctrl + c) Exit Program " << endl;
        getline(cin, Soption);
        option = std::atoi(Soption.c_str());

        if(option == 1) // Checking if user selected option 1
        {   
            locks.ekf_run_mtx.lock();
                //ekf_run = false;
                ekf.run = false;
            locks.ekf_run_mtx.unlock();
            
        }
        if(option == 2)
        {
            //if(ekf_run == true)
            if(ekf.run == true)
            {
               cout << "EKF-SLAM still running, waiting for completion.." << endl; 
            }
            option = 99;
        }


    } 
    while((option != 99)&&(ekf.run == true));  //condition of do-while loop   
    //while((option != 99)&&(ekf_run == true));  //condition of do-while loop


    // Wait for thread t1 to finish     
    th1.join();
    th2.join();
    th3.join();

    sleep(4);

    std::vector<double> x_K, y_K, z_K;
    //-------------------------
    for(int i = 0; i< gmap.KeyFDATA.size();i++)
    {
        x_K.push_back(gmap.KeyFDATA[i].CameraPos(0));
        y_K.push_back(-gmap.KeyFDATA[i].CameraPos(1));
        z_K.push_back(-gmap.KeyFDATA[i].CameraPos(2));

    }

    
    // display results
    ekf.x.subvec(7,9).print();
     std::vector<double> x_i, y_i, z_i,x_im, y_im, z_im;
    //-------------------------
    
    for(int i=0;i< gmap.AnchorsDATA.size();i++)
    {     
        if (gmap.AnchorsDATA[i].init_type == 0)
        {
          x_i.push_back(gmap.AnchorsDATA[i].AnchorState(0));
          y_i.push_back(-gmap.AnchorsDATA[i].AnchorState(1));
          z_i.push_back(-gmap.AnchorsDATA[i].AnchorState(2));
        }
        else
        {
           x_im.push_back(gmap.AnchorsDATA[i].AnchorState(0));
          y_im.push_back(-gmap.AnchorsDATA[i].AnchorState(1));
          z_im.push_back(-gmap.AnchorsDATA[i].AnchorState(2));
        }
        
    }

    //---------------------------------------------------
    plt::figure(1);   
        // plot last camera position
        double c_x = ekf.x_c[ekf.x_c.size()-1];
        double c_y = ekf.y_c[ekf.y_c.size()-1];
        std::vector<double> cam_x,cam_y;
        cam_x.push_back(c_x);
        cam_y.push_back(c_y);
        plt::plot(cam_x,cam_y,"bx");
    
        //std::string par1 = "markersize=1";  
        //plt::plot(x_im,y_im, {{"color", "g"}, {"marker", "."},{"markersize","1" }});  
        plt::plot(x_im,y_im,"g.");  
        plt::plot(x_i,y_i,"y.");  
        plt::plot(x_K,y_K,"r.");   //
        plt::plot(ekf.x_c, ekf.y_c); 
        plt::plot(ekf.store.x_gps,ekf.store.y_gps,"k-");  // plot gps
        plt::axis("equal"); 
    
     plt::figure(3);
        plt::scatter(x_im,y_im,1,{{"color", "g"}});
        plt::scatter(x_i,y_i,1,{{"color", "y"}});
        plt::scatter(x_K,y_K,4,{{"color", "r"}});
        plt::plot(x_K,y_K,"b-");
        //plt::scatter(ekf.x_c, ekf.y_c,4,{{"color", "b"}});
        plt::axis("equal"); 
    

    //----------------------------
    plt::figure(2); 
     
    plt::plot(y_im,z_im,"g."); 
    plt::plot(y_i,z_i,"y.");
    plt::plot(y_K,z_K,"r.");  
    plt::plot(ekf.y_c, ekf.z_c);
    plt::axis("equal");

     //plt::figure(3);
    
    
    //plt::axis("equal");

    plt::show();

    double max_v = gmap.Vgraph.max()/2;
    

    cv::Mat vg(gmap.Vgraph.n_rows, gmap.Vgraph.n_cols, CV_8UC1, Scalar(255));

    for (int i =0 ; i < gmap.Vgraph.n_rows; i++)
    {
      for (int j =0 ; j < gmap.Vgraph.n_cols; j++)
        {  
           if (gmap.Vgraph(i,j) != 0) 
             {
                   // vg.at<uchar>(i,j) =  (-255/max_v)*gmap.Vgraph(i,j) + 255; 
                   vg.at<uchar>(i,j) = 0;
             }          
        }
    }

    imshow("Display window", vg);
    waitKey(0); // Wait for a keystroke in the window
    imwrite("test.png" ,vg);
    cout << gmap.Vgraph.n_cols << endl;

     int q = 10;
    
     return 0;
}

//-------------------------------------------------------------------------
// Function thread for loop closing
//-------------------------------------------------------------------------
//void loop(LOOP &cloop,GMAP &gmap,parameters &par,bool &ekf_run,LOCKS &locks)
void loop(LOOP &cloop, GMAP &gmap,parameters &par,EKF &ekf,LOCKS &locks)
{
    while(ekf.run == true)
    {
       // gmap.update(locks);
       if(cloop.newFrame == true && par.sys.closing_loop_active == true)
       {
        cloop.update(gmap,locks);           
       }     
    }

}


//-------------------------------------------------------------------------
// Function thread for Mapping using optimization techniques
//-------------------------------------------------------------------------
//void mapping(GMAP &gmap,parameters &par,bool &ekf_run,LOCKS &locks)
void mapping(GMAP &gmap,parameters &par,EKF &ekf,LOCKS &locks)
{
    while(ekf.run == true)
    {
        if(gmap.closing_loop_active == false )  
        { 
            gmap.update(locks);
        }


    }

}



//-------------------------------------------------------------------------
// Function thread for EKF-SLAM
//-------------------------------------------------------------------------

//void ekf_slam(EKF &ekf,GMAP &gmap,LOOP &cloop,parameters &par, bool &ekf_run, double &run_time,LOCKS &locks)
void ekf_slam(EKF &ekf,GMAP &gmap,LOOP &cloop,parameters &par, double &run_time,LOCKS &locks)
{

    cout << "EKF-SLAM thread running... " << endl;   

    DATA dat;
   

    ekf.state_init(); // initialize system state
   
    
    // Start measuring time
    auto begin = std::chrono::high_resolution_clock::now();
    
    double last_time;
    bool init = false;
    double delta_t;
    double total_time = 0;

    
        
    //for(int i = 0;i < 2200;i++)
    //while((ekf_run == true)&&(total_time < run_time))
    while((ekf.run == true)&&(total_time < run_time))
    {
        if (par.init.DataSetType == 1) dat = getData(par);  // get data from QUAD dataset
        if (par.init.DataSetType == 2) dat = getDataB(par);  // get data from QUAD dataset
                
        if (dat.data_type == "frame" )
        {
            // compute delta time-----
            if (init == false)
            {
                delta_t = par.ekf.delta_t;
                last_time = dat.frame.time;
                init = true;
            }
            else
            {
                delta_t = (dat.frame.time - last_time)/1000000;
                last_time = dat.frame.time;
            }
            total_time = total_time + delta_t;
            //------------------------
                        
            ekf.prediction(delta_t); // EKF prediction        

            ekf.visual_update(&dat.frame,gmap,cloop,locks); // EKF visual update
            
            ekf.store_data_for_plot(locks,&dat.frame);
            
            ekf.store_info();
            
            //std::printf("Execution time: %.3f seconds.\n", total_time);
            //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
            //imshow( "Display window", dat.frame.image );                   // Show our image inside it.
            //waitKey(0);      
        }
        if (dat.data_type == "bar" )
        {
           ekf.altitude_update_bar(dat.bar);                            
        }
        if (dat.data_type == "alt")
        {           
           ekf.altitude_update_alt(dat.alt);  
        }  

        if (cloop.new_xy_position_available == true )
        {
            arma::vec::fixed<2> xy_pos = cloop.get_new_xy_pos();
            ekf.closing_loop_position_update(xy_pos);          

        }
        if (dat.data_type == "gps")
        {
            ekf.store_gps(dat.gps);
        }
        if (dat.data_type == "att")
        {
          //cout << "roll: " << dat.att.roll << " pitch: " << dat.att.pitch << " yaw:" << dat.att.yaw << endl;
           if(par.sys.EKF_attitude_update== true)
                ekf.attitude_update(dat.att);
        }    

       
    }

     // Stop measuring time and calculate the elapsed time
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    
    cout << "EKF-SLAM thread ended! " << endl;   
    std::printf("Total execution time: %.3f seconds.\n", elapsed.count() * 1e-9);
    std::printf("Total real time: %.3f seconds.\n", total_time);
    
    
    locks.ekf_run_mtx.lock();
        //ekf_run = false;
        ekf.run = false;
    locks.ekf_run_mtx.unlock();

}

void plot_f(PLOT &plot, EKF &ekf,GMAP &gmap,parameters &par,LOCKS &locks)
{

   cout << "Plot thread running... " << endl;

   plot.init(ekf,gmap,locks);

  


}
 