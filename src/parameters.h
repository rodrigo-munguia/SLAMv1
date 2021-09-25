/*---------------------------------------------------
Rodrigo Munguía 2020.

System parameters
-----------------------------------------------------
*/
#include <string>
#include "opencv2/opencv.hpp"

#ifndef PARAMETERS_H
#define PARAMETERS_H

using namespace std;


struct SYSTEM
{
  bool EKF_initialize_anchors;
  bool EKF_use_anchors_from_GMAP;
  bool EKF_attitude_update;
  bool GMAP_use_anchors_from_EKF;
  bool GMAP_update_optimized_anchors;
  bool GMAP_optimize_Keyframe_pos;
  bool GMAP_update_optimized_Keyframe_pos;
  bool closing_loop_active;
  double camera_init_pitch_angle;
}; 

struct EKF_par
{
    double delta_t;
    double sigma_a; // traslational aceel noise 
    double sigma_w; // rotational accel noise
    double Tau_r; //  decaying time constant for rotational motion
    double Tau_p;  // decaying time cosntant for translational motion
    double sigma_uv; // (pixels) camera measurement noise standard deviation       
    bool visual_attitude_update; // visual measurements "affect" quaternion orientation  
    float Uncertainty_depth_anchor_init;  // threshold for converting a feat to anchor
    double sigma_h; // altimeter measurement noise
    double sigma_cl; // closing loop measurement noise
    double AntGPSoffset[3]; //  GPS antena to origin of robot coordinate frame, Must be expresed in NED coordiantes
    double sigma_id_range_dem;  // for features initialization, measurment noise with range data
    double sigma_id_WOrange_dem; // for features initialization, measurement nois without range datas

    double sigma_att_update; // for attitude updates

};

struct INIT
{
    int x_len;
    double roll_init;
    double pitch_init;
    double yaw_init;
    double x_init;
    double y_init;
    double z_init;
    string DataSetFile;
    int DataSetType;
    double run_time;
    double GPS_init_yaw;
    

};

struct CAM
{
  double distortions[5];
  double cc[2];
  double fc[2];
  double alpha_c;  
};

struct IMG
{
  int image_rows;
  int image_cols;
  int n_sub_imgs; // times the image is divided to search new features, must be a power of 2, eg: 1,2,4,8
  CAM cam_parameters;
  int search_margin; // size (in pixels) of the margin of the image exluded to match visual features
  bool check_innovation_for_mathing; // 
  double max_innov_pixels; // if no stochastic innovation is checked then use this constant value
  int minimun_distance_new_points ;
};


struct RANGE_sensor
{
  double max_range_pattern;
  double r_max;
  double Range_offset;
  double max_range_operation;
};

struct BAROMETER_sensor
{
  double la; // local altitude (at flight location)
  double lt; // local temperature (at flight location)
  double T0; //  Kelvin standard temperature at sea level
  double P0; // N/m^2 standard pressure at sea level
  double L0; //  K/m lapse rate of temperature deacrese in lower atmosphere
  double M; //  kg/mol standard molar mass of atmospheric air
  double R; // % N-m/mol-K universal gas constant for air
  double a; // % K/m
  double g; // gravity

};




struct SENSORS
{
  RANGE_sensor range_sensor;
  BAROMETER_sensor barometer_sensor;
};

struct MAPPING
{ 
  // for Update the visibility graph
  int n_consec_kf_wo_link;  //  If there is not visual link for n consecutive Keyframes, then stop searching
  int c_consec_anchor_wo_link; //  // if n Anchors have not been visualy linked with the last Key frame, then stop seaching
  // for Matching the visual features
  int n_consec_kf_wo_link_match; 
  int min_kf_matched_for_keep_anchor;

};

struct CLOSE_LOOP
{
  double std_ab_vo; // standard deviation visual odometry observation
  double std_ab_cl; // standard deviation close-loop observation
  int min_matches_for_potential_loop;
  int min_inliers_for_potential_loop;
  int min_KeyFrames_with_potential_loop;
  int min_matches_for_computing_pos;
  int min_time_since_last_close; 

};

// 3d plot
struct PLOT_3D
{
 int viewer_width;
 int viewer_height;
 int default_init_axis_view;
 
 bool show_grid_xy;
 cv::Rect grid_rect_xy; 
 int grid_xy_z;
 
 bool show_grid_yz;
 cv::Rect grid_rect_yz; 
 int grid_yz_x;

 bool show_frame;
 bool show_efk_feats;
 bool show_ekf_anchors;
 bool show_ekf_trajectory;
 bool show_global_map;
 bool show_camera_pose;
 bool show_key_frames;

};



// main structure of parameters
struct parameters
{
    EKF_par ekf;
    INIT init;
    IMG img;
    SENSORS sensors;
    MAPPING map;
    SYSTEM sys;
    CLOSE_LOOP close_loop;
    PLOT_3D plot_3D;
};







//------------------------------------------------------------------------
// set parameters 

void QUADdataset(parameters &PAR);
void BEBOPdataset(parameters &PAR);

parameters get_parameters();



#endif /*PARAMETERS_H*/