#include "parameters.h"






parameters get_parameters()
{
    parameters PAR;
    
    PAR.init.DataSetType = 2;  // 1-> Quad DataSet  2-> Bebop2 Dataset
    
    if (PAR.init.DataSetType == 1) QUADdataset(PAR);
    if (PAR.init.DataSetType == 2) BEBOPdataset(PAR);
    


    return PAR;
}

void BEBOPdataset(parameters &PAR)
{   
    // 2021-8-4-12-45 -> 72 seconds
    // 2021-8-4-12-50 -> 94
    // 2021-8-4-12-54 -> 86.1  good
    // initial conditions   
    PAR.init.DataSetFile = "/home/rodrigo/RESEARCH/DataSets/Bebop/2021-8-4-12-54/";
    PAR.init.run_time = 86.1; // 70
    PAR.init.GPS_init_yaw = 98*(3.1416/180);  // For aligning GPS points with local coordinates 
    

    //------  
    // check camera bebop coordinate system in:
    // https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-bebop2-vs.html
    // e.g. roll (rotation over camera x_c axis) correspond to bebop pitch (rotation over y_e axis)
   
    PAR.sys.camera_init_pitch_angle = (-84 + 90)*(3.1416/180);

    //PAR.init.roll_init = PAR.sys.camera_init_pitch_angle;
    PAR.init.roll_init =  0;
    PAR.init.pitch_init = 0;    
    PAR.init.yaw_init = 0;  //     
    PAR.init.x_init = 0;
    PAR.init.y_init = 0;
    PAR.init.z_init = 0;

    
    // general system parameters
    PAR.sys.closing_loop_active = true;

    PAR.sys.EKF_initialize_anchors = true;

    PAR.sys.EKF_attitude_update = false;

    PAR.sys.EKF_use_anchors_from_GMAP = false;
    PAR.sys.GMAP_use_anchors_from_EKF = true; // (overwrite EKF_initialize_anchors )
    
    PAR.sys.GMAP_update_optimized_anchors = true;
    PAR.sys.GMAP_optimize_Keyframe_pos = false;
    PAR.sys.GMAP_update_optimized_Keyframe_pos = false; // (overwrite GMAP_optimize_Keyframe_pos )
    
    //--------------------------------------------------
    if( PAR.sys.GMAP_use_anchors_from_EKF == true) 
        PAR.sys.EKF_initialize_anchors = true;

    if(PAR.sys.GMAP_update_optimized_Keyframe_pos == true)
        PAR.sys.GMAP_optimize_Keyframe_pos = true;    

    // EKF parameters
    PAR.ekf.delta_t = 1/(double)24;
    PAR.ekf.sigma_a = 2; //2
    PAR.ekf.sigma_w = .001; //.001
    PAR.ekf.Tau_r = 1; // 1
    PAR.ekf.Tau_p = 100; // 2.5
    PAR.ekf.sigma_uv = 3; // 3 (pixels) camera measurement noise standard deviation  
    PAR.ekf.visual_attitude_update = true;
    PAR.ekf.Uncertainty_depth_anchor_init = .1;
    PAR.ekf.sigma_h = .01;  // altitude update uncertanty
    PAR.ekf.sigma_cl = .025; // closing loop uncertanty
    PAR.ekf.AntGPSoffset[0] = 0; // % GPS antena to origin of robot coordinate frame, Must be expresed in NED coordiantes
    PAR.ekf.AntGPSoffset[1] = 0;
    PAR.ekf.AntGPSoffset[2] = 0;
    PAR.ekf.sigma_id_range_dem = 20; //25  // Feats with range:  sig_d_ini = depth/par.ekf.sigma_id_range_dem
    PAR.ekf.sigma_id_WOrange_dem = 5; // 20 // Feats without range:  sig_d_ini = depth/par.ekf.sigma_id_WOrange_dem
    PAR.ekf.sigma_att_update = .01;

    // Mapping parameters
    PAR.map.n_consec_kf_wo_link = 10;
    PAR.map.c_consec_anchor_wo_link = 100;
    PAR.map.n_consec_kf_wo_link_match = 50; // deprecated!!  // If there is no visual link from the last KFrame to the i-Kframe for n consecutive Kframes break (stop the matching)
    PAR.map.min_kf_matched_for_keep_anchor = 3; // minumin number of keyframes that a anchor must be matched to be keeped in the global map

    // Image processing paramters
    PAR.img.image_rows = 240;
    PAR.img.image_cols = 320;
    PAR.img.n_sub_imgs = 1;
    PAR.img.search_margin = 15;
    
    // Bebop2 Camera        
        PAR.img.cam_parameters.distortions[0] = 0.02921;
        PAR.img.cam_parameters.distortions[1] = -0.00504; 
        PAR.img.cam_parameters.distortions[2] = 0.00297; 
        PAR.img.cam_parameters.distortions[3] = -0.00843; 
        PAR.img.cam_parameters.distortions[4] = 0.00000;   
        PAR.img.cam_parameters.cc[0] = 156.24435;
        PAR.img.cam_parameters.cc[1] = 117.04562 ;
        PAR.img.cam_parameters.fc[0] = 206.34225;
        PAR.img.cam_parameters.fc[1] = 268.65192;
        PAR.img.cam_parameters.alpha_c = 0;        

   
    PAR.img.check_innovation_for_mathing = false; 
    PAR.img.max_innov_pixels = 40; // if no stochastic innovation is checked then use this constant value
    PAR.img.minimun_distance_new_points = 20; // pixels

    // Sensors parameters
    PAR.sensors.range_sensor.max_range_pattern = 6; // max range of pattern, from sensor beam pattern
    PAR.sensors.range_sensor.r_max = 1.75; // maximun radius of elipsoid at (max range of pattern) from sensor beam pattern
    PAR.sensors.range_sensor.max_range_operation = 6; // max operation range of sensor (9m according to data sheet)
    PAR.sensors.range_sensor.Range_offset = 0; // offset for range measurements
    
    PAR.sensors.barometer_sensor.M = 0.0289644; // kg/mol standard molar mass of atmospheric air
    PAR.sensors.barometer_sensor.R = 8.31432; // N-m/mol-K universal gas constant for air
    PAR.sensors.barometer_sensor.a = .0065; // K/m
    PAR.sensors.barometer_sensor.g = 9.80665; // constant gravity
    PAR.sensors.barometer_sensor.T0 = 288.15; //  Kelvin standard temperature at sea level
    PAR.sensors.barometer_sensor.P0 =  101325; // N/m^2 standard pressure at sea level
    PAR.sensors.barometer_sensor.L0 = -0.0065; // K/m lapse rate of temperature deacrese in lower atmosphere
    //---------------------------------------------
    PAR.sensors.barometer_sensor.la =  1670 ; // altitude over sea level (zapopan)
    PAR.sensors.barometer_sensor.lt = 32.7633; // Temperature at flight location (celcius)

    // Close_loop process parameters
    PAR.close_loop.std_ab_vo = .1; // m
    PAR.close_loop.std_ab_cl = .1; // m
    PAR.close_loop.min_matches_for_potential_loop = 40; // Minumin matches before RANSAC to consider a potential loop
    PAR.close_loop.min_inliers_for_potential_loop = 10; // Minimun inliers after RANSAC to consider a potential loop
    PAR.close_loop.min_KeyFrames_with_potential_loop = 1; // Minimun number of (near) keyframes with potential loop detections to consider an actual loop
    PAR.close_loop.min_matches_for_computing_pos = 7; // Minimun number of matches for computing the updated pos in function "get_measured_position_of_current_KF"
    PAR.close_loop.min_time_since_last_close  = 3; // seconds // After a closure, the close loop process must wait "min_num_f_since_last_close" frames for try to detect the next closure


    // Plot 3D
    PAR.plot_3D.viewer_width = 1400;
    PAR.plot_3D.viewer_height = 600;
    PAR.plot_3D.default_init_axis_view = 20;
    
    PAR.plot_3D.show_grid_xy = false;
    PAR.plot_3D.grid_rect_xy.x = -20;
    PAR.plot_3D.grid_rect_xy.y = -20;
    PAR.plot_3D.grid_rect_xy.width = 40;
    PAR.plot_3D.grid_rect_xy.height = 40;    
    PAR.plot_3D.grid_xy_z = -10;

    PAR.plot_3D.show_grid_yz = true;
    PAR.plot_3D.grid_rect_yz.x = -20;
    PAR.plot_3D.grid_rect_yz.y = -20;
    PAR.plot_3D.grid_rect_yz.width = 40;
    PAR.plot_3D.grid_rect_yz.height = 40;    
    PAR.plot_3D.grid_yz_x = 0;
    
    PAR.plot_3D.show_frame = true;
    PAR.plot_3D.show_efk_feats = true ;
    PAR.plot_3D.show_ekf_anchors = true;
    PAR.plot_3D.show_global_map = true;
    PAR.plot_3D.show_ekf_trajectory = true;
    PAR.plot_3D.show_camera_pose = true;
    PAR.plot_3D.show_key_frames = true;

} 






void QUADdataset(parameters &PAR)
{
  
    // initial conditions   
    PAR.init.DataSetFile = "/home/rodrigo/RESEARCH/DataSets/QUAD/2017-4-20-9-36/";
    PAR.init.run_time = 115; // 90/115/190 run time
    PAR.init.GPS_init_yaw = 8*(3.1416/180);  // For aligning GPS points with local coordinates 
    PAR.init.roll_init = 0;
    PAR.init.pitch_init = 0;
    //PAR.init.yaw_init = 8*(3.1416/180);  // Quad dataset  
    PAR.init.yaw_init = 0*(3.1416/180);  // Quad dataset      
    PAR.init.x_init = 0;
    PAR.init.y_init = 0;
    PAR.init.z_init = 0;
    
    // general system parameters
    PAR.sys.closing_loop_active = true;
    PAR.sys.EKF_initialize_anchors = true;
    PAR.sys.EKF_use_anchors_from_GMAP = true;
    PAR.sys.GMAP_use_anchors_from_EKF = true; // (overwrite EKF_initialize_anchors )
    
    PAR.sys.GMAP_update_optimized_anchors = true;
    PAR.sys.GMAP_optimize_Keyframe_pos = false;
    PAR.sys.GMAP_update_optimized_Keyframe_pos = false; // (overwrite GMAP_optimize_Keyframe_pos )
    
    //--------------------------------------------------
    if( PAR.sys.GMAP_use_anchors_from_EKF == true) 
        PAR.sys.EKF_initialize_anchors = true;

    if(PAR.sys.GMAP_update_optimized_Keyframe_pos == true)
        PAR.sys.GMAP_optimize_Keyframe_pos = true;    

    // EKF parameters
    PAR.ekf.delta_t = 1/(double)24;
    PAR.ekf.sigma_a = 2; //.05
    PAR.ekf.sigma_w = .001;
    PAR.ekf.Tau_r = 1; // 1
    PAR.ekf.Tau_p = 100; // 2.5
    PAR.ekf.sigma_uv = 1; // 3 (pixels) camera measurement noise standard deviation  
    PAR.ekf.visual_attitude_update = true;
    PAR.ekf.Uncertainty_depth_anchor_init = .1;
    PAR.ekf.sigma_h = .05;  // altitude update uncertanty
    PAR.ekf.sigma_cl = .01; // closing loop uncertanty
    PAR.ekf.AntGPSoffset[0] = 0; // % GPS antena to origin of robot coordinate frame, Must be expresed in NED coordiantes
    PAR.ekf.AntGPSoffset[1] = 0;
    PAR.ekf.AntGPSoffset[2] = 0;
    PAR.ekf.sigma_id_range_dem = 20;
    PAR.ekf.sigma_id_WOrange_dem = 5;

    // Mapping parameters
    PAR.map.n_consec_kf_wo_link = 10;
    PAR.map.c_consec_anchor_wo_link = 100;
    PAR.map.n_consec_kf_wo_link_match = 50; // deprecated!!  // If there is no visual link from the last KFrame to the i-Kframe for n consecutive Kframes break (stop the matching)
    PAR.map.min_kf_matched_for_keep_anchor = 3;

    // Image processing paramters
    PAR.img.image_rows = 240;
    PAR.img.image_cols = 320;
    PAR.img.n_sub_imgs = 1;
    PAR.img.search_margin = 15;
    // DX201 DPS camera
         
        PAR.img.cam_parameters.distortions[0] = -0.38999;
        PAR.img.cam_parameters.distortions[1] = 0.13667; 
        PAR.img.cam_parameters.distortions[2] = -0.00000; 
        PAR.img.cam_parameters.distortions[3] = 0.00057; 
        PAR.img.cam_parameters.distortions[4] = 0.00000;   
        PAR.img.cam_parameters.cc[0] = 171.60729;
        PAR.img.cam_parameters.cc[1] = 137.90109 ;
        PAR.img.cam_parameters.fc[0] = 209.08860;
        PAR.img.cam_parameters.fc[1] = 206.46388;
        PAR.img.cam_parameters.alpha_c = 0;
        

    // RUM CAM
    /*
        PAR.img.cam_parameters.distortions[0] = -0.29597;
        PAR.img.cam_parameters.distortions[1] = 0.07012; 
        PAR.img.cam_parameters.distortions[2] = 0.00030; 
        PAR.img.cam_parameters.distortions[3] = 0.00029; 
        PAR.img.cam_parameters.distortions[4] = 0.00000;   
        PAR.img.cam_parameters.cc[0] = 160.12317;
        PAR.img.cam_parameters.cc[1] = 120.29071 ;
        PAR.img.cam_parameters.fc[0] = 184.10715;
        PAR.img.cam_parameters.fc[1] = 179.13770;
        PAR.img.cam_parameters.alpha_c = 0;
    */        
    PAR.img.check_innovation_for_mathing = false; 
    PAR.img.max_innov_pixels = 40; // if no stochastic innovation is checked then use this constant value
    PAR.img.minimun_distance_new_points = 20; // pixels

    // Sensors parameters
    PAR.sensors.range_sensor.max_range_pattern = 6; // max range of pattern, from sensor beam pattern
    PAR.sensors.range_sensor.r_max = 1.75; // maximun radius of elipsoid at (max range of pattern) from sensor beam pattern
    PAR.sensors.range_sensor.max_range_operation = 6; // max operation range of sensor (9m according to data sheet)
    PAR.sensors.range_sensor.Range_offset = 0; // offset for range measurements
    
    PAR.sensors.barometer_sensor.M = 0.0289644; // kg/mol standard molar mass of atmospheric air
    PAR.sensors.barometer_sensor.R = 8.31432; // N-m/mol-K universal gas constant for air
    PAR.sensors.barometer_sensor.a = .0065; // K/m
    PAR.sensors.barometer_sensor.g = 9.80665; // constant gravity
    PAR.sensors.barometer_sensor.T0 = 288.15; //  Kelvin standard temperature at sea level
    PAR.sensors.barometer_sensor.P0 =  101325; // N/m^2 standard pressure at sea level
    PAR.sensors.barometer_sensor.L0 = -0.0065; // K/m lapse rate of temperature deacrese in lower atmosphere
    //---------------------------------------------
    PAR.sensors.barometer_sensor.la =  1670 ; // altitude over sea level (zapopan)
    PAR.sensors.barometer_sensor.lt = 32.7633; // Temperature at flight location (celcius)

    // Close_loop process parameters
    PAR.close_loop.std_ab_vo = .1; // m
    PAR.close_loop.std_ab_cl = .05; // m
    PAR.close_loop.min_matches_for_potential_loop = 50;
    PAR.close_loop.min_inliers_for_potential_loop = 15; // Minimun inliers after RANSAC to consider a potential loop
    PAR.close_loop.min_KeyFrames_with_potential_loop = 2; // Minimun number of (near) keyframes with potential loop detections to consider an actual loop
    PAR.close_loop.min_matches_for_computing_pos = 6; // Minimun number of matches for computing the updated pos in function "get_measured_position_of_current_KF"
    PAR.close_loop.min_time_since_last_close  = 3; // seconds // After a closure, the close loop process must wait "min_num_f_since_last_close" frames for try to detect the next closure

    // Plot 3D
    PAR.plot_3D.viewer_width = 1400;
    PAR.plot_3D.viewer_height = 600;
    PAR.plot_3D.default_init_axis_view = 20;
    
    PAR.plot_3D.show_grid_xy = true;
    PAR.plot_3D.grid_rect_xy.x = -20;
    PAR.plot_3D.grid_rect_xy.y = -20;
    PAR.plot_3D.grid_rect_xy.width = 40;
    PAR.plot_3D.grid_rect_xy.height = 40;    
    PAR.plot_3D.grid_xy_z = -10;

    PAR.plot_3D.show_grid_yz = true;
    PAR.plot_3D.grid_rect_yz.x = -20;
    PAR.plot_3D.grid_rect_yz.y = -20;
    PAR.plot_3D.grid_rect_yz.width = 40;
    PAR.plot_3D.grid_rect_yz.height = 40;    
    PAR.plot_3D.grid_yz_x = -10;

    PAR.plot_3D.show_frame = true;
    PAR.plot_3D.show_efk_feats = true ;
    PAR.plot_3D.show_ekf_anchors = true;
    PAR.plot_3D.show_global_map = true;
    PAR.plot_3D.show_ekf_trajectory = true;
    PAR.plot_3D.show_camera_pose = true;
    PAR.plot_3D.show_key_frames = true;

}