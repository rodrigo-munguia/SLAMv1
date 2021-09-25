#include "plot.h"
#include <chrono>
#include <thread>
#include "getData.h"

parameters PLOT::PAR;

void  PLOT::KeyboardViz3d(const viz::KeyboardEvent &w, void *t)
{
    viz::Viz3d *viewer=(viz::Viz3d*)t;

   //cv::Affine3d T = viewer->getViewerPose();
   //cv::Vec3f t_c = T.translation(); 
   
   if (w.action)
      if (w.symbol == "F1")
      { 
        cv::Mat Rxy = (Mat_<double>(3,3) << 1, 0, 0, 0, -1, 0, 0, 0, -1); 
        cv::Vec3f t_c;
        t_c(0) = 0;
        t_c(1) = 0;
        t_c(2) = PAR.plot_3D.default_init_axis_view;
        cv::Affine3d Pose(Rxy, t_c);
        viewer->setViewerPose(Pose);
      }
      if (w.symbol == "F2")
      { 
        cv::Mat Ryz = (Mat_<double>(3,3) << 0, 0, -1, 1, 0, 0, 0, -1, 0); 
        cv::Vec3f t_c;
        t_c(0) = PAR.plot_3D.default_init_axis_view;
        t_c(1) = 0;
        t_c(2) = -3;
        cv::Affine3d Pose(Ryz, t_c);
        viewer->setViewerPose(Pose);
      }
      if (w.symbol == "F3")
      {
        cv::Mat Rxz = (Mat_<double>(3,3) << 1, 0, 0, 0, 0, 1, 0, -1, 0); 
        cv::Vec3f t_c;
        t_c(0) =  0;
        t_c(1) = -PAR.plot_3D.default_init_axis_view;
        t_c(2) = -3;
        cv::Affine3d Pose(Rxz, t_c);
        viewer->setViewerPose(Pose);
      } 

       //cout << "you pressed "<< w.code<<" = "<<w.symbol<< " in viz window "<<viewer->getWindowName()<<"\n";
}

void PLOT::init(EKF &ekf,GMAP &gmap,LOCKS &locks)
{
   int viewer_width = PAR.plot_3D.viewer_width;
   int viewer_height = PAR.plot_3D.viewer_height;
   
   //std::vector<cv::Point3d> EKFmap; 
   viz::Viz3d viewer("Viewer");
   viewer.setWindowSize(cv::Size(viewer_width,viewer_height));
   // set initial position
    cv::Mat Rxy = (Mat_<double>(3,3) << 1, 0, 0, 0, -1, 0, 0, 0, -1); 
        cv::Vec3f t_c;
        t_c(0) = 0;
        t_c(1) = 0;
        t_c(2) = PAR.plot_3D.default_init_axis_view;        
    cv::Affine3d Pose(Rxy, t_c);
    viewer.setViewerPose(Pose);   
   // set callbacks  
   viewer.registerKeyboardCallback(KeyboardViz3d,&viewer);   
   // set background
   viewer.setBackgroundColor(viz::Color::black(),viz::Color::not_set()); 
   // show coordinate reference 
   viewer.showWidget("Coordinate Widget", viz::WCoordinateSystem());
   // 
   viewer.spinOnce(10, true);

  string info1 = "F1-> xy view     F2->  yz view    F3->   xz view";
  cv::Point p;
            p.x = 0;
            p.y = 10;
  viz::WText tmark(info1,p,15,viz::Color::white());
  viewer.showWidget("info1", tmark); 
  

  //-------------- Grid

  if(PAR.plot_3D.show_grid_xy == true)
  {   
    cv::Rect grid_rect = PAR.plot_3D.grid_rect_xy;
      int z_grid = PAR.plot_3D.grid_xy_z ;    
      
      for(int i= grid_rect.y; i <= grid_rect.y + grid_rect.height; i++)
      {
      cv::Point3d p1,p2;
      p1.x = grid_rect.y;
      p1.y = i;
      p1.z = z_grid;
      p2.x = grid_rect.y + grid_rect.height;
      p2.y = i;
      p2.z = z_grid;
      viz::WLine l(p1, p2);
            string id = "lh" + to_string(i);
            l.setRenderingProperty(viz::LINE_WIDTH, .1);
            l.setRenderingProperty(viz::OPACITY,.2);         
            viewer.showWidget(id, l);
      }
      for(int i= grid_rect.x; i <= grid_rect.x + grid_rect.width; i++)
      {
      cv::Point3d p1,p2;
      p1.x = i;
      p1.y = grid_rect.x;
      p1.z = z_grid;
      p2.x = i;
      p2.y = grid_rect.x + grid_rect.width;
      p2.z = z_grid;
      viz::WLine l(p1, p2);
            string id = "lv" + to_string(i);
            l.setRenderingProperty(viz::LINE_WIDTH, .1);
            l.setRenderingProperty(viz::OPACITY,.2);         
            viewer.showWidget(id, l);
      }
  } 
  
  if(PAR.plot_3D.show_grid_yz == true)
  {   
      cv::Rect grid_rect = PAR.plot_3D.grid_rect_yz;
      int x_grid = PAR.plot_3D.grid_yz_x;  
      for(int i= grid_rect.x; i <= grid_rect.x + grid_rect.width; i++)
          {
          cv::Point3d p1,p2;
          p1.x = x_grid;
          p1.y = grid_rect.x;
          p1.z = i;
          p2.x = x_grid;
          p2.y = grid_rect.x + grid_rect.width;
          p2.z = i;
          viz::WLine l(p1, p2);
                string id = "lyzh" + to_string(i);
                l.setRenderingProperty(viz::LINE_WIDTH, .1);
                l.setRenderingProperty(viz::OPACITY,.2);         
                viewer.showWidget(id, l);
          }
          for(int i= grid_rect.y; i <= grid_rect.y + grid_rect.height; i++)
          {
          cv::Point3d p1,p2;
          p1.x = x_grid;
          p1.y = i;
          p1.z = grid_rect.y;
          p2.x = x_grid;
          p2.y = i;
          p2.z = grid_rect.y + grid_rect.height;
          viz::WLine l(p1, p2);
                string id = "lyzv" + to_string(i);
                l.setRenderingProperty(viz::LINE_WIDTH, .1);
                l.setRenderingProperty(viz::OPACITY,.2);         
                viewer.showWidget(id, l);
          }
  }      



  // main loop --------------------------------------------
   while(!viewer.wasStopped()||ekf.run == true)
    {     

      locks.ekf_run_mtx.lock();          
          cv::Mat EKFmap = ekf.store.EKFmap; 
          cv::Mat EKFmap_color = ekf.store.EKFmap_color;
          std::vector<cv::Point3f> EKFtrajectory = ekf.store.EKFtrajectory;          
          cv::Mat ANCHORSmap = ekf.store.ANCHORSmap;
          cv::Mat ANCHORSmap_color =  ekf.store.ANCHORSmap_color;
          std::vector<cv::Affine3d> pose = ekf.store.pose;
          FRAME current_frame = ekf.store.current_frame;
          vectorFeat FeatsDATA = *ekf.store.FeatsDATA; 
         vectorFeat AnchorsDATA = *ekf.store.AnchorsDATA;
     locks.ekf_run_mtx.unlock();

      locks.ReadGlobalMAP_mtx.lock();
         cv::Mat GLOBALmap = gmap.store.GLOBALmap; 
         cv::Mat GLOBALmap_color = gmap.store.GLOBALmap_color;
         std::vector<cv::Affine3d> path = gmap.store.path;
      locks.ReadGlobalMAP_mtx.unlock(); 
       
      
     
     //---------------- show frame
     if(PAR.plot_3D.show_frame == true)
     {
        cv::Size viewer_size = viewer.getWindowSize();
        viewer_height = viewer_size.height;      
        // upper-left location
        cv::Rect frame_rect(0,viewer_height-current_frame.image.rows,current_frame.image.cols,current_frame.image.rows);
        cv::viz::WImageOverlay frame(current_frame.image,frame_rect);
        viewer.showWidget("frame",frame);
        
        for(int i =0; i<FeatsDATA.size(); i++)
        {         
          string id = "s" + to_string(i);
          if(FeatsDATA[i].predicted == true)
          { 
              cv::Point p;
              p.x = FeatsDATA[i].PredictedPoint.x + frame_rect.x;
              p.y = FeatsDATA[i].PredictedPoint.y + frame_rect.y;
              if (FeatsDATA[i].matched == true)
              {
                viz::WText tmark("o",p,10,viz::Color::blue());
                viewer.showWidget(id, tmark);
              }
              else
              {
                viz::WText tmark(".",p,10,viz::Color::red());
                viewer.showWidget(id, tmark);
              }           
          }       
        }
        for(int i =0; i<AnchorsDATA.size(); i++)
        {         
          string id = "s" + to_string(i);
          if(AnchorsDATA[i].predicted == true)
          { 
              cv::Point p;
              p.x = AnchorsDATA[i].PredictedPoint.x + frame_rect.x;
              p.y = AnchorsDATA[i].PredictedPoint.y + frame_rect.y;
              if (AnchorsDATA[i].matched == true)
              {
                viz::WText tmark("o",p,10,viz::Color::green());
                viewer.showWidget(id, tmark);
              }
              else
              {
                viz::WText tmark(".",p,10,viz::Color::yellow());
                viewer.showWidget(id, tmark);
              }           
          }       
        }
     }  
      //-----------------------------------------------------------------------

      
      //----------- plot EKF feats
      if(PAR.plot_3D.show_efk_feats == true)
      {
        if (EKFmap.rows > 0)
        {             
          //viewer.removeWidget("Cloud"); 
            //cv::Mat colors(EKFmap.rows,3, CV_8UC3);
            //cv::randu(colors, Scalar(0, 0, 0), Scalar(255, 255, 255));         
            if(EKFmap.size() == EKFmap_color.size())
            {         
              cv::viz::WCloud cloud(EKFmap,EKFmap_color);
              cloud.setRenderingProperty(cv::viz::POINT_SIZE, 2);
              viewer.showWidget( "Cloud", cloud );           

            }                 
        }
      }    
      //---------- plot EKF anchors
      if(PAR.plot_3D.show_ekf_anchors == true)
      {
        if (ANCHORSmap.rows > 0)
        {             
          //viewer.removeWidget("Cloud"); 
            //cv::Mat colors(EKFmap.rows,3, CV_8UC3);
            //cv::randu(colors, Scalar(0, 0, 0), Scalar(255, 255, 255));         
            if(ANCHORSmap.size() == ANCHORSmap_color.size())
            {         
              cv::viz::WCloud cloud_a(ANCHORSmap,ANCHORSmap_color);
              cloud_a.setRenderingProperty(cv::viz::POINT_SIZE, 2);
              viewer.showWidget( "Cloud_a", cloud_a );
            }                 
        }
      }  
      //------- Plot Global Map
      if(PAR.plot_3D.show_global_map==true)
      {
        if (GLOBALmap.rows > 0)
        {             
          //viewer.removeWidget("Cloud"); 
            //cv::Mat colors(EKFmap.rows,3, CV_8UC3);
            //cv::randu(colors, Scalar(0, 0, 0), Scalar(255, 255, 255));         
            if(GLOBALmap.size() == GLOBALmap_color.size())
            {         
              cv::viz::WCloud cloud_g(GLOBALmap,GLOBALmap_color);
              cloud_g.setRenderingProperty(cv::viz::POINT_SIZE, 1);
              viewer.showWidget( "Cloud_g", cloud_g );
            }                 
        }
      }    
      //----------------  Plot EKF camera trajectory
      if(PAR.plot_3D.show_ekf_trajectory == true)
      {
        for(int i = 1; i<EKFtrajectory.size(); i++ )
        {
          string id = to_string(i);
          viz::WLine segment(EKFtrajectory[i-1], EKFtrajectory[i]);
          segment.setRenderingProperty(viz::LINE_WIDTH, 1.0);
          viewer.showWidget(id, segment);
        }
      }  
      
      //---- camera pose
      if(PAR.plot_3D.show_camera_pose == true)
      {
        if(pose.size()>0)
        {
        viz::WTrajectoryFrustums cam_pose(pose, Vec2f(0.889484, 0.523599), 0.5,viz::Color::white());
        viewer.showWidget("camera", cam_pose);
        }
      }  


       // ------------- Plot KeyFrames
       if(PAR.plot_3D.show_key_frames == true)
       {
        if(path.size()>0)
        { 
          cv::viz::WTrajectory trajectory(path,cv::viz::WTrajectory::PATH,1,cv::viz::Color::blue());
          viewer.showWidget("Trajectory",trajectory);
          viz::WTrajectoryFrustums frustums(path, Vec2f(0.889484, 0.523599), 0.2,
                                      viz::Color::yellow());
          viewer.showWidget("frustums", frustums);
        }
       } 

        viewer.spinOnce(10, true);
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        

    }   
       
    
  

}


void PLOT::Update(EKF &ekf,GMAP &gmap)
{
  
  
        
  /*
  
  

  */

   
 
}