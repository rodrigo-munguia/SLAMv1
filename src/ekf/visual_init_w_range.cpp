
#include "visual_init_w_range.h"
#include "../anms/anms.h"
#include "../Transforms/quat2R.h"
#include "../Jacs/Jac_XYZ_uvr.h"






//-----------------------------------------------------------------------------
//   Initialize visual features with range measurements
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
using namespace arma;

void visual_init_w_range(arma::vec& x, arma::mat& P, parameters &par,FRAME *frame,vectorFeat& FeatsDATA,vectorFeat& AnchorsDATA  )
{
    vector<cv::KeyPoint> keyPoints;
    
    int fastThresh = 10; // Fast threshold. Usually this value is set to be in range [10,35]
   // cv::FAST(frame->image,keyPoints,fastThresh,true);
    
    // int nfeatures=500, float scaleFactor=1.2f, int nlevels=8, int edgeThreshold=31, int firstLevel=0, int WTA_K=2, ORB::ScoreType scoreType=ORB::HARRIS_SCORE, int patchSize=31, int fastThreshold=20
    Ptr<FeatureDetector> detector = ORB::create(1000, 1.2f, 8, 16,0,2, ORB::FAST_SCORE,31, 5);
    detector->detect(frame->image, keyPoints);
    
    /*
    cv::Mat fastDetectionResults; //draw FAST detections
    cv::drawKeypoints(frame->image,keyPoints,fastDetectionResults, cv::Scalar(94.0, 206.0, 165.0, 0.0));
    cv::namedWindow("FAST Detections", cv::WINDOW_AUTOSIZE); 
    cv::imshow( "FAST Detections", fastDetectionResults);
    */ 
    
    //Sorting keypoints by deacreasing order of strength
    //clock_t sscStart = clock();
    
    vector<float> responseVector;
    for (unsigned int i =0 ; i<keyPoints.size(); i++) responseVector.push_back(keyPoints[i].response);
    vector<int> Indx(responseVector.size()); 
    std::iota (std::begin(Indx), std::end(Indx), 0);
    cv::sortIdx(responseVector, Indx, cv::SORT_DESCENDING);
    vector<cv::KeyPoint> keyPointsSorted;
    for (unsigned int i = 0; i < keyPoints.size(); i++) keyPointsSorted.push_back(keyPoints[Indx[i]]);

    int numRetPoints;
    if (FeatsDATA.size() > 0 )
    {
        numRetPoints = 100; //choose exact number of return points
    }
    else
    {
       numRetPoints = 50;
    }
    
        
    //float percentage = 0.1; //or choose percentage of points to be return
    //int numRetPoints = (int)keyPoints.size()*percentage;

    float tolerance = 0.1; // tolerance of the number of return points

    vector<cv::KeyPoint> sscKP = Ssc(keyPointsSorted,numRetPoints,tolerance,frame->image.cols,frame->image.rows);
    //clock_t sscTotalTime = double( clock() - sscStart)*1000/(double)CLOCKS_PER_SEC;
    //cout << "Finish SSC ANMS " << sscTotalTime << " miliseconds." << endl;
    
  
    //--- filtrate keypoints that are to close to other already initilized features
    // 
    vector<cv::KeyPoint> Kp_init;
    remove_close_points(Kp_init,sscKP,par,FeatsDATA,AnchorsDATA  );
    
    // test new points detected
      /*     
       for(int i =0 ; i< FeatsDATA.size();i++ )
       {
           if(FeatsDATA[i].predicted == true)
           {
               cv::circle(frame->image,FeatsDATA[i].Keypoint.pt, 5, (0,0,255), 1,8);

           }

       } 
       for(int i =0 ; i< Kp_init.size();i++ )
       {
                     
               cv::circle(frame->image,Kp_init[i].pt, 5, (0,0,255), -1,8);

       } 
        namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
        imshow( "Display window", frame->image );                   // Show our image inside it.
        waitKey(0);          
    */
    //.......

    cv::Mat descriptors;


    detector->compute(frame->image, Kp_init,descriptors); // compute descriptors for Keypoints
    //----------------------------
       

     for (unsigned int i =0 ; i<Kp_init.size(); i++)
     {
        init_XYZ_feat_point(x,P,par,frame,Kp_init[i],descriptors.row(i),FeatsDATA); // initialize new points in state
     }


    
}
//---------------------------------------------------------------------------------------------------------
void remove_close_points(vector<cv::KeyPoint>& Kp_init, vector<cv::KeyPoint>&  sscKP, parameters &par,vectorFeat& FeatsDATA, vectorFeat& AnchorsDATA  )
{

 if (FeatsDATA.size() > 0 )
    {
        for(int i = 0; i < sscKP.size(); i++) // check each candidate point to be at a minimun distance from predicted features 
        {
            
            bool add = true;
            for(int j = 0; j < FeatsDATA.size(); j++)
            {
                double d =  sqrt( pow(sscKP[i].pt.x - FeatsDATA[j].PredictedPoint.x, 2) + pow(sscKP[i].pt.y - FeatsDATA[j].PredictedPoint.y, 2) );

                if (d < par.img.minimun_distance_new_points)
                {
                    add = false;
                    break;
                }
            }
            for(int j = 0; j < AnchorsDATA.size(); j++ )
            {
                double d =  sqrt( pow(sscKP[i].pt.x - AnchorsDATA[j].PredictedPoint.x, 2) + pow(sscKP[i].pt.y -AnchorsDATA[j].PredictedPoint.y, 2) );
                if (d < par.img.minimun_distance_new_points)
                {
                    add = false;
                    break;
                }

            }     

            if(add == true)
            {
                Kp_init.push_back(sscKP[i]);
            }
        
        }

    
    }
    else
    {
        Kp_init = sscKP;
    } 


}
//----------------------------------------------------------------------------------------------------------
void init_XYZ_feat_point(arma::vec& x, arma::mat& P, parameters &par,FRAME *frame,cv::KeyPoint kp,cv::Mat Descriptor, vectorFeat& FeatsDATA)
{
    
    //-- test data
    /*
       arma::vec xin;
        xin.load("x_in.txt");
        //x = xin.subvec(0,12);
        x.set_size( size(xin) );
        x = xin;
        arma::mat Pin;
        Pin.load("P_in.txt");
        P.set_size(size(Pin)); 
        P = Pin;  
        kp.pt.x = 136;
        kp.pt.y = 106;
        frame->range = 3.72;
        */
    //---

    // system state augmentation -------------------------------
    
    cv::Point2d uvd;
    uvd = kp.pt;
    arma::vec::fixed<3> hc;
    arma::mat::fixed<3,2> dhc_duvd;    

    hc = Inverse_projection_model(uvd,1,false, par.img.cam_parameters,dhc_duvd); // compute inverse projection model
    
    std::vector<double> quat;
    quat = arma::conv_to<vec_t>::from(x.subvec(0,3)); // convert from armadillo vector to c++ vector
    double Rn2c_a[9];

    quat2R(&quat[0],Rn2c_a);
    
    arma::mat Rn2c(Rn2c_a,3,3); // convert from arrat to armadillo

    arma::mat::fixed<3,3> Rc2n = Rn2c.t();

    arma::vec::fixed<3> hn = Rc2n*hc;

    arma::vec::fixed<3> m = hn/arma::norm(hn); // normalized vector (in the nav frame) pointing in the direction of the feature
    
    //-- compute feature depth from range measurement
    arma::vec::fixed<3> v = {0,0,1};
    arma::vec::fixed<3> vn =  Rc2n*v;
    double depth = (frame->range + par.sensors.range_sensor.Range_offset) /arma::dot(m,vn);   
    //---------


    arma::vec new_yi = x.subvec(7,9) + depth*m;  // compute new Euclidean (XYZ) feature map
    
    int old_x_size = x.n_elem;
    x.resize( old_x_size +3 ); // resize state vector
    int new_x_size = x.n_elem;
    x.subvec(old_x_size,new_x_size-1) = new_yi; // augmentate state with new feature state
    
    
    // ------------ System covariance matrix augmentation
    
    //-- compute radious in the image covered by the range sensor 
    double ce = par.sensors.range_sensor.max_range_pattern/pow(par.sensors.range_sensor.r_max,2);
    double r_at_c = sqrt(frame->range/ce);
    arma::vec::fixed<3> Pc = {r_at_c,0,frame->range};
    
    arma::mat::fixed<2,3> duv_dPc;
    
    cv::Point2d uvd_r = Projection_model(Pc,1,false,par.img.cam_parameters,duv_dPc );

    double r_range_image = uvd_r.x - par.img.cam_parameters.cc[0];

    
    //--- compute if the visual point is inside the area of measured by the range senor 
    cv::Point2d uv;
    uv = Undistort_a_point(uvd,par.img.cam_parameters,1);
    
    double dp = sqrt(  pow(par.img.cam_parameters.cc[0] - uv.x,2) + pow(par.img.cam_parameters.cc[1] - uv.y,2) );
    
    
    double sig_d_ini;
    if( (dp < r_range_image)&&(frame->range < par.sensors.range_sensor.max_range_operation ))
    {
        //sig_d_ini = depth/15; 
        sig_d_ini = depth/par.ekf.sigma_id_range_dem;   
    }
    else
    {
        sig_d_ini = depth/par.ekf.sigma_id_WOrange_dem;    // 5
        //sig_d_ini = depth/5;   
    }

    //---- get Initialization Jacobians
    arma::mat::fixed<3,13> dy_dx;  
    arma::mat::fixed<3,3> dy_duvr;    
    Jac_XYZ_uvr(uvd,x,par,depth,dy_dx,dy_duvr); 
    
    arma::mat dh_dx;
    dh_dx = zeros(3,old_x_size);
    dh_dx(arma::span(0,2),arma::span(0,12)) = dy_dx;
    
    arma::mat::fixed<3,3> Ry;
    Ry(0,0) = pow(par.ekf.sigma_uv,2);
    Ry(1,1) = pow(par.ekf.sigma_uv,2);
    Ry(2,2) = pow(sig_d_ini,2);
    
    arma::mat dh_dxP = dh_dx*P;      
    arma::mat::fixed<3,3> Py =  dh_dxP(arma::span(0,2),arma::span(0,12))*dy_dx.t() + dy_duvr*Ry*dy_duvr.t();   

    P.resize( old_x_size +3, old_x_size +3); // resize covariance matrix
    P(arma::span(old_x_size,new_x_size-1),arma::span(old_x_size,new_x_size-1)) = Py;    
    P(arma::span(old_x_size,new_x_size-1),arma::span(0,old_x_size-1)) = dh_dxP;
    P(arma::span(0,old_x_size-1),arma::span(old_x_size,new_x_size-1)) = dh_dxP.t();
     
   
    //-- test data
        /*
        arma::vec xout;
        xout.load("x_out.txt");
        arma::mat Pout;
        Pout.load("P_out.txt");
        //xout.subvec(0,12).print();
        //cout << "xxxx" << endl;
        //x.print();
        double dif = arma::norm(P-Pout, "inf");
        */
    //-----

    //--------- Update table features 
    
    
    FEAT feat;  // feature data structure

    feat.feat_type = "XYZ";
    feat.idx_i_state = old_x_size;
    feat.idx_f_state = new_x_size-1;
    feat.Initial_KeyPoint = kp;
    feat.Keypoint = kp;
    feat.Descriptor = Descriptor;
    feat.times_mathed = 0;
    feat.times_not_mathed = 0;
    feat.times_not_considered = 0;
    feat.CameraState = x.subvec(7,9);
    feat.init_type = 0;
     
     
    FeatsDATA.push_back(feat);
    




}