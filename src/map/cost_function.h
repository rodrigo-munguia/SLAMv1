#ifndef COST_F_H
#define COST_F_H

#include "ceres/ceres.h"
#include "../parameters.h"
#include <armadillo>
#include "opencv2/opencv.hpp"
#include "ceres/rotation.h"

//---------------------------------------------------------------------------------------------------
// Local bundle ajustment
// Rodrigo M. 2020
//-----------------------------------------------------------------------------------------------------
class CostFunction : public ceres::SizedCostFunction<2, /* number of residuals */
                                                    8, /* size of camera parameter */
                                                    3> /*, /* size of point parameter */ {
                                                   /* 7>   size of intrinsic camera parameter */ //{ 
   public:
     CostFunction(const double observed_x, const double observed_y,parameters &par) : observed_x(observed_x), observed_y(observed_y),par(par) {}
     virtual ~CostFunction() {}
    
     virtual bool Evaluate(double const* const* parameters,
                           double* residuals,
                           double** jacobians) const {
       const double q1 = parameters[0][0];
       const double q2 = parameters[0][1];
       const double q3 = parameters[0][2];
       const double q4 = parameters[0][3];
       const double t1 = parameters[0][4];
       const double t2 = parameters[0][5];
       const double t3 = parameters[0][6];

       const double p1 = parameters[1][0];
       const double p2 = parameters[1][1];
       const double p3 = parameters[1][2];
       
     
       const double f_x = par.img.cam_parameters.fc[0];
       const double f_y = par.img.cam_parameters.fc[1];
       const double cx = par.img.cam_parameters.cc[0];
       const double cy = par.img.cam_parameters.cc[1];
       const double k1 = par.img.cam_parameters.distortions[0];
       const double k2 = par.img.cam_parameters.distortions[1];
       const double alpha_c = par.img.cam_parameters.alpha_c;

       //  Rn2b1 = [(b(1)^2+b(2)^2-b(3)^2-b(4)^2) 2*(b(2)*b(3)-b(1)*b(4)) 2*(b(1)*b(3)+b(2)*b(4)) 
    //              2*(b(2)*b(3)+b(1)*b(4)) (b(1)^2-b(2)^2+b(3)^2-b(4)^2) 2*(b(3)*b(4)-b(1)*b(2)) 
    //              2*(b(2)*b(4)-b(1)*b(3)) 2*(b(1)*b(2)+b(3)*b(4))   b(1)^2-b(2)^2-b(3)^2+b(4)^2] 
       arma::mat::fixed<3,3> Rn2c;
       Rn2c(0,0) = q1*q1+q2*q2-q3*q3-q4*q4;
       Rn2c(0,1) = 2*(q2*q3-q1*q4);
       Rn2c(0,2) = 2*(q1*q3+q2*q4);
       Rn2c(1,0) = 2*(q2*q3+q1*q4);
       Rn2c(1,1) = q1*q1-q2*q2+q3*q3-q4*q4;
       Rn2c(1,2) = 2*(q3*q4-q1*q2);
       Rn2c(2,0) = 2*(q2*q4-q1*q3);
       Rn2c(2,1) = 2*(q1*q2+q3*q4);
       Rn2c(2,2) = q1*q1-q2*q2-q3*q3+q4*q4;

       arma::vec::fixed<3> Tn2c;
       Tn2c(0) = t1;
       Tn2c(1) = t2;
       Tn2c(2) = t3;

       arma::vec::fixed<3> pxyz;
       pxyz(0) = p1;
       pxyz(1) = p2;
       pxyz(2) = p3;

    arma::vec::fixed<3> pn = (pxyz - Tn2c);

    arma::vec::fixed<3> pc = Rn2c*pn; // feature point expressed in the camera frame

    const double um = pc(0)/pc(2);
    const double vm = pc(1)/pc(2);     
    
    // Apply second and fourth order radial distortion.
    const double r2 = um*um + vm*vm;
    const double distortion = 1.0 + r2  * (k1 + k2  * r2);

    // Compute final projected point position.  
    const double predicted_x = f_x * distortion * um + cx ;
    const double predicted_y = f_y * distortion * vm + cy ;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;


       if (jacobians != NULL && jacobians[0] != NULL) 
       { 
        
        arma::mat::fixed<3,3>  KK = {{f_x, alpha_c*f_y, cx},
                                   { 0 ,    f_y  ,    cy},
                                   { 0 ,      0  ,    1  }};
        arma::vec::fixed<3> ph = KK*pc;

        cv::Point2d uv;

        uv.x = ph(0)/ph(2);   // undistorted image coordinates
        uv.y = ph(1)/ph(2);    

        double  um = (uv.x-cx)/f_x;
        double vm = (uv.y-cy)/f_y;
        //double r = um*um + vm*vm;
        //double g = (1 + k1*r*r + k2*pow(r,4));
        
        double dg_d_r = 4*k2*pow(r2,3) + 2*k1*r2;

        arma::mat::fixed<2,2> duvd_duvmd  = { {f_x,  0 },
                                            {0,    f_y}};                          
        arma::vec::fixed<2> uvm = {um,vm};        
        arma::rowvec::fixed<2> dr_duvm = { um/r2, vm/r2} ;  
        arma::mat::fixed<2,2> duvmd_duvm = arma::eye(2,2)*distortion + uvm*dg_d_r*dr_duvm;
        arma::mat::fixed<2,2> duvm_uv  = { {1/f_x,  0 },
                                            {0,    1/f_y}};
        arma::mat::fixed<2,2> duv_uvd = duvd_duvmd*duvmd_duvm*duvm_uv; 

        double Pc1 = pc(0);
        double Pc2 = pc(1);
        double Pc3 = pc(2);
      
        arma::mat::fixed<2,3> duvd_Pc = {{ f_x/Pc3, (alpha_c*f_y)/Pc3, cx/Pc3 - (Pc3*cx + Pc1*f_x + Pc2*alpha_c*f_y)/pow(Pc3,2)},
                                        { 0, f_y/Pc3, cy/Pc3 - (Pc3*cy + Pc2*f_y)/pow(Pc3,2)}};
           
        arma::mat::fixed<2,3> duv_dPc;
        duv_dPc = duv_uvd*duvd_Pc;

          const double Pn1 = pn(0);
          const double Pn2 = pn(1);
          const double Pn3 = pn(2);
          arma::mat::fixed<3,4> dPc_dq;
          dPc_dq = {{ 2*Pn3*q1 + 2*Pn2*q2 - 2*Pn1*q3, 2*Pn2*q1 - 2*Pn3*q2 + 2*Pn1*q4, 2*Pn2*q4 - 2*Pn3*q3 - 2*Pn1*q1, 2*Pn1*q2 + 2*Pn2*q3 + 2*Pn3*q4},
                  { 2*Pn1*q1 + 2*Pn3*q3 - 2*Pn2*q4, 2*Pn1*q2 + 2*Pn2*q3 + 2*Pn3*q4, 2*Pn3*q1 + 2*Pn2*q2 - 2*Pn1*q3, 2*Pn3*q2 - 2*Pn2*q1 - 2*Pn1*q4},
                  { 2*Pn2*q1 - 2*Pn3*q2 + 2*Pn1*q4, 2*Pn1*q3 - 2*Pn2*q2 - 2*Pn3*q1, 2*Pn1*q2 + 2*Pn2*q3 + 2*Pn3*q4, 2*Pn1*q1 + 2*Pn3*q3 - 2*Pn2*q4}}; 

          arma::mat::fixed<3,7> dPc_dx;          
          dPc_dx(arma::span(0,2),arma::span(0,3)) = dPc_dq;
          dPc_dx(arma::span(0,2),arma::span(4,6)) = -1*Rn2c;
          
          
          arma::mat::fixed<2,7> duv_dx; 
          arma::mat::fixed<2,3> duv_dy;
          //----------
          duv_dx = duv_dPc*dPc_dx;
          //-----------

          duv_dy = duv_dPc*Rn2c;

          //duv_dx.print();
          //duv_dy.print();
          
          // row-major jacobian for camera paramter block 
          jacobians[0][0] = duv_dx(0,0);
          jacobians[0][1] = duv_dx(0,1);
          jacobians[0][2] = duv_dx(0,2);
          jacobians[0][3] = duv_dx(0,3);
          jacobians[0][4] = duv_dx(0,4);
          jacobians[0][5] = duv_dx(0,5);
          jacobians[0][6] = duv_dx(0,6);
          jacobians[0][7] = 0; // for dummy parameter!!          
          
          jacobians[0][8] = duv_dx(1,0);
          jacobians[0][9] = duv_dx(1,1);
          jacobians[0][10] = duv_dx(1,2);
          jacobians[0][11] = duv_dx(1,3);
          jacobians[0][12] = duv_dx(1,4);
          jacobians[0][13] = duv_dx(1,5);
          jacobians[0][14] = duv_dx(1,6); 
          jacobians[0][15] = 0; // for dummy parameter!!               

          // row-major jacobian for point block
          jacobians[1][0] = duv_dy(0,0);
          jacobians[1][1] = duv_dy(0,1);
          jacobians[1][2] = duv_dy(0,2);               

          jacobians[1][3] = duv_dy(1,0);
          jacobians[1][4] = duv_dy(1,1);
          jacobians[1][5] = duv_dy(1,2);


       }
       return true;
     }

   private:
     double observed_x;
     double observed_y;
     parameters par;
 };



//------------------------------------------------------------------------------------------
// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 10 parameters. 4 for rotation, 3 for
// translation, 1 for focal length and 2 for radial distortion. The
// principal point is not modeled (i.e. it is assumed be located at
// the image center).
struct ReprojectionErrorWithQuaternions {
  // (u, v): the position of the observation with respect to the image
  // center point.
  ReprojectionErrorWithQuaternions(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  const T* const cam_par,
                  T* residuals) const {
    // camera[0,1,2,3] is are the rotation of the camera as a quaternion.
    //
    // We use QuaternionRotatePoint as it does not assume that the
    // quaternion is normalized, since one of the ways to run the
    // bundle adjuster is to let Ceres optimize all 4 quaternion
    // parameters without a local parameterization.
    
    // pc = Rn2c*(pxyz - Tn2c); // feature point expressed in the camera frame
    T p[3];
    p[0] =  point[0] - camera[4];
    p[1] =  point[1] - camera[5];
    p[2] =  point[2] - camera[6];

    T pc[3];
    //ceres::QuaternionRotatePoint(camera, p, pc);
    ceres::UnitQuaternionRotatePoint(camera,p,pc); 
    // --------------------------------------------  
    
    /*
    arma::mat::fixed<3,3>  KK = {{fc1, alpha_c*fc2, cc1},
                                   { 0 ,    fc2  ,    cc2},
                                   { 0 ,      0  ,    1  }};
     arma::vec::fixed<3> ph = KK*Pc;
     /* uv.x = ph(0)/ph(2);   // undistorted image coordinates
     uv.y = ph(1)/ph(2);   
    */ 
    

     const T um = pc[0]/pc[2];
     const T vm = pc[1]/pc[2];
     
    

    // Apply second and fourth order radial distortion.
    const T& k1 = cam_par[4];
    const T& k2 = cam_par[5];

    const T r2 = um*um + vm*vm;
    const T distortion = 1.0 + r2  * (k1 + k2  * r2);

    // Compute final projected point position.
    const T& f_x = cam_par[0];
    const T& f_y = cam_par[1];
    const T predicted_x = f_x * distortion * um + cam_par[2] ;
    const T predicted_y = f_y * distortion * vm + cam_par[3] ;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.  
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<
            ReprojectionErrorWithQuaternions, 2, 8, 3, 7>(
                new ReprojectionErrorWithQuaternions(observed_x,
                                                            observed_y)));
  }

  double observed_x;
  double observed_y;
};









 #endif