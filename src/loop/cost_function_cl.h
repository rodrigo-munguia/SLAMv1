#ifndef COST_F_H_CL
#define COST_F_H_CL

#include "ceres/ceres.h"
#include "../parameters.h"
#include <armadillo>
#include "opencv2/opencv.hpp"
#include "ceres/rotation.h"

//---------------------------------------------------------------------------------------------------
// Close loop (graph-based SLAM)
// Rodrigo M. 2020
//-----------------------------------------------------------------------------------------------------
class CostFunctionCL : public ceres::SizedCostFunction<2, /* number of residuals */
                                                    2,
                                                    2> /* size of camera parameter */
                                                    { 
   public:
     CostFunctionCL(const double x_ab, const double y_ab,parameters &par,int observation_type) : x_ab(x_ab), y_ab(y_ab),par(par),observation_type(observation_type) {}
     virtual ~CostFunctionCL() {}
    
     virtual bool Evaluate(double const* const* parameters,
                           double* residuals,
                           double** jacobians) const {
       const double x_a = parameters[0][0];
       const double y_a = parameters[0][1];
       const double x_b = parameters[1][0];
       const double y_b = parameters[1][1];
       
     
       const double std_vo = par.close_loop.std_ab_vo;  // standard deviation "visual odometry" observation
       const double std_cl = par.close_loop.std_ab_cl;  // standard deviation "close loop" observation
       
    if(observation_type == 0) // if it is a visual odometry observation
    {
        // The error is the difference between the predicted and observed relative position (weighted by the inverse of standard deviation).
        residuals[0] = (1/std_vo)*((x_b- x_a) - x_ab);
        residuals[1] = (1/std_vo)*((y_b- y_a) - y_ab);
    }
    else if(observation_type == 1) // if it is a clossing loop observation
    {
        residuals[0] = (1/std_cl)*((x_b- x_a) - x_ab);
        residuals[1] = (1/std_cl)*((y_b- y_a) - y_ab);
    }
    

     //if (!jacobians) return true;
     //  double* jacobian = jacobians[0];
     //  if (!jacobian) return true;

       if (jacobians != NULL && jacobians[0] != NULL) 
       {         
          double s;
          if(observation_type == 0)
            s = std_vo;
          else if(observation_type == 1)
            s = std_cl;  
         
          // row-major jacobian parameter block 

         jacobians[0][0] = -1/s;
         jacobians[0][1] = 0;
         jacobians[0][2] = 0;
         jacobians[0][3] = -1/s;
       
         jacobians[1][0] = 1/s;
         jacobians[1][1] = 0;
         jacobians[1][2] = 0;
         jacobians[1][3] = 1/s; // for dummy parameter!!  

        int q = 10;

       }
       return true;
     }

   private:
     double x_ab;
     double y_ab;
     parameters par;
     int observation_type;
 };


//----- Autodifferentiation cost function

struct CostFunctionCL2 {  
  CostFunctionCL2(const double x_ab, const double y_ab,const Eigen::Matrix<double, 2, 2>& sqrt_information,int observation_type) : x_ab(x_ab), y_ab(y_ab),sqrt_information_(sqrt_information),observation_type(observation_type) {}

  template <typename T>
  bool operator()(const T* const pos_a,
                  const T* const pos_b,                  
                  T* residuals) const {

        
    
    
    const T x_a =  pos_a[0];
    const T y_a = pos_a[1];
    
    const T x_b =  pos_b[0];
    const T y_b = pos_b[1];

    
   
        // The error is the difference between the predicted and observed relative position (weighted by the inverse of standard deviation).
        residuals[0] =((x_b- x_a) - x_ab);
        residuals[1] =((y_b- y_a) - y_ab);
        Eigen::Map<Eigen::Matrix<T, 2, 1> > residuals_map(residuals);
        residuals_map = sqrt_information_.template cast<T>() * residuals_map;
   

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.  
  static ceres::CostFunction* Create(const double x_ab, const double y_ab,const Eigen::Matrix<double, 2, 2>& sqrt_information,int observation_type) {
    return (new ceres::AutoDiffCostFunction<
            CostFunctionCL2, 2, 2, 2>(
                new CostFunctionCL2(x_ab,y_ab,sqrt_information,observation_type)));
  }

  
     double x_ab;
     double y_ab;
     const Eigen::Matrix<double, 2, 2> sqrt_information_;
     int observation_type;
};

//---------------------------------------------------------------------------------------------------
// PNP function with altitude and Camera fixed rotation
// Rodrigo M. 2020
//-----------------------------------------------------------------------------------------------------


struct CostFunctionPNPr {  
  CostFunctionPNPr(const double u, const double v,const double x,const double y,const double z,const arma::mat::fixed<3,3>  R,const double t_z , const parameters &par) : u(v), v(v),x(x),y(y),z(z),R(R),t_z(t_z),par(par) {}

  template <typename T>
  bool operator()(const T* const t_x,
                  const T* const t_y,                  
                  T* residuals) const {
    
    const double f_x = par.img.cam_parameters.fc[0];
    const double f_y = par.img.cam_parameters.fc[1];
    const double cx = par.img.cam_parameters.cc[0];
    const double cy = par.img.cam_parameters.cc[1];
    
    double r_11 = R(0,0);
    double r_12 = R(0,1);
    double r_13 = R(0,2);

    const T pc_x = r_11*x + r_12*y + r_13*z + t_x[0];
    const T pc_y = R(1,0)*x + R(1,1)*y + R(1,2)*z + t_y[0];
    const double pc_z = R(2,0)*x + R(2,1)*y + R(2,2)*z + t_z;

    const T up = f_x*pc_x + cx*pc_z;
    const T vp = f_y*pc_y + cy*pc_z;
    const double s = pc_z;

   residuals[0] = u - up/s;
   residuals[1] = v - vp/s;
     

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.  
  static ceres::CostFunction* Create(const double u, const double v,const double x,const double y,const double z,arma::mat::fixed<3,3>  R,const double t_z , const parameters &par) {
    return (new ceres::AutoDiffCostFunction<
            CostFunctionPNPr, 2, 1,1>(
                new CostFunctionPNPr(u,v,x,y,z,R,t_z,par)));
  }

  
     double u;
     double v;
     double x;
     double y;
     double z;
     const arma::mat::fixed<3,3>  R;
     const double t_z;
     parameters par;
};

//-------------------------------------------------------------------------------------------
// estimates x,y,z camera position
struct CostFunctionPNPr2 {  
  CostFunctionPNPr2(const double u, const double v,const double x,const double y,const double z,const arma::mat::fixed<3,3>  R, const parameters &par) : u(v), v(v),x(x),y(y),z(z),R(R),par(par) {}

  template <typename T>
  bool operator()(const T* const t_x,
                  const T* const t_y,
                  const T* const t_z,                  
                  T* residuals) const {
    
    const double f_x = par.img.cam_parameters.fc[0];
    const double f_y = par.img.cam_parameters.fc[1];
    const double cx = par.img.cam_parameters.cc[0];
    const double cy = par.img.cam_parameters.cc[1];
    
    double r_11 = R(0,0);
    double r_12 = R(0,1);
    double r_13 = R(0,2);

    const T pc_x = r_11*x + r_12*y + r_13*z + t_x[0];
    const T pc_y = R(1,0)*x + R(1,1)*y + R(1,2)*z + t_y[0];
    const T pc_z = R(2,0)*x + R(2,1)*y + R(2,2)*z + t_z[0];

    const T up = f_x*pc_x + cx*pc_z;
    const T vp = f_y*pc_y + cy*pc_z;
    const T s = pc_z;

   residuals[0] = u - up/s;
   residuals[1] = v - vp/s;
     

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.  
  static ceres::CostFunction* Create(const double u, const double v,const double x,const double y,const double z,arma::mat::fixed<3,3>  R , const parameters &par) {
    return (new ceres::AutoDiffCostFunction<
            CostFunctionPNPr2, 2, 1,1,1>(
                new CostFunctionPNPr2(u,v,x,y,z,R,par)));
  }

  
     double u;
     double v;
     double x;
     double y;
     double z;
     const arma::mat::fixed<3,3>  R;     
     parameters par;
};

//--------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// estimates x,y,z, and R camera position
struct CostFunctionPNPr3 {  
  CostFunctionPNPr3(const double u, const double v,const double x,const double y,const double z, const parameters &par) : u(v), v(v),x(x),y(y),z(z),par(par) {}

  template <typename T>
  bool operator()(const T* const t_x,
                  const T* const t_y,
                  const T* const t_z,
                  const T* const R,                  
                  T* residuals) const {
    
    const double f_x = par.img.cam_parameters.fc[0];
    const double f_y = par.img.cam_parameters.fc[1];
    const double cx = par.img.cam_parameters.cc[0];
    const double cy = par.img.cam_parameters.cc[1];
    
    //double r_11 = R(0,0);
    //double r_12 = R(0,1);
    //double r_13 = R(0,2);

    const T pc_x = R[0]*x + R[3]*y + R[6]*z + t_x[0];
    const T pc_y = R[1]*x + R[4]*y + R[7]*z + t_y[0];
    const T pc_z = R[2]*x + R[5]*y + R[8]*z + t_z[0];

    const T up = f_x*pc_x + cx*pc_z;
    const T vp = f_y*pc_y + cy*pc_z;
    const T s = pc_z;

   residuals[0] = u - up/s;
   residuals[1] = v - vp/s;
     

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.  
  static ceres::CostFunction* Create(const double u, const double v,const double x,const double y,const double z , const parameters &par) {
    return (new ceres::AutoDiffCostFunction<
            CostFunctionPNPr3, 2, 1,1,1,9>(
                new CostFunctionPNPr3(u,v,x,y,z,par)));
  }

  
     double u;
     double v;
     double x;
     double y;
     double z;         
     parameters par;
};







 #endif