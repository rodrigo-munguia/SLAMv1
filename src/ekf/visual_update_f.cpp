#include "visual_update_f.h"
#include "../Jacs/Jac_uv_XYZ.h"
#include <math.h>



//-----------------------------------------------------------------------------
//   Update EKF with visual measurements
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  


void visual_update_f(arma::vec& x, arma::mat& P, parameters &par, vectorFeat& FeatsDATA, vectorFeat& AnchorsDATA )
{

    visual_update_anchors(x,P,par,AnchorsDATA ); // update with anchors

    visual_update_feats(x,P,par, FeatsDATA);  // update with feats


}




//----------------------------------------------------------------------------------------------

void visual_update_anchors(arma::vec& x, arma::mat& P, parameters &par, vectorFeat& AnchorsDATA )
{

    int n_anchors;
    int x_len = x.size();
    arma::mat H;
    arma::vec z;
    arma::vec h;
    arma::mat R;
    for(int i = 0; i< AnchorsDATA.size();i++)
    {
        if(AnchorsDATA[i].matched == true)
        {
            arma::vec::fixed<3> pxyz = AnchorsDATA[i].AnchorState;            
            arma::mat::fixed<2,13> duv_dx;
            arma::mat::fixed<2,3> duv_dy;
            Jac_uv_XYZ(pxyz,x,par,duv_dx,duv_dy);  // implement efficient JAcobian!!

            arma::mat hi;
            hi.zeros(2,x_len);
            hi(arma::span(0,1),arma::span(0,12)) = duv_dx; 

            int idx = H.n_rows;
            
            // Form jacobian
            H.resize(idx+2,x_len);
            H(arma::span(idx,idx+1),arma::span(0,x_len-1)) = hi;   
            
            // Form prediction vector
            h.resize(idx+2);  
            h(idx) = AnchorsDATA[i].PredictedPoint.x;
            h(idx+1) = AnchorsDATA[i].PredictedPoint.y;
            
            // Form measurement vector
            z.resize(idx+2);
            z(idx) = AnchorsDATA[i].MatchedPoint.x;
            z(idx+1) = AnchorsDATA[i].MatchedPoint.y;

            // Form measurement noise matrix

            R.resize(idx+2,idx+2);
            R(idx,idx) = pow(par.ekf.sigma_uv*2,2);
            R(idx+1,idx+1) = pow(par.ekf.sigma_uv*2,2); 



        }

    }

    //----------------------------------------------------------------
// Kalman Update
    int n = h.size();

    if(n>3)
    {
        arma::mat H_P = H*P;

        arma::mat S = H_P*H.t() + R; // Innovation matrix
        //arma::mat K = P*H.t()*arma::inv_sympd(S);
        //S.print();
       arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

        P = P - K*H_P;  // System Covariance matrix update 
 
        x = x + K*(z-h);  // System vector update


    }





}


//---------------------------------------------------------------------------------------------
void visual_update_feats(arma::vec& x, arma::mat& P, parameters &par, vectorFeat& FeatsDATA)
{
     int x_len = x.size();
    arma::mat H;
    arma::vec z;
    arma::vec h;
    arma::mat R;

    for(int i = 0; i< FeatsDATA.size();i++ )
    {
        if(FeatsDATA[i].matched == true)
        {

            int idx_i = FeatsDATA[i].idx_i_state;
            int idx_f = FeatsDATA[i].idx_f_state;

            arma::vec::fixed<3> pxyz = x.subvec(idx_i,idx_f);

            arma::mat::fixed<2,13> duv_dx;
            arma::mat::fixed<2,3> duv_dy;
            if (par.img.check_innovation_for_mathing == true) // In this case Jacobians have been already computed
            {
                duv_dx = FeatsDATA[i].duv_dx;
                duv_dy = FeatsDATA[i].duv_dy;
            }
            else
            {
                Jac_uv_XYZ(pxyz,x,par,duv_dx,duv_dy);    
            }   
            

            arma::mat hi;
            hi.zeros(2,x_len);
            hi(arma::span(0,1),arma::span(0,12)) = duv_dx;
            hi(arma::span(0,1),arma::span(idx_i,idx_f)) = duv_dy;        

            int idx = H.n_rows;
            
            // Form jacobian
            H.resize(idx+2,x_len);
            H(arma::span(idx,idx+1),arma::span(0,x_len-1)) = hi;   
            
            // Form prediction vector
            h.resize(idx+2);  
            h(idx) = FeatsDATA[i].PredictedPoint.x;
            h(idx+1) = FeatsDATA[i].PredictedPoint.y;
            
            // Form measurement vector
            z.resize(idx+2);
            z(idx) = FeatsDATA[i].MatchedPoint.x;
            z(idx+1) = FeatsDATA[i].MatchedPoint.y;

            // Form measurement noise matrix

            R.resize(idx+2,idx+2);
            R(idx,idx) = pow(par.ekf.sigma_uv,2);
            R(idx+1,idx+1) = pow(par.ekf.sigma_uv,2);
            
        
        }

    }
 //Hi(arma::span(0,24),arma::span(0,30)).print();

//----------------------------------------------------------------
// Kalman Update
    int n = h.size();

    if(n>0)
    {
        arma::mat H_P = H*P;

        arma::mat S = H_P*H.t() + R; // Innovation matrix
        //arma::mat K = P*H.t()*arma::inv_sympd(S);
       arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

        P = P - K*H_P;  // System Covariance matrix update 
 
        x = x + K*(z-h);  // System vector update


    }


}