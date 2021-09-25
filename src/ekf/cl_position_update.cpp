#include "cl_position_update.h"

void cl_position_update2(arma::vec& x, arma::mat& P,arma::vec::fixed<2> xy_pos,parameters &par,vectorFeat &FeatsDATA,vectorFeat &AnchorsDATA)
{
 
   double dx = xy_pos(0) - x(7);
   double dy = xy_pos(1) - x(8);
   
   // update camera state
   x(7) = x(7) + dx;
   x(8) = x(8) + dy;

   for(int i = 0; i< FeatsDATA.size();i++ )    {

            int idx_i = FeatsDATA[i].idx_i_state;
            
            x(idx_i) = x(idx_i) + dx;
            x(idx_i+1) = x(idx_i+1) + dy;

    }
    for(int i = 0; i< AnchorsDATA.size();i++ )    {

            
            AnchorsDATA[i].AnchorState(0) = AnchorsDATA[i].AnchorState(0) + dx;
            AnchorsDATA[i].AnchorState(1) = AnchorsDATA[i].AnchorState(1) + dy;
            

    }  

   int q = 10;
  


}



void cl_position_update(arma::vec& x, arma::mat& P,arma::vec::fixed<2> xy_pos,parameters &par)
{

        int x_len = x.size();
        arma::mat H;
        // Form jacobian
        H.resize(2,x_len);
        H(0,7) = 1;
        H(1,8) = 1;  

       // H.print();
        
        //double h = x(9);
        arma::vec::fixed<2> h;
        h(0) = x(7);
        h(1) = x(8);
        
        double r = par.ekf.sigma_cl;
        arma::mat::fixed<2,2> R;
        R(0,0) = r*r;
        R(1,1) = r*r;
        

        arma::mat H_P = H*P;

        arma::mat S = H_P*H.t() + R; // Innovation matrix
        //arma::mat K = P*H.t()*arma::inv_sympd(S);
       arma::mat K = P*H.t()*arma::inv(S); // Kalman gain

        P = P - K*H_P;  // System Covariance matrix update 
 
        x = x + K*(xy_pos-h);  // System vector update


  int q = 10;


}    