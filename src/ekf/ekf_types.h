#ifndef EKF_TYPES_H
#define EKF_TYPES_H

#include "opencv2/opencv.hpp"
#include <armadillo>
#include "../getData.h"

//------  Map feature structure

using namespace std;



struct KeyFramesData
 {
     int KeyF_idx;  // index of Keyframes associated with the Mapfeature
     bool matched;
     cv::Point2f PredictedPoint; // Predicted pixel position of the feature
     cv::Point2f MatchedPoint;  // Matched (measured) position of the feature
 };

 struct MatchInfo
{
    int n_predicted_img_feats;
    int n_matched_feats;
    int n_predicted_img_anchors;
    int n_matched_anchors;
}; 

struct FEAT
{
    string feat_type; // Feature tupe: euclidean (XYZ): ID, etc
    int idx_i_state;  // initial index of the feature in the system state
    int idx_f_state;  // final index of the feature in the system state
    cv::KeyPoint Initial_KeyPoint; //  feature Keypoint when it was first detected
    cv::KeyPoint Keypoint; // current feature Keypoint (include actual matched pixel position of the feature)
    cv::Mat Descriptor;  //  Keypoint descriptor
    int times_mathed;
    int times_not_mathed;
    int times_not_considered;
    bool predicted; // the image is predicted to "appear" in the current image
    bool matched; // the feature was matched in the current frame ? 
    arma::mat::fixed<2,2> Si;  // Innovation matrix at step i
    arma::mat::fixed<2,13> duv_dx; // Jacobian respect to camera/robot state
    arma::mat::fixed<2,3> duv_dy; // Jacobian respect to feature
    cv::Point2f PredictedPoint; // Predicted pixel position of the feature
    cv::Point2f MatchedPoint; // Actual pixel position matched fo the feature
    arma::vec::fixed<3> CameraState; // State of the camera at the moment that the feature was initialized
    arma::vec::fixed<3> AnchorState; //
    int init_KF; //  index of the KeyFrame  where the Feature-anchor was initialized
    std::vector<KeyFramesData> iKeyFrame; // vector for storing data for the i-KeyFrame
    int init_type; // 0 -> Initialized by EKF 
                   // 1 -> Initialized by the global map process
    int ekf_to_gmap_status; // 0-> Not updated to the global map (for anchors initialized by the EKF)
                            // 1-> Included in the global map
    int id_anchor; // Unique anchor identifier                        


};

typedef std::vector<FEAT> vectorFeat;

struct STORE
{
    // for plotting
    cv::Mat EKFmap;
    cv::Mat EKFmap_color;
    cv::Mat ANCHORSmap;
    cv::Mat ANCHORSmap_color;

    std::vector<cv::Point3f> EKFtrajectory;
    std::vector<cv::Affine3d> pose; 

    std::vector<double>  x_gps, y_gps,z_gps;

    FRAME current_frame;

    vectorFeat *FeatsDATA;

    vectorFeat *AnchorsDATA;
        //------------------

};    







#endif