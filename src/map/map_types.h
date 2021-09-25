#ifndef MAP_TYPES_H
#define MAP_TYPES_H

#include "opencv2/opencv.hpp"
#include <armadillo>

//------  Map feature structure

using namespace std;


struct GSTORE
{
    // for plotting
    cv::Mat GLOBALmap;
    cv::Mat GLOBALmap_color;    
    std::vector<cv::Affine3d> path; 
};    


struct KEYFRAME
{   

    cv::Mat frame; // image frame
    arma::vec::fixed<4> CameraAtt; // Attitude state of the camera for the i Keyframe
    arma::vec::fixed<3> CameraPos; // Position State of the camera for the i Keyframe
    vector<cv::KeyPoint> keyPoints; // Keypoints associated with the KeyFrame
    cv::Mat Descriptors;   //Descriptors of the Keypoints
    vector<int> PredictedFeats_idx; // vector of index of features (anchors) predicted to appear in the keyframe
    vector<int> MatchedFeats_idx; // vector of index of features (anchors) matched in the keyframe

};

typedef std::vector<KEYFRAME> vectorKeyF;


#endif