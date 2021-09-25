#ifndef SYS_VISUAL_match_I_H
#define SYS_VISUAL_match_I_H



#include "ekf_types.h"
#include <armadillo>
#include "../parameters.h"
#include "../getData.h"
#include "opencv2/opencv.hpp"
#include "../vision/vision.h"
#include "../map/map.h"
#include "../map/map_types.h"
#include "../loop/loop.h"
#include "../locks.h"
#include <chrono>  
#include <thread>  





//-----------------------------------------------------------------------------
//   Match visual features with range measurements
//   Rodrigo Munguia 2020
//-----------------------------------------------------------------------------  
using namespace arma;
typedef std::vector<double> vec_t;


MatchInfo visual_match_feats(arma::vec& x, arma::mat& P, parameters &par,FRAME *frame, vectorFeat& FeatsDATA ,vectorFeat& AnchorsDATA,GMAP &gmap,LOOP &cloop,LOCKS &locks );

void matchFeatures(const cv::Mat &query, const cv::Mat &target,std::vector<cv::DMatch> &goodMatches );

void add_key_frame_to_global_map(arma::vec& x,parameters &par,GMAP &gmap,LOCKS &locks,vector<cv::KeyPoint> &keyPoints,cv::Mat &Descriptors,std::vector<cv::DMatch> &Matches,vectorFeat& AnchorsDATA,cv::Mat &frame,vectorFeat& FeatsDATA);

void add_key_frame_to_close_loop(arma::vec& x,parameters &par,LOOP &cloop,LOCKS &locks,vector<cv::KeyPoint> &keyPoints,cv::Mat &Descriptors,std::vector<cv::DMatch> &Matches,vectorFeat& AnchorsDATA,cv::Mat &frame,vectorFeat& FeatsDATA);

#endif