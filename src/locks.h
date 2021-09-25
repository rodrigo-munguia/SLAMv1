#ifndef LOCKS_H
#define LOCKS_H


#include <mutex>


struct LOCKS
{
    std::mutex ekf_run_mtx;
    std::mutex add_keyframe_mtx;
    std::mutex add_keyframe_mtx_CL;
    std::mutex NewEKFAnchorsDATA_mtx;
    std::mutex Pull_NewAnchorsDATA_mtx;
    std::mutex update_anchors_mtx;
    std::mutex ReadGlobalMAP_mtx;

};

















#endif 