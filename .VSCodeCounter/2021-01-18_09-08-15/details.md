# Details

Date : 2021-01-18 09:08:15

Directory /home/rodrigo/Dropbox/RESEARCH/RTSLAM/SLAMv1

Total : 66 files,  7783 codes, 2045 comments, 2987 blanks, all 12815 lines

[summary](results.md)

## Files
| filename | language | code | comment | blank | total |
| :--- | :--- | ---: | ---: | ---: | ---: |
| [Makefile](/Makefile) | Makefile | 55 | 44 | 49 | 148 |
| [README.md](/README.md) | Markdown | 26 | 0 | 21 | 47 |
| [src/Jacs/JAC_XYZ_uvr.cpp](/src/Jacs/JAC_XYZ_uvr.cpp) | C++ | 41 | 34 | 31 | 106 |
| [src/Jacs/JacSystemPredictionV3b.cpp](/src/Jacs/JacSystemPredictionV3b.cpp) | C++ | 252 | 90 | 24 | 366 |
| [src/Jacs/JacSystemPredictionV3b.h](/src/Jacs/JacSystemPredictionV3b.h) | C++ | 7 | 18 | 5 | 30 |
| [src/Jacs/Jac_XYZ_uvr.h](/src/Jacs/Jac_XYZ_uvr.h) | C++ | 10 | 6 | 12 | 28 |
| [src/Jacs/Jac_uv_XYZ.h](/src/Jacs/Jac_uv_XYZ.h) | C++ | 10 | 6 | 13 | 29 |
| [src/Jacs/Jav_uv_XYZ.cpp](/src/Jacs/Jav_uv_XYZ.cpp) | C++ | 40 | 8 | 16 | 64 |
| [src/Transforms/Euler_to_Ra2b.cpp](/src/Transforms/Euler_to_Ra2b.cpp) | C++ | 28 | 37 | 6 | 71 |
| [src/Transforms/Euler_to_Ra2b.h](/src/Transforms/Euler_to_Ra2b.h) | C++ | 6 | 18 | 5 | 29 |
| [src/Transforms/Geo2ECEF.cpp](/src/Transforms/Geo2ECEF.cpp) | C++ | 21 | 4 | 14 | 39 |
| [src/Transforms/Geo2ECEF.h](/src/Transforms/Geo2ECEF.h) | C++ | 4 | 3 | 6 | 13 |
| [src/Transforms/Makefile](/src/Transforms/Makefile) | Makefile | 9 | 0 | 9 | 18 |
| [src/Transforms/Ra2b_TO_Quat_a2b.cpp](/src/Transforms/Ra2b_TO_Quat_a2b.cpp) | C++ | 11 | 24 | 6 | 41 |
| [src/Transforms/Ra2b_TO_Quat_a2b.h](/src/Transforms/Ra2b_TO_Quat_a2b.h) | C++ | 6 | 18 | 5 | 29 |
| [src/Transforms/quat2R.cpp](/src/Transforms/quat2R.cpp) | C++ | 84 | 26 | 12 | 122 |
| [src/Transforms/quat2R.h](/src/Transforms/quat2R.h) | C++ | 6 | 18 | 5 | 29 |
| [src/anms/anms.cpp](/src/anms/anms.cpp) | C++ | 262 | 7 | 54 | 323 |
| [src/anms/anms.h](/src/anms/anms.h) | C++ | 44 | 8 | 24 | 76 |
| [src/anms/nanoflann.hpp](/src/anms/nanoflann.hpp) | C++ | 816 | 401 | 182 | 1,399 |
| [src/anms/range-tree/lrtypes.h](/src/anms/range-tree/lrtypes.h) | C++ | 12 | 0 | 5 | 17 |
| [src/anms/range-tree/ranget.h](/src/anms/range-tree/ranget.h) | C++ | 485 | 62 | 166 | 713 |
| [src/ekf/altitude_update.cpp](/src/ekf/altitude_update.cpp) | C++ | 39 | 4 | 18 | 61 |
| [src/ekf/altitude_update.h](/src/ekf/altitude_update.h) | C++ | 7 | 4 | 9 | 20 |
| [src/ekf/cl_position_update.cpp](/src/ekf/cl_position_update.cpp) | C++ | 22 | 4 | 15 | 41 |
| [src/ekf/cl_position_update.h](/src/ekf/cl_position_update.h) | C++ | 6 | 4 | 10 | 20 |
| [src/ekf/ekf.cpp](/src/ekf/ekf.cpp) | C++ | 68 | 2 | 38 | 108 |
| [src/ekf/ekf.h](/src/ekf/ekf.h) | C++ | 49 | 0 | 35 | 84 |
| [src/ekf/ekf_types.h](/src/ekf/ekf_types.h) | C++ | 40 | 3 | 10 | 53 |
| [src/ekf/prediction.cpp](/src/ekf/prediction.cpp) | C++ | 59 | 51 | 36 | 146 |
| [src/ekf/prediction.h](/src/ekf/prediction.h) | C++ | 8 | 4 | 10 | 22 |
| [src/ekf/system_init.cpp](/src/ekf/system_init.cpp) | C++ | 29 | 17 | 15 | 61 |
| [src/ekf/system_init.h](/src/ekf/system_init.h) | C++ | 9 | 4 | 11 | 24 |
| [src/ekf/visual_delete_feats.cpp](/src/ekf/visual_delete_feats.cpp) | C++ | 49 | 8 | 41 | 98 |
| [src/ekf/visual_delete_feats.h](/src/ekf/visual_delete_feats.h) | C++ | 11 | 4 | 11 | 26 |
| [src/ekf/visual_init_anchors.cpp](/src/ekf/visual_init_anchors.cpp) | C++ | 67 | 7 | 52 | 126 |
| [src/ekf/visual_init_anchors.h](/src/ekf/visual_init_anchors.h) | C++ | 12 | 4 | 8 | 24 |
| [src/ekf/visual_init_w_range.cpp](/src/ekf/visual_init_w_range.cpp) | C++ | 145 | 83 | 82 | 310 |
| [src/ekf/visual_init_w_range.h](/src/ekf/visual_init_w_range.h) | C++ | 14 | 4 | 17 | 35 |
| [src/ekf/visual_match_feats.cpp](/src/ekf/visual_match_feats.cpp) | C++ | 211 | 68 | 84 | 363 |
| [src/ekf/visual_match_feats.h](/src/ekf/visual_match_feats.h) | C++ | 21 | 4 | 14 | 39 |
| [src/ekf/visual_update_f.cpp](/src/ekf/visual_update_f.cpp) | C++ | 104 | 22 | 64 | 190 |
| [src/ekf/visual_update_f.h](/src/ekf/visual_update_f.h) | C++ | 14 | 4 | 15 | 33 |
| [src/getData.cpp](/src/getData.cpp) | C++ | 141 | 17 | 41 | 199 |
| [src/getData.h](/src/getData.h) | C++ | 46 | 11 | 16 | 73 |
| [src/locks.h](/src/locks.h) | C++ | 12 | 0 | 22 | 34 |
| [src/loop/cost_function_cl.h](/src/loop/cost_function_cl.h) | C++ | 122 | 19 | 55 | 196 |
| [src/loop/loop.cpp](/src/loop/loop.cpp) | C++ | 377 | 169 | 155 | 701 |
| [src/loop/loop.h](/src/loop/loop.h) | C++ | 32 | 0 | 33 | 65 |
| [src/main.cpp](/src/main.cpp) | C++ | 182 | 39 | 86 | 307 |
| [src/map/Add_KeyFrames.cpp](/src/map/Add_KeyFrames.cpp) | C++ | 279 | 125 | 103 | 507 |
| [src/map/Add_KeyFrames2.cpp](/src/map/Add_KeyFrames2.cpp) | C++ | 258 | 40 | 104 | 402 |
| [src/map/cost_function.h](/src/map/cost_function.h) | C++ | 172 | 48 | 66 | 286 |
| [src/map/delete_ak.cpp](/src/map/delete_ak.cpp) | C++ | 85 | 3 | 34 | 122 |
| [src/map/local_bundle_ajustment.cpp](/src/map/local_bundle_ajustment.cpp) | C++ | 194 | 50 | 65 | 309 |
| [src/map/local_bundle_ajustment2.cpp](/src/map/local_bundle_ajustment2.cpp) | C++ | 201 | 52 | 60 | 313 |
| [src/map/map.cpp](/src/map/map.cpp) | C++ | 63 | 8 | 24 | 95 |
| [src/map/map.h](/src/map/map.h) | C++ | 49 | 1 | 56 | 106 |
| [src/map/map_types.h](/src/map/map_types.h) | C++ | 17 | 1 | 9 | 27 |
| [src/map/visual_match.cpp](/src/map/visual_match.cpp) | C++ | 157 | 74 | 67 | 298 |
| [src/matplotlib/matplotlibcpp.h](/src/matplotlib/matplotlibcpp.h) | C++ | 1,848 | 120 | 587 | 2,555 |
| [src/matplotlib/numpy_flags.py](/src/matplotlib/numpy_flags.py) | Python | 7 | 2 | 4 | 13 |
| [src/parameters.cpp](/src/parameters.cpp) | C++ | 70 | 10 | 20 | 100 |
| [src/parameters.h](/src/parameters.h) | C++ | 101 | 11 | 31 | 143 |
| [src/vision/vision.cpp](/src/vision/vision.cpp) | C++ | 139 | 94 | 73 | 306 |
| [src/vision/vision.h](/src/vision/vision.h) | C++ | 12 | 14 | 11 | 37 |

[summary](results.md)