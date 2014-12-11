#ifndef PROJECT_UTILS_DOT_H
#define PROJECT_UTILS_DOT_H

//  Camera
#define INTRINSIC_U0 258
#define INTRINSIC_V0 156
#define INTRINSIC_ALPHA_U 410
#define INTRINSIC_ALPHA_V 410
#define STEREO_BASELINE 0.22
#define CAMERA_TY 1.8
#define CAMERA_HEIGHT 1.28

//  Lidar
#define LIDAR_PITCH_ANGLE (-1.*M_PI/180)
#define LIDAR_HEIGHT (0.47)

/*returns a 8bit Grey image where each pixel represent the index of the object (0 corresponds to the background*/
unsigned int segmentDisparity(const cv::Mat &disparity, cv::Mat &output);

#endif
