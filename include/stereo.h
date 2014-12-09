#ifndef STEREO_DOT_H
#define STEREO_DOT_H

#define INTRINSIC_U0 258
#define INTRINSIC_V0 156
#define INTRINSIC_ALPHA_U 410
#define INTRINSIC_ALPHAU_V 410
#define STEREO_BASELINE 0.22
#define CAMERA_HEIGHT 1.128

#include <opencv2/opencv.hpp>
#include <vector>

void loadStereoImg (std::vector<cv::Mat>& leftImages, std::vector<cv::Mat>& rightImages);

void filterOutDisparity (const cv::Mat& disparityMap, cv::Mat& filteredDisparityMap, float tooCloseThreshold=0.2, float tooHighThreshold=2.5);

void computeVDisparity (const cv::Mat& disparityMap, cv::Mat& outputVDisparityMap);

void disparitySeg ();
void cartesianSeg ();

#endif
