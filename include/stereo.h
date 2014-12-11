#ifndef STEREO_DOT_H
#define STEREO_DOT_H


#include <opencv2/opencv.hpp>
#include <vector>

#include "utils.h"

#define CARTESIAN_SPACE 1<<0
#define DISPARITY_SPACE 1<<1

void cartesianFiltering (const cv::Mat& disparityMap, cv::Mat& filteredDisparityMap, float tooCloseThreshold=0.2, float tooHighThreshold=2.5);

void disparityFiltering (const cv::Mat& disparityMap, cv::Mat& outputVDisparityMap);

void stereoDisparity (unsigned char flags);

#endif
