#ifndef STEREO_DOT_H
#define STEREO_DOT_H


#include <opencv2/opencv.hpp>
#include <vector>

#include "utils.h"

void loadStereoImg (std::vector<cv::Mat>& leftImages, std::vector<cv::Mat>& rightImages);

void filterOutDisparity (const cv::Mat& disparityMap, cv::Mat& filteredDisparityMap, float tooCloseThreshold=0.2, float tooHighThreshold=2.5);

void computeVDisparity (const cv::Mat& disparityMap, cv::Mat& outputVDisparityMap);

void disparitySeg ();
void cartesianSeg ();

#endif
