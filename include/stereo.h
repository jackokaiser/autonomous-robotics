#ifndef STEREO_DOT_H
#define STEREO_DOT_H


#include <opencv2/opencv.hpp>
#include <vector>

#include "utils.h"

void loadStereoImg (std::vector<cv::Mat>& leftImages, std::vector<cv::Mat>& rightImages);

void detectCartesian (const cv::Mat& disparityMap, cv::Mat& filteredDisparityMap, float tooCloseThreshold=0.2, float tooHighThreshold=2.5);

void computeVDisparity (const cv::Mat& disparityMap, cv::Mat& outputVDisparityMap);

void disparitySeg ();
void cartesianSeg ();
float lineY(cv::Point2f &np, cv::Point2f &p0, float X);
float lineD(cv::Point2f &np, cv::Point2f &p0, cv::Point2f &p1);
void fitLineRansac(const std::vector<cv::Point2f> points, cv::Vec4f &line, int iterations, double sigma, double a_max);


#endif
