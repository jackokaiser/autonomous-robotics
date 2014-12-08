#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>

#include "stereo.h"

using namespace cv;
using namespace std;

void loadStereoImg (vector<Mat>& leftImages, vector<Mat>& rightImages) {
  const unsigned int numberTotalImg = 3;
  string imgNumbers[] = {"46", "89", "250"};
  string path = "data/imgStereo/";

  for (unsigned int i=0; i < numberTotalImg; i++) {
    leftImages.push_back(imread(path + "left_" + imgNumbers[i] + ".png", 0));
    rightImages.push_back(imread(path + "right_" + imgNumbers[i] + ".png", 0));
  }
}

void filterOutDisparity (const Mat& disparityMap, Mat& filteredDisparityMap, float tooCloseThreshold, float tooHighThreshold) {
  disparityMap.convertTo(filteredDisparityMap, CV_32F);

  Point3f pixelInWorld;
  float* linePointer;
  float disparityMapValue;

  for(int i = 0; i < filteredDisparityMap.rows; i++) {
    linePointer = filteredDisparityMap.ptr<float>(i);
    for (int j = 0; j < filteredDisparityMap.cols; j++) {
      disparityMapValue = linePointer[j] / 16.;

      pixelInWorld.x = (j - INTRINSIC_U0) * STEREO_BASELINE / disparityMapValue - STEREO_BASELINE / 2.;
      pixelInWorld.y = INTRINSIC_ALPHA_U * STEREO_BASELINE / disparityMapValue;
      pixelInWorld.z = CAMERA_HEIGHT - (i - INTRINSIC_V0) * INTRINSIC_ALPHA_U * STEREO_BASELINE / (INTRINSIC_ALPHAU_V * disparityMapValue);

      if ((pixelInWorld.z < tooCloseThreshold) || (pixelInWorld.z > tooHighThreshold)) {
        linePointer[j] = 0.;
      }
    }
  }
}

void computeVDisparity (const Mat& disparityMap, Mat& outputVDisparityMap) {
  double maxDisparity;
  int scaleDownFactor = 1;

  minMaxLoc(disparityMap, NULL, &maxDisparity, NULL, NULL);
  int maxDisparityValue = max<int>(32,(int)maxDisparity / scaleDownFactor);

  outputVDisparityMap = Mat(Size(maxDisparityValue, disparityMap.rows),
                            CV_8UC1);

  const short* linePointer;
  short disparityMapValue;
  for(int i = 0; i < disparityMap.rows; i++) {
    linePointer = disparityMap.ptr<short>(i);
    for (int j = 0; j < disparityMap.cols; j++) {
      disparityMapValue = linePointer[j] / scaleDownFactor;
      outputVDisparityMap.at<char>(i,(int)disparityMapValue)++;
    }
  }
}
