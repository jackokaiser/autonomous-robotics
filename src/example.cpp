#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>

#include "stereo.h"

using namespace cv;
using namespace std;


int main(int argc, char **argv)
{
  cout<<"OpenCV version: "<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION<<endl;
  vector<Mat> leftImages;
  vector<Mat> rightImages;
  Mat disparityMap;
  Mat outputImg;
  Mat displayImg;
  loadStereoImg(leftImages, rightImages);
  unsigned int numberOfImages = leftImages.size();

  StereoSGBM sgbm = StereoSGBM(0, 32, 7, 8*7*7, 32*7*7, 2, 0, 5, 100, 32, true);

  for (unsigned int i=0; i < numberOfImages; i++) {

    sgbm( leftImages[i], rightImages[i], disparityMap );

    computeVDisparity(disparityMap, outputImg);
    // filterOutDisparity(disparityMap, outputImg);

    outputImg.convertTo(displayImg, CV_8UC1);
    imshow("result", displayImg);
    waitKey();

    disparityMap.release();
    displayImg.release();
    outputImg.release();
  };

  return 0;
}
