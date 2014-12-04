#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>

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

void initializeDisparityMaps (vector<Mat>& disparityMap, const Size& size, unsigned int numberOfImages) {
  for (unsigned int i=0; i < numberOfImages; i++) {
    disparityMap.push_back(Mat(size, CV_16UC1));
  }
}

int main(int argc, char **argv)
{
  cout<<"OpenCV version: "<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION<<endl;
  vector<Mat> leftImages;
  vector<Mat> rightImages;
  Mat displayImg;

  loadStereoImg(leftImages, rightImages);
  unsigned int numberOfImages = leftImages.size();

  // initialize the disparityMap with empty images of the right size
  vector<Mat> disparityMap;
  initializeDisparityMaps(disparityMap, leftImages[0].size(), numberOfImages);

  // compute disparityMaps
  StereoSGBM sgbm = StereoSGBM(0, 32, 7, 8*7*7, 32*7*7, 2, 0, 5, 100, 32, true);
  for (unsigned int i=0; i < numberOfImages; i++) {
    sgbm( leftImages[i], rightImages[i], disparityMap[i]);
    disparityMap[i].convertTo(displayImg, CV_8UC1);
    imshow("disparity map", displayImg);
    waitKey();
  };



  return 0;
}
