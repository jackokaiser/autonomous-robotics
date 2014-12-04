#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>

#define INTRINSIC_U0 258
#define INTRINSIC_V0 156
#define INTRINSIC_ALPHA_U 410
#define INTRINSIC_ALPHAU_V 410
#define STEREO_BASELINE 0.22
#define CAMERA_HEIGHT 1.128

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

void initializeDisparityMaps (vector<Mat>& disparityMaps, const Size& size, unsigned int numberOfImages) {
  for (unsigned int i=0; i < numberOfImages; i++) {
    disparityMaps.push_back(Mat(size, CV_16UC1));
  }
}

void filterOutObstacles (Mat& disparityMap, float tooCloseThreshold=0.2, float tooHighThreshold=2.5) {
  Point3f pixelInWorld;
  float* linePointer;
  float disparityMapValue;

  for(int i = 0; i < disparityMap.rows; i++) {
    linePointer = disparityMap.ptr<float>(i);
    for (int j = 0; j < disparityMap.cols; j++) {
      disparityMapValue = linePointer[j];

      pixelInWorld.x = (i - INTRINSIC_U0) * STEREO_BASELINE / disparityMapValue - STEREO_BASELINE / 2.;
      pixelInWorld.y = INTRINSIC_ALPHA_U * STEREO_BASELINE / disparityMapValue;
      pixelInWorld.z = CAMERA_HEIGHT - (j - INTRINSIC_V0) * INTRINSIC_ALPHA_U * STEREO_BASELINE / (INTRINSIC_ALPHAU_V * disparityMapValue);

      if ((pixelInWorld.z < tooCloseThreshold) || (pixelInWorld.y > tooHighThreshold)) {
        linePointer[j] = 0.;
      }
    }
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
  vector<Mat> disparityMaps;
  initializeDisparityMaps(disparityMaps, leftImages[0].size(), numberOfImages);

  // compute disparityMaps
  StereoSGBM sgbm = StereoSGBM(0, 32, 7, 8*7*7, 32*7*7, 2, 0, 5, 100, 32, true);
  for (unsigned int i=0; i < numberOfImages; i++) {
    sgbm( leftImages[i], rightImages[i], disparityMaps[i]);

    disparityMaps[i].convertTo(disparityMaps[i], CV_32F);
    filterOutObstacles(disparityMaps[i]);


    disparityMaps[i].convertTo(displayImg, CV_8UC1);
    imshow("disparity map", displayImg);
    waitKey();
  };



  return 0;
}
