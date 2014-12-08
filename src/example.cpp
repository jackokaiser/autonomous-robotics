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

void filterOutDisparity (const Mat& disparityMap, Mat& filteredDisparityMap, float tooCloseThreshold=0.2, float tooHighThreshold=2.5) {
  filteredDisparityMap.release();
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
  Mat scaledDownDisparityMap = Mat(disparityMap);
  scaledDownDisparityMap = scaledDownDisparityMap / 16;

  minMaxLoc(scaledDownDisparityMap, NULL, &maxDisparity, NULL, NULL);
  int maxDisparityValue = max<int>(32,maxDisparity);

  outputVDisparityMap.release();
  outputVDisparityMap = Mat(Size(maxDisparityValue, scaledDownDisparityMap.rows),
                            CV_8UC1);

  const short* linePointer;
  short disparityMapValue;
  for(int i = 0; i < scaledDownDisparityMap.rows; i++) {
    linePointer = scaledDownDisparityMap.ptr<short>(i);
    for (int j = 0; j < scaledDownDisparityMap.cols; j++) {
      disparityMapValue = linePointer[j];
      char& vDisparity = outputVDisparityMap.at<char>(i,(int)disparityMapValue);
      vDisparity++;
    }
  }
}

void displayImages (const vector<Mat>& images) {
  Mat displayImg;
  for (unsigned int i=0; i < images.size(); i++) {
    images[i].convertTo(displayImg, CV_8UC1);
    imshow("image", displayImg);
    waitKey();
  }
}

int main(int argc, char **argv)
{
  cout<<"OpenCV version: "<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION<<endl;
  vector<Mat> leftImages;
  vector<Mat> rightImages;
  vector<Mat> vDisparityMaps;
  Mat outputImg;
  loadStereoImg(leftImages, rightImages);
  unsigned int numberOfImages = leftImages.size();

  // initialize the disparityMap with empty images of the right size
  vector<Mat> disparityMaps;

  initializeDisparityMaps(disparityMaps, leftImages[0].size(), numberOfImages);

  StereoSGBM sgbm = StereoSGBM(0, 32, 7, 8*7*7, 32*7*7, 2, 0, 5, 100, 32, true);
  for (unsigned int i=0; i < numberOfImages; i++) {
    sgbm( leftImages[i], rightImages[i], disparityMaps[i]);
    computeVDisparity(disparityMaps[i], outputImg);
    // filterOutDisparity(disparityMaps[i], outputImg);
    vDisparityMaps.push_back(Mat(outputImg));


  };

  displayImages(vDisparityMaps);



  return 0;
}
