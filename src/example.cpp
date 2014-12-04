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

int main(int argc, char **argv)
{
  cout<<"OpenCV version: "<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION<<endl;
  vector<Mat> leftImages;
  vector<Mat> rightImages;

  loadStereoImg(leftImages, rightImages);

  for (unsigned int i=0; i<leftImages.size(); i++) {
    imshow("output image", leftImages[i]);
    imshow("output image2", rightImages[i]);
  }
  vector<Mat> disparityMap;
  StereoSGBM sgbm = StereoSGBM(0, 32, 7, 8*7*7, 32*7*7, 2, 0, 5, 100, 32, true);
  sgbm( leftImages, rightImages, disparityMap);

  waitKey();
  return 0;
}
