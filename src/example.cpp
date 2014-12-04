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
  vector<Mat> leftImages;
  vector<Mat> rightImages;

  loadStereoImg(leftImages, rightImages);

  for (unsigned int i=0; i<leftImages.size(); i++) {
    imshow("output image", leftImages[i]);
    imshow("output image2", rightImages[i]);
  }

  waitKey();
  return 0;
}
