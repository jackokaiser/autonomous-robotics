#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  if(argc != 2) {
    cerr << "Usage : " << argv[0] << " image" << endl;
    return 0;
  }

  // Open image from input file in grayscale
  Mat img = imread(argv[1], 0);

  // Create the output image in 8 bits grayscale format
  Mat output_img(img.size(), CV_8UC1);

  // We explore and each pixel and invert the gray level
  for(int i=0; i<img.rows; i++) {
    const unsigned char *input = img.ptr<unsigned char>(i);
    unsigned char *output = output_img.ptr<unsigned char>(i);
    for(int j=0; j<img.cols; j++) {
      output[j] = 255 - input[j];
    }
  }

  //  We change the value of the pixel (200,100)
  output_img.at<unsigned char>(100,200) = 255;
    // warning: here the coordinates order is (row number, column number);

  // Display images and wait for a key press
  imshow("input image", img);
  imshow("output image", output_img);
  waitKey();
  return 0;
}
