#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>

#include "stereo.h"
#include "utils.h"

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


/*returns the distance from the point p1 to the line defined by normalized point-np and arbitrary point p0*/
float lineD(Point2f &np, Point2f &p0, Point2f &p1)
{
  Point2f v = p1-p0;
  return v.y*np.x - v.x*np.y;
}

/*computes the equation of a line as y = mx + b */
float lineY(Point2f &np, Point2f &p0, float X)
{
  float Y = np.y*(X-p0.x)/np.x + p0.y;
  return Y;
}

void cartesianFiltering (const Mat& disparityMap, Mat& filteredDisparityMap, float tooCloseThreshold, float tooHighThreshold) {
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
      pixelInWorld.z = CAMERA_HEIGHT - (i - INTRINSIC_V0) * INTRINSIC_ALPHA_U * STEREO_BASELINE / (INTRINSIC_ALPHA_V * disparityMapValue);

      if ((pixelInWorld.z < tooCloseThreshold) || (pixelInWorld.z > tooHighThreshold)) {
        linePointer[j] = 0.;
      }
    }
  }
}

void disparityFiltering (const Mat& disparityMap, Mat& filteredDisparityMap) {
  int maxDisparityValue = 32;
  int scaleDownFactor = 16;
  filteredDisparityMap = disparityMap.clone();

  Mat vDisparityMap = Mat::zeros(disparityMap.rows, maxDisparityValue, CV_8UC1);

  const short* linePointer;
  short disparityMapValue;
  for(int i = 0; i < disparityMap.rows; i++) {
    linePointer = disparityMap.ptr<short>(i);
    for (int j = 0; j < disparityMap.cols; j++) {
      disparityMapValue = linePointer[j] / scaleDownFactor;
      if (disparityMapValue > 0.) {
        vDisparityMap.at<unsigned char>(i,(int)disparityMapValue)++;
      }
    }
  }
  threshold(vDisparityMap,vDisparityMap, 60.0, THRESH_BINARY, THRESH_TOZERO);

  vector<Point2f> candidatesRansac;

  for(int i = 0; i < vDisparityMap.rows; i++) {
    for (int j = 0; j < vDisparityMap.cols; j++) {
      if(vDisparityMap.at<unsigned char>(i,j) > 0) {
        candidatesRansac.push_back(Point2f(j,i));
      }
    }
  }
  Vec4f line;
  int iterations = 1000;
  double sigma = 1.0;
  double a_max = 7.0;

  //apply Ransac
  fitLineRansac(candidatesRansac, line, iterations, sigma, a_max);
  Point2f lnp;
  Point2f lp0;
  lnp.x = line[0];
  lnp.y = line[1];
  lp0.x = line[2];
  lp0.y = line[3];

  for(int v = 0; v < disparityMap.rows; v++)
    {
      for(int u = 0; u < disparityMap.cols; u++)
        {
          float value = (float)disparityMap.at<short>(v,u)/16.;
          Point2f thePoint(value,(float)v);
          float d = lineD(lnp,lp0, thePoint);
          float epsilon = 2.5;
          if((d > -epsilon) || (d < -6.))
            {
              filteredDisparityMap.at<short>(v,u) = 0;
            }
        }
    }
}

void clustering(const Mat& disparityMapFiltered, Mat& imgLeft, Mat& imgRight) {
  //apply the erosion operation
  Mat erosion_img = Mat::zeros(disparityMapFiltered.size(), CV_32F);
  Mat dilation_img = Mat::zeros(disparityMapFiltered.size(), CV_32F);
  int erosion_type = MORPH_ELLIPSE;
  int erosion_size = 4;
  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
  erode(disparityMapFiltered, erosion_img, element);
  //apply the dilation operation
  dilate(erosion_img, dilation_img, element);
  //segment the image
  Mat segmented_img(disparityMapFiltered.size(), CV_8UC1);
  int nbObjects = segmentDisparity(dilation_img, segmented_img);
  //imshow("Segmented Disparity 1", segmented_img);

  //printf("Max label 1: %d \n", nbObjects);

  vector< vector<int> > objects(nbObjects);
  for (int i=0; i<nbObjects; i++)
    {
      objects[i].resize(6);
      objects[i][0] = 1e6; //vmin
      objects[i][1] = -1; //vmax
      objects[i][2] = 1e6; //umin
      objects[i][3] = -1; //umax
      objects[i][4] = 0; //mean disparity
      objects[i][5] = 0; //size
    }

  for(int v = 0; v < disparityMapFiltered.rows; v++)
    {

      for(int u = 0; u < disparityMapFiltered.cols; u++)
        {
          int label = segmented_img.at<unsigned int>(v, u);
          if (v<objects[label][0]) objects[label][0] = v;
          if (v>objects[label][1]) objects[label][1] = v;
          if (u<objects[label][2]) objects[label][2] = u;
          if (u>objects[label][3]) objects[label][3] = u;
          objects[label][4] += disparityMapFiltered.at<float>(v,u)/16;
          objects[label][5]++;
        }
    }

  Mat imgLeftC;
  Mat imgRightC;
  cvtColor(imgLeft,imgLeftC,CV_GRAY2BGR);
  for (int i=1; i<nbObjects; i++)
    {
      if (objects[i][5]>50)
        {
          objects[i][4] /= objects[i][5];
          rectangle(imgLeftC,Point(objects[i][2],objects[i][0]),Point(objects[i][3],objects[i][1]),Scalar(0,255,0),1);
        }
    }
  cvtColor(imgRight,imgRightC,CV_GRAY2BGR);
  for (int i=1; i<nbObjects; i++)
    {
      if (objects[i][5]>50)
        {
          objects[i][4] /= objects[i][5];
          rectangle(imgRightC,Point(objects[i][2],objects[i][0]),Point(objects[i][3],objects[i][1]),Scalar(0,255,0),1);
        }
    }
  imgLeft = imgLeftC;
  imgRight = imgRightC;
}

void cartesianSeg () {
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

    cartesianFiltering(disparityMap, outputImg);

    clustering(outputImg, leftImages[i], rightImages[i]);

    leftImages[i].convertTo(displayImg, CV_8UC1);
    imshow("result", displayImg);
    waitKey();

    disparityMap.release();
    displayImg.release();
    outputImg.release();
  };
}

void disparitySeg () {
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

    disparityFiltering(disparityMap, outputImg);
    clustering(outputImg, leftImages[i], rightImages[i]);

    leftImages[i].convertTo(displayImg, CV_8UC1);
    imshow("result", displayImg);
    outputImg.convertTo(displayImg, CV_8UC1);
    waitKey();
    imshow("result", displayImg);
    waitKey();

    disparityMap.release();
    displayImg.release();
    outputImg.release();
  };
}
