#include <opencv2/opencv.hpp>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#include "readLidarData.h"

using namespace cv;
using namespace std;

Point2f computeMiddle (const Rect& r) {
  Point2f topLeft = r.tl();
  Point2f bottomRight = r.br();

  return Point2f((topLeft.x + bottomRight.x) / 2.,
                 (topLeft.y + bottomRight.y) / 2.);
}
bool validIndex(Mat img, Point2f p) {
  return ((p.x>=0 && p.x<img.cols) &&
    (p.y>=0 && p.y<img.rows));
}
void shiftRoi (Rect& roi, const Point2f& targetCenter) {
  Point2f currentCenter = computeMiddle(roi);
  Point2f shift = targetCenter - currentCenter;
  roi = roi + Point2i((int)shift.x, (int)shift.y);
}
void polygon (Mat& img, const vector<Point2f>& points, Scalar col) {
  unsigned int nbEdges = points.size();
  for (unsigned int i=0; i<nbEdges; i++) {
    line(img, points[i], points[ (i+1) % nbEdges ], col);
  }
}
Point2f cartesianToGrid (const Point2f& point) {

  if (point.x>GRID_X_MIN && point.x<GRID_X_MAX && point.y>GRID_Y_MIN && point.y<GRID_Y_MAX && point.y>0) {
    return Point2f((point.x-GRID_X_MIN)/GRID_X_STEP, (GRID_Y_MAX-(point.y-GRID_Y_MIN))/GRID_Y_STEP);
  }
  else {
    return Point2f(-1,-1);
  }
}

Rect cartesianToGrid (const Rect& rect) {
  Point2f topLeft = rect.tl();
  Point2f bottomRight = rect.br();

  Point2f topLeftConverted = cartesianToGrid(topLeft);
  Point2f bottomRightConverted = cartesianToGrid(bottomRight);

  return Rect(topLeftConverted, bottomRightConverted);
}


Point2f cartesianToImage (const Point2f& point) {
  double z=CAMERA_HEIGHT -(LIDAR_HEIGHT + sqrt(point.x*point.x+point.y*point.y)*sin(LIDAR_PITCH_ANGLE));
  int u=(int)INTRINSIC_U0 + INTRINSIC_ALPHA_U *
    (point.x/(point.y+CAMERA_TY));
  int v=(int)INTRINSIC_V0+ INTRINSIC_ALPHA_V *
    (z/(point.y+CAMERA_TY));

  return Point2f(u,v);
}

vector<Point2f> cartesianToImage (const Rect& rect) {
  Point2f topLeft = rect.tl();
  Point2f bottomRight = rect.br();
  Point2f topRight = Point2f(topLeft.x, bottomRight.y);
  Point2f bottomLeft = Point2f(bottomRight.x, topLeft.y);

  vector<Point2f> ret;
  ret.push_back(cartesianToImage(topLeft));
  ret.push_back(cartesianToImage(topRight));
  ret.push_back(cartesianToImage(bottomRight));
  ret.push_back(cartesianToImage(bottomLeft));
  return ret;
}

void drawOnImage (Mat& img, Point2f impact, Point2f pImpact, Rect roi) {
  Point2f meanImpactImage = cartesianToImage(impact);
  Point2f meanPredictedImpactImage = cartesianToImage(pImpact);
  vector<Point2f> roiImage = cartesianToImage(roi);
  circle(img, meanImpactImage, 1, Scalar(255,0,0));
  circle(img, meanPredictedImpactImage, 1, Scalar(0,255,255));
  polygon(img, roiImage, Scalar(0,255,0));
}

void drawOnGrid (Mat& img, Point2f impact, Point2f pImpact, Point2f speedPredicted, Rect roi) {
  Point2f meanImpactGrid = cartesianToGrid(impact);
  Point2f meanPredictedImpactGrid = cartesianToGrid(pImpact);
  Rect roiGrid = cartesianToGrid(roi);
  if (validIndex(img, meanImpactGrid)) {
    circle(img, meanImpactGrid, 1, Scalar(255,0,0));
  }
  if(validIndex(img, meanPredictedImpactGrid)) {
    circle(img, meanPredictedImpactGrid, 1, Scalar(0,255,255));
  }
  // the speed
  // line(img, meanPredictedImpactGrid, meanPredictedImpactGrid + speedPredicted, Scalar(0,0,255));

  if (validIndex(img, roiGrid.tl()) && validIndex(img, roiGrid.br())) {
    rectangle(img, roiGrid, Scalar(0,255,0));
  }
}

void plotSpeedMagnitude (Mat img, int frame_nb, Point2f speed1, Point2f speed2) {
  static Point2f previousPoint1(0,0);
  static Point2f previousPoint2(0,0);

  float scaleFactor = 50;
  float speed1Magn = sqrt(speed1.dot(speed1));
  float speed2Magn = sqrt(speed2.dot(speed2));

  Point2f point1(frame_nb, speed1Magn * scaleFactor);
  Point2f point2(frame_nb, speed2Magn * scaleFactor);

  line(img, previousPoint1, point1, Scalar(255,0,0));
  line(img, previousPoint2, point2, Scalar(0,0,255));

  previousPoint1 = point1;
  previousPoint2 = point2;
}

void readLidarData () {
  //  Read lidar data from a file
  Mat lidar_data;
  string filename("data/laser/lidarData.xml");
  FileStorage fs(filename, FileStorage::READ);
  fs["lidarData"]>> lidar_data;
  int nb_impacts = lidar_data.cols;
  int nb_frames = lidar_data.rows;
  int maxScaledUpSpeed = 60;
  Mat displaySpeedMagnitudePlot = Mat::zeros(Size(nb_frames, maxScaledUpSpeed), CV_8UC3);
  Point2f initialRoiTopLeft(4., 9.);
  Point2f initialRoiBottomRight(7.5, 11.);
  Rect bicyleRoi(initialRoiTopLeft, initialRoiBottomRight);
  Point2f meanImpactInRoi = computeMiddle(bicyleRoi);

  KalmanFilter KF(4, 2, 0);
  float timestep = 1./12.5;

  KF.transitionMatrix = (Mat_<float>(4, 4) <<
                         1, 0, timestep, 0,
                         0, 1, 0, timestep,
                         0, 0, 1, 0,
                         0, 0, 0, 1);
  setIdentity(KF.measurementMatrix);
  setIdentity(KF.processNoiseCov, Scalar::all(0));
  setIdentity(KF.measurementNoiseCov, Scalar::all(0));
  setIdentity(KF.errorCovPost, Scalar::all(1));
  KF.statePost.at<float>(0,0) = meanImpactInRoi.x;
  KF.statePost.at<float>(0,1) = meanImpactInRoi.y;
  KF.statePost.at<float>(0,2) = 0;
  KF.statePost.at<float>(0,3) = 0;



  char key = 'a';
  int frame_nb = 0;

  while (key != 'q' && frame_nb != nb_frames)
    {
      //  Allocation/initialization of the grid
      Mat grid = Mat::zeros(Size(GRID_NB_CELLS_X, GRID_NB_CELLS_Y), CV_32F);
      //  Read the stereo image
      ostringstream filename;
      filename<<"data/img/left_img_"<<frame_nb<<".png";
      Mat left_img = imread(filename.str(), 0);
      Mat left_display_img;
      cvtColor(left_img, left_display_img, CV_GRAY2RGB);
      Point2f allImpactSum(0.,0.);
      unsigned int totalImpactInRoi = 0;


      Mat prediction = KF.predict();
      Point2f meanPredicted(prediction.at<float>(0,0),
                            prediction.at<float>(0,1));
      Point2f speedPredicted(prediction.at<float>(0,2), - prediction.at<float>(0,3));
      speedPredicted = speedPredicted * timestep;

      shiftRoi(bicyleRoi, meanPredicted);

      //  Process all the lidar impacts
      for (int i=0; i<nb_impacts/2; ++i) {
        double x=lidar_data.at<double>(frame_nb, 2*i);
        double y=lidar_data.at<double>(frame_nb, 2*i+1);

        Point2f lidarImpact(x,y);
        if (bicyleRoi.contains(lidarImpact)) {
          allImpactSum+=lidarImpact;
          totalImpactInRoi++;
        }

        //  compute the grid
        Point2f onGrid(cartesianToGrid(lidarImpact));
        grid.at<float>(onGrid.y, onGrid.x) = 1.0;

        //  display on stereo image
        if ((y>0 && y<30) && (x>-10 && x<10)) {
          Point2f onImage(cartesianToImage(lidarImpact));
          if (validIndex(left_display_img, onImage)) {
            circle(left_display_img, onImage, 0, Scalar(0,0,255));
          }
        }
      }

      Point2f previousMeanImpact(meanImpactInRoi);
      float totalImpactInRoiInv = 1./totalImpactInRoi;
      meanImpactInRoi = allImpactSum * totalImpactInRoiInv;

      Point2f speedActual = meanImpactInRoi - previousMeanImpact;

      Mat measurement = (Mat_<float>(2, 1) <<
                         meanImpactInRoi.x,
                         meanImpactInRoi.y);

      KF.correct(measurement);

      //   prepare the display of the grid
      Mat display_grid; //  to have a RGB grid for display
      grid.convertTo(display_grid, CV_8U, 255);
      cvtColor(display_grid, display_grid, CV_GRAY2RGB);


      drawOnImage(left_display_img, meanImpactInRoi, meanPredicted, bicyleRoi);
      drawOnGrid(display_grid, meanImpactInRoi, meanPredicted, speedPredicted, bicyleRoi);
      plotSpeedMagnitude(displaySpeedMagnitudePlot, frame_nb, speedActual, speedPredicted);

      Mat display_grid_large;// to have a large grid for display
      resize(display_grid, display_grid_large, Size(600,600));
      Mat displayPlotLarge;// to have a large grid for display
      resize(displaySpeedMagnitudePlot, displayPlotLarge, Size(300,300));

      //  show images
      imshow("top view",  display_grid_large);
      imshow("left image", left_display_img);
      imshow("speed plot", displayPlotLarge);

      //  Wait for the user to press a key
      frame_nb++;
      key = waitKey( );
    }
}
