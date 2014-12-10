#include <opencv2/opencv.hpp>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

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
  //  define the parameters of the grid
  float x_min = -10.;
  float x_max = 10.;
  float y_min = 0;
  float y_max = 30.;
  float x_step = 0.2;
  float y_step =  0.2;
  int nb_cells_x = (x_max-x_min)/x_step;
  int nb_cells_y = (y_max-y_min)/y_step;

  if (point.x>x_min && point.x<x_max && point.y>y_min && point.y<y_max && point.y>0) {
    return Point2f((point.x-x_min)/x_step, (y_max-(point.y-y_min))/y_step);
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
  //  extrinsic parameters of the lidar
  double lidar_pitch_angle = -1.*M_PI/180;
  double lidar_height = 0.47;
  //  parameters of the camera
  double uo = 256;
  double vo = 156;
  double alpha_u = 410;
  double alpha_v = 410;
  double camera_height = 1.28;
  double camera_ty = 1.8;

  double z=camera_height -(lidar_height + sqrt(point.x*point.x+point.y*point.y)*sin(lidar_pitch_angle));
  int u=(int)uo+alpha_u*(point.x/(point.y+camera_ty));
  int v=(int)vo+alpha_v*(z/(point.y+camera_ty));

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

void setPixelColor(Mat img, const Point2f& pixelCoord, const Scalar& color) {
  img.at<unsigned char>(pixelCoord.y, 3*pixelCoord.x) = color[0];
  img.at<unsigned char>(pixelCoord.y, 3*pixelCoord.x+1) = color[1];
  img.at<unsigned char>(pixelCoord.y, 3*pixelCoord.x+2) = color[2];
}

void readLidarData () {
  //  Read lidar data from a file
  Mat lidar_data;
  string filename("data/laser/lidarData.xml");
  FileStorage fs(filename, FileStorage::READ);
  fs["lidarData"]>> lidar_data;
  int nb_impacts = lidar_data.cols;
  int nb_frames = lidar_data.rows;
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
  setIdentity(KF.processNoiseCov, Scalar::all(1e-1));
  setIdentity(KF.measurementNoiseCov, Scalar::all(1e-5));
  setIdentity(KF.errorCovPost, Scalar::all(1));
  KF.statePost.at<float>(0,0) = meanImpactInRoi.x;
  KF.statePost.at<float>(0,1) = meanImpactInRoi.y;
  KF.statePost.at<float>(0,2) = 0;
  KF.statePost.at<float>(0,3) = 0;




  //  extrinsic parameters of the lidar
  double lidar_pitch_angle = -1.*M_PI/180;
  double lidar_height = 0.47;
  //  parameters of the camera
  double uo = 256;
  double vo = 156;
  double alpha_u = 410;
  double alpha_v = 410;
  double camera_height = 1.28;
  double camera_ty = 1.8;

  //  define the parameters of the grid
  float x_min = -10.;
  float x_max = 10.;
  float y_min = 0;
  float y_max = 30.;
  float x_step = 0.2;
  float y_step =  0.2;
  int nb_cells_x = (x_max-x_min)/x_step;
  int nb_cells_y = (y_max-y_min)/y_step;


  char key = 'a';
  int frame_nb = 0;

  while (key != 'q' && frame_nb != nb_frames)
    {
        //  Allocation/initialization of the grid
      Mat grid = Mat::zeros(Size(nb_cells_x, nb_cells_y), CV_32F);
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
      shiftRoi(bicyleRoi, meanPredicted);

      //  Process all the lidar impacts
      for (int i=0; i<nb_impacts/2; ++i)
        {
          double x=lidar_data.at<double>(frame_nb, 2*i);
          double y=lidar_data.at<double>(frame_nb, 2*i+1);

          Point2f lidarImpact(x,y);
          if (bicyleRoi.contains(lidarImpact)) {
            allImpactSum+=lidarImpact;
            totalImpactInRoi++;
          }

          //  compute the grid
          if (x>x_min && x<x_max && y>y_min && y<y_max && y>0) {
            grid.at<float>((y_max-(y-y_min))/y_step, (x-x_min)/x_step) = 1.0;
          }

          //  display on stereo image
          if ((y>0 && y<30) && (x>-10 && x<10))
            {
              double z=camera_height -(lidar_height + sqrt(x*x+y*y)*sin(lidar_pitch_angle));
              int u=(int)uo+alpha_u*(x/(y+camera_ty));
              int v=(int)vo+alpha_v*(z/(y+camera_ty));
              if (u>0 && u<left_img.cols && v>0 && v<left_img.rows)
                {
                  left_display_img.at<unsigned char>(v, 3*u) = 0;
                  left_display_img.at<unsigned char>(v, 3*u+1) = 0;
                  left_display_img.at<unsigned char>(v, 3*u+2) = 255;
                }
            }
        }
      Point2f previousMeanImpact(meanImpactInRoi);
      float totalImpactInRoiInv = 1./totalImpactInRoi;
      meanImpactInRoi = allImpactSum * totalImpactInRoiInv;

      Mat measurement = (Mat_<float>(2, 1) <<
                         meanImpactInRoi.x,
                         meanImpactInRoi.y);

      KF.correct(measurement);
      Point2f meanImpactImage = cartesianToImage(meanImpactInRoi);
      Point2f meanPredictedImpactImage = cartesianToImage(meanPredicted);
      vector<Point2f> roiImage = cartesianToImage(bicyleRoi);

      Point2f meanImpactGrid = cartesianToGrid(meanImpactInRoi);
      Point2f meanPredictedImpactGrid = cartesianToGrid(meanPredicted);
      Rect roiGrid = cartesianToGrid(bicyleRoi);

      //   prepare the display of the grid
      Mat display_grid; //  to have a RGB grid for display
      grid.convertTo(display_grid, CV_8U, 255);
      cvtColor(display_grid, display_grid, CV_GRAY2RGB);

      circle(left_display_img, meanImpactImage, 1, Scalar(255,0,0));
      circle(left_display_img, meanPredictedImpactImage, 1, Scalar(0,255,255));
      polygon(left_display_img, roiImage, Scalar(0,255,0));

      if (validIndex(display_grid, meanImpactGrid)) {
        circle(display_grid, meanImpactGrid, 1, Scalar(255,0,0));
      }
      if(validIndex(display_grid, meanPredictedImpactGrid)) {
        circle(display_grid, meanPredictedImpactGrid, 1, Scalar(0,255,255));
      }
      line(display_grid, meanPredictedImpactGrid, meanPredictedImpactGrid + speedPredicted, Scalar(0,0,255));

      if (validIndex(display_grid, roiGrid.tl()) && validIndex(display_grid, roiGrid.br())) {
        rectangle(display_grid, roiGrid, Scalar(0,255,0));
      }

      Mat display_grid_large;// to have a large grid for display
      resize(display_grid, display_grid_large, Size(600,600));

      //  show images
      imshow("top view",  display_grid_large);
      imshow("left image", left_display_img);

      //  Wait for the user to press a key
      frame_nb++;
      key = waitKey( );
    }
}
