#include <opencv2/opencv.hpp>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;

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

  KalmanFilter KF(4, 2, 0);
  float timestep = 1./12.5;

  KF.transitionMatrix = (Mat_<float>(4, 4) <<
                         1, 0, timestep, 0,
                         0, 1, 0, timestep,
                         0, 0, 1, 0,
                         0, 0, 0, 1);
  setIdentity(KF.measurementMatrix);
  setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
  setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
  setIdentity(KF.errorCovPost, Scalar::all(1));
  KF.statePost.at<float>(0,0) = initialRoiTopLeft.x + initialRoiBottomRight.x / 2.;
  KF.statePost.at<float>(0,1) = initialRoiTopLeft.y + initialRoiBottomRight.y / 2.;
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
          if (x>x_min && x<x_max && y>y_min && y<y_max && y>0)
            grid.at<float>((y_max-(y-y_min))/y_step, (x-x_min)/x_step) = 1.0;

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
      float totalImpactInRoiInv = 1./totalImpactInRoi;
      Point2f meanImpactInRoi = allImpactSum * totalImpactInRoiInv;
      double z=camera_height -(lidar_height + sqrt(meanImpactInRoi.x*meanImpactInRoi.x+meanImpactInRoi.y*meanImpactInRoi.y)*sin(lidar_pitch_angle));
      int u=(int)uo+alpha_u*(meanImpactInRoi.x/(meanImpactInRoi.y+camera_ty));
      int v=(int)vo+alpha_v*(z/(meanImpactInRoi.y+camera_ty));
      if (u>0 && u<left_img.cols && v>0 && v<left_img.rows)
        {
          left_display_img.at<unsigned char>(v, 3*u) = 0;
          left_display_img.at<unsigned char>(v, 3*u+1) = 255;
          left_display_img.at<unsigned char>(v, 3*u+2) = 0;
        }

      //   prepare the display of the grid
      Mat display_grid; //  to have a RGB grid for display
      grid.convertTo(display_grid, CV_8U, 255);
      cvtColor(display_grid, display_grid, CV_GRAY2RGB);

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
