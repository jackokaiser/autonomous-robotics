#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>

#include "stereo.h"
#include "readLidarData.h"


#define CARTESIAN_SEG 1
#define DISPARITY_SEG 2
#define LIDAR_TRACK 3

using namespace cv;
using namespace std;


int main(int argc, char **argv)
{
  cout<<"OpenCV version: "<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION<<endl;

  if (argc < 2) {
    cout<<"Usage : "<<argv[0]<<" {exerciceNb} "<<endl;
    cout<<"\t\t 1 <-> cartesian space clustering"<<endl;
    cout<<"\t\t 2 <-> disparity space clustering"<<endl;
    cout<<"\t\t 3 <-> lidar tracking"<<endl;
    return 0;
  }

  switch(atoi(argv[1])) {
  case CARTESIAN_SEG:
    cout<<"Cartesian filtering"<<endl;
    stereoDisparity(CARTESIAN_SPACE);
    break;
  case DISPARITY_SEG:
    cout<<"Disparity filtering"<<endl;
    stereoDisparity(DISPARITY_SPACE);
    break;
  case LIDAR_TRACK:
    readLidarData();
    break;
  }

  return 0;
}
