#include <opencv2/opencv.hpp>
#include "utils.h"

using namespace cv;
using namespace std;

unsigned int segmentDisparity(const Mat &disparity, Mat &output)
{
  output = Mat::zeros(disparity.size(), CV_32SC1);
  Mat tmp=Mat::zeros(disparity.size(), CV_8UC1);
  Mat tmp2=Mat::zeros(disparity.size(), CV_32SC1);

  disparity.convertTo(tmp, CV_8UC1);
  Mat mask=Mat::zeros(disparity.size().height+2,
                      disparity.size().width+2,
                      CV_8UC1);

  Mat mask2 = mask(Rect(1, 1, disparity.size().width, disparity.size().height));
  //threshold(tmp, mask2, 0, 255, CV_THRESH_BINARY_INV);

  unsigned int k=1;

  for(int i=0; i<tmp.rows; i++)
    {
      const unsigned char* buf = tmp.ptr<unsigned char>(i);
      for(int j=0; j<tmp.cols; j++)
        {
          unsigned char d = buf[j];
          if(d>0)
            {
              if(floodFill(tmp,
                           mask,
                           Point(j, i),
                           Scalar(1),
                           NULL,
                           cvScalarAll(0),
                           cvScalarAll(0),
                           8+FLOODFILL_FIXED_RANGE+FLOODFILL_MASK_ONLY)>0)
                {
                  mask2.convertTo(tmp2, CV_32SC1, k);
                  output += tmp2;
                  tmp.setTo(0, mask2);
                  mask.setTo(0);
                  k++;
                }
            }
        }
    }
  return k;
 }
