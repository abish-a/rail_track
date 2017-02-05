#ifndef RAILTRACK_H
#define RAILTRACK_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <limits>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "rail_track/Roi.h"

#define DMAX 260//115//140
#define DMIN 220//5//30
#define ROI_X 50
#define ROI_Y 20

using namespace cv;
using namespace std;

class RailTrack
{
public:
  RailTrack();
  ~RailTrack();
  float getYintersect(const Vec4f &line);
  float getSlope(const Vec4f &line);
  float getLength(const Vec4f &line);
  void DoHough(const Mat &dst);
  void showWindow(const string &title, const Mat &image);
  Mat region_of_interest(const Mat &imgBin);
  vector<Vec4i> extend_lines(const Vec4i &l, const Vec4i &l_2);
  void getCurves(const Mat &imgCanny);
  void setROI(void);
  void track(const sensor_msgs::ImageConstPtr &msg);

private:
  Mat m_imgOriginal;
  int m_canny = 300;
  int line_number;
  Vec4i m_aPrevLines[2];
  Vec4i m_aTracks[2];
  Point poly_points[1][4];
  int prev_track_0;
  int prev_track_1;

  ros::NodeHandle n;
  image_transport::ImageTransport it;
  image_transport::Subscriber frame_sub;
  ros::Publisher roi_pub;
  ros::Rate loop_rate;
  rail_track::Roi msg_roi;
};

#endif // RAILTRACK_H
