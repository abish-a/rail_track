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
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include <time.h>
#include <fstream>

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
  Mat DoHough(const Mat &dst);
  void showWindow(const string &title, const Mat &image);
  Mat region_of_interest(const Mat &imgBin);
  void getContours(const Mat &imgCanny);
  vector<Vec4i> extend_lines(Vec4i l, Vec4i l_2);
  void track(Mat &imgOriginal);

private:
  Mat m_imgOriginal;
  int m_canny = 300;
  int line_number;
  Vec4i m_aPrevLines[2];


  ros::NodeHandle n;
  ros::Publisher lines_pub;
  ros::Rate loop_rate;
  geometry_msgs::Point msg_tracks;


protected:
  Vec4i m_aTracks[2];
  Point poly_points[1][4];
  ofstream myfile;
};

#endif // RAILTRACK_H
