#ifndef RAILTRACK_IMG_H
#define RAILTRACK_IMG_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include <ctime>
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "rail_track/Frame.h"

using namespace std;
using namespace cv;

class RailTrack_IMG
{
public:
  RailTrack_IMG(const string &path);

private:
  Mat m_imgOriginal;                   // input image
  ros::NodeHandle n;
  ros::Rate loop_rate;
  ros::Publisher frame_pub;
  rail_track::Frame msg_frame;
};

#endif // RAILTRACK_IMG_H
