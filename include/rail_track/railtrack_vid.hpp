#ifndef RAILTRACK_VID_H
#define RAILTRACK_VID_H

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

class RailTrack_VID
{
public:
  RailTrack_VID(const string &path);
  ~RailTrack_VID();

private:
  VideoCapture m_video;
  ros::NodeHandle n;
  ros::Rate loop_rate;
  ros::Publisher frame_pub;
  rail_track::Frame msg_frame;
};

#endif // RAILTRACK_VID_H
