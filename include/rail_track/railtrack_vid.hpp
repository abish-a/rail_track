#ifndef RAILTRACK_VID_H
#define RAILTRACK_VID_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>

using namespace cv;
using namespace std;

class RailTrack_VID
{
public:
  RailTrack_VID(const string &path);
  ~RailTrack_VID();

private:
  VideoCapture m_video;
  ros::NodeHandle n;
  ros::Rate loop_rate;
  image_transport::ImageTransport it;
  image_transport::Publisher frame_pub;
  sensor_msgs::ImagePtr msg_frame;
};

#endif // RAILTRACK_VID_H
