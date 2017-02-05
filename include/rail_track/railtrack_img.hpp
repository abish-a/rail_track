#ifndef RAILTRACK_IMG_H
#define RAILTRACK_IMG_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>

using namespace cv;
using namespace std;

class RailTrack_IMG
{
public:
  RailTrack_IMG(const string &path);
  ~RailTrack_IMG();

private:
  Mat m_imgOriginal;                   // input image
  ros::NodeHandle n;
  ros::Rate loop_rate;
  image_transport::ImageTransport it;
  image_transport::Publisher img_pub;
  sensor_msgs::ImagePtr msg_img;
};

#endif // RAILTRACK_IMG_H
