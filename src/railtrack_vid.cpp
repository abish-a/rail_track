/*!
 * \file	rail_track
 * \author	Abish Asphandiar <abish@uni-bremen.de> 3020632
 * \date	19-October-2016
 * \brief Publishing video
 */

#include "rail_track/railtrack_vid.hpp"

RailTrack_VID::RailTrack_VID(const string &path):
  loop_rate(30)
{
  frame_pub = n.advertise<rail_track::Frame>("/rail_track/frame", 1);
  Mat frame;
  m_video.open(path);

  if (m_video.isOpened() == false)
    cerr << "error: Video not accessed successfully" << endl;

  while(m_video.isOpened() && ros::ok())
  {
    if (!m_video.read(frame))
    {
      cerr << "error: frame not read from video" << endl;
    }

    msg_frame.timestamp.stamp = ros::Time::now();
    sensor_msgs::ImagePtr im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    msg_frame.orig_image = *im_msg;
    frame_pub.publish(msg_frame);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

RailTrack_VID::~RailTrack_VID()
{
  m_video.release();
}
