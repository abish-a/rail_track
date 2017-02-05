#include "rail_track/railtrack_img.hpp"

RailTrack_IMG::RailTrack_IMG(const string &path):
  loop_rate(30)
{
  frame_pub = n.advertise<rail_track::Frame>("/rail_track/frame", 1);
  Mat frame = cv::imread(path);          // open image
  if (frame.empty())						// if unable to open image
    cout << "Cannot open image" << endl;

  frame = cv::imread(path);

  while(!frame.empty() && ros::ok())
  {
    msg_frame.timestamp.stamp = ros::Time::now();
    sensor_msgs::ImagePtr im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    msg_frame.orig_image = *im_msg;
    frame_pub.publish(msg_frame);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
