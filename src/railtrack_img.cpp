#include "rail_track/railtrack_img.hpp"

RailTrack_IMG::RailTrack_IMG(const string &path):
  loop_rate(30),
  it(n)
{
  frame_pub = it.advertise("/rail_track/frame", 1);
  Mat frame = cv::imread(path);          // open image
  if (frame.empty())						// if unable to open image
    cout << "Cannot open image" << endl;

  frame = cv::imread(path);

  while(!frame.empty() && ros::ok())
  {
    msg_frame = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    frame_pub.publish(msg_frame);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
