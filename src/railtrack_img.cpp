#include "rail_track/railtrack_img.hpp"

RailTrack_IMG::RailTrack_IMG(const string &path) :
  loop_rate(30),
  it(n)
{
  img_pub = it.advertise("/rail_track/frame", 1);

  Mat frame = cv::imread(path);          // open image
  if (!frame.empty())
  {
    msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    img_pub.publish(msg_img);
  }
  else
    cout << "Cannot open image" << endl;
  loop_rate.sleep();
}

RailTrack_IMG::~RailTrack_IMG()
{

}
