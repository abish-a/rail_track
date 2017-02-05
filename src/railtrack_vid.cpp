#include "rail_track/railtrack_vid.hpp"

RailTrack_VID::RailTrack_VID(const string &path) :
  loop_rate(60),
  it(n)
{
  frame_pub = it.advertise("/rail_track/frame", 1);
  Mat frame;
  m_video.open(path);

  while(m_video.isOpened() && n.ok())
  {
    if (!m_video.read(frame))
    {
      cerr << "error: frame not read from video" << endl;
    }

    msg_frame = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    frame_pub.publish(msg_frame);
    loop_rate.sleep();
  }
}

RailTrack_VID::~RailTrack_VID()
{
  m_video.release();
}
