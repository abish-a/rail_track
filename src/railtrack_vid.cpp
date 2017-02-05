#include "rail_track/railtrack_vid.hpp"

RailTrack_VID::RailTrack_VID(const string &path)
{
  Mat imgOriginal;
  char esc = 0;
  m_video.open(path);

  if (m_video.isOpened() == false)
    cerr << "error: Video not accessed successfully" << endl;

  while(m_video.isOpened() && esc != 27)
  {
    if (!m_video.read(imgOriginal))
    {
      cerr << "error: frame not read from video" << endl;
    }

    track(imgOriginal);

    esc = waitKey(1);
  }
  m_video.release();
}
