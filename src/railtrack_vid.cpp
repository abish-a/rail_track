#include "rail_track/railtrack_vid.hpp"

RailTrack_VID::RailTrack_VID(const string &path)
{
  Mat imgOriginal;
  char esc = 0;
  m_video.open(path);

  if (m_video.isOpened() == false)
    cerr << "error: Video not accessed successfully" << endl;

  if(m_video.isOpened() && m_video.read(imgOriginal))
  {
    /** Create some points */
    poly_points[0][0]  = Point(0, imgOriginal.rows);
    poly_points[0][1]  = Point(imgOriginal.cols/3, imgOriginal.rows/2);
    poly_points[0][2]  = Point(imgOriginal.cols*2/3, imgOriginal.rows/2);
    poly_points[0][3]  = Point(imgOriginal.cols, imgOriginal.rows);
  }

  while(m_video.isOpened() && esc != 27)
  {
    if (!m_video.read(imgOriginal))
    {
      cerr << "error: frame not read from video" << endl;
    }

    float prev_track_0 = m_aTracks[0][2];
    float prev_track_1 = m_aTracks[1][0];

    track(imgOriginal);

    if ((m_aTracks[1][0] != -1) && (abs(prev_track_0 - m_aTracks[0][2]) < 2.0) && (abs(prev_track_1 - m_aTracks[1][0]) < 2.0))
    {
      /** Create some points */
      poly_points[0][0]  = Point((m_aTracks[1][0] - 50), m_aTracks[1][1]);
      poly_points[0][1]  = Point((m_aTracks[1][2] - 20), (m_aTracks[1][3] - 20));
      poly_points[0][2]  = Point((m_aTracks[0][0] + 20), (m_aTracks[0][1] - 20));
      poly_points[0][3]  = Point((m_aTracks[0][2] + 50), m_aTracks[0][3]);
    }

    esc = waitKey(1);
  }
  m_video.release();
}
