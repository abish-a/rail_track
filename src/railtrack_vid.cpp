#include "rail_track/railtrack_vid.hpp"

RailTrack_VID::RailTrack_VID(const string &path)
{
  Mat imgOriginal;
  char esc = 0;
  char space = 0;
  char go = 0;
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

    int prev_track_0 = m_aTracks[0][2];
    int prev_track_1 = m_aTracks[1][0];

    track(imgOriginal);

    if (m_aTracks[1][0] != -1 && m_aTracks[1][0] != 0)
    {
      if((abs(prev_track_0 - m_aTracks[0][2]) < 5) && (abs(prev_track_1 - m_aTracks[1][0]) < 5))
      {
        /** Create some points */
        poly_points[0][0]  = Point((m_aTracks[1][0] - ROI_X), m_aTracks[1][1]);
        poly_points[0][1]  = Point((m_aTracks[1][2] - ROI_Y), (m_aTracks[1][3] - ROI_Y));
        poly_points[0][2]  = Point((m_aTracks[0][0] + ROI_Y), (m_aTracks[0][1] - ROI_Y));
        poly_points[0][3]  = Point((m_aTracks[0][2] + ROI_X), m_aTracks[0][3]);
        myfile << "Yes" << endl;
      }
      else
        myfile << "No" << endl;
    }
    else
      myfile << "No" << endl;

//    space = waitKey(1);
//    while (space == 32)
//    {
//      go = waitKey(1);
//      if (go == 32)
//        break;
//    }

    esc = waitKey(1);
  }
  m_video.release();
}
