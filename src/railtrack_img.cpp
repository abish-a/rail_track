#include "rail_track/railtrack_img.hpp"

RailTrack_IMG::RailTrack_IMG(const string &path)
{
  Mat imgOriginal = cv::imread(path);          // open image
  if (imgOriginal.empty())						// if unable to open image
    cout << "Cannot open image" << endl;

  /** Create some points */
  poly_points[0][0]  = Point(0, imgOriginal.rows);
  poly_points[0][1]  = Point(imgOriginal.cols/3, imgOriginal.rows/2);
  poly_points[0][2]  = Point(imgOriginal.cols*2/3, imgOriginal.rows/2);
  poly_points[0][3]  = Point(imgOriginal.cols, imgOriginal.rows);

  track(imgOriginal);

  waitKey(0);

  /** Create some points */
  poly_points[0][0]  = Point((m_aTracks[1][0] - ROI_X), m_aTracks[1][1]);
  poly_points[0][1]  = Point((m_aTracks[1][2] - ROI_Y), (m_aTracks[1][3] - ROI_Y));
  poly_points[0][2]  = Point((m_aTracks[0][0] + ROI_Y), (m_aTracks[0][1] - ROI_Y));
  poly_points[0][3]  = Point((m_aTracks[0][2] + ROI_X), m_aTracks[0][3]);

  imgOriginal = cv::imread(path);
  track(imgOriginal);

  waitKey(0);
}
