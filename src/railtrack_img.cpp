#include "rail_track/railtrack_img.hpp"

RailTrack_IMG::RailTrack_IMG(const string &path)
{
  Mat imgOriginal = cv::imread(path);          // open image
  if (imgOriginal.empty())						// if unable to open image
    cout << "Cannot open image" << endl;

  track(imgOriginal);
  imgOriginal = cv::imread(path);
  track(imgOriginal);
  imgOriginal = cv::imread(path);
  track(imgOriginal);
  imgOriginal = cv::imread(path);
  track(imgOriginal);
  waitKey(0);
}
