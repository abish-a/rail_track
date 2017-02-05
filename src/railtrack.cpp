#include "rail_track/railtrack.hpp"


RailTrack::RailTrack() :
  loop_rate(30),
  it(n)
{
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 4; j++)
    {
      m_aPrevLines[i][j] = 0;
      m_aTracks[i][j] = 0;
    }

  poly_points[0][0]  = Point(0, 720);
  poly_points[0][1]  = Point(427, 360);
  poly_points[0][2]  = Point(853, 360);
  poly_points[0][3]  = Point(1280, 720);

  roi_pub = n.advertise<rail_track::Roi>("/rail_track/roi", 1000);
  frame_sub = it.subscribe("/rail_track/frame", 1, &RailTrack::track, this);
  ros::spin();
}

RailTrack::~RailTrack()
{

}

float RailTrack::getYintersect(const Vec4f &line)
{
  if (line[2] == line[0])
    return numeric_limits<float>::max();
  else
    return static_cast<float>(((line[2] * line[1]) - (line[0] * line[3])) / (line[2] - line[0]));
}

float RailTrack::getSlope(const Vec4f &line)
{
  if (line[2] == line[0])
    return numeric_limits<float>::max();
  else
    return ((line[3] - line[1]) / (line[2] - line[0]));
}

float RailTrack::getLength(const Vec4f &line)
{
  return sqrt(((line[3] - line[1]) * (line[3] - line[1])) + ((line[2] - line[0]) * (line[2] - line[0])));
}

void RailTrack::DoHough(const Mat &dst)
{
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 4; j++)
    {
      m_aPrevLines[i][j] = 0;
      //m_aTracks[i][j] = 0;
    }

  line_number = 0;
  vector<Vec4i> lines;

  HoughLinesP(dst, lines, 2, CV_PI / 180, 50, 50, 10);
  line_number = lines.size();

  for (size_t i = 0; i < lines.size(); i++)
  {
    Vec4i l = lines.at(i);
    for (size_t j = 0; j < lines.size(); j++)
    {
      Vec4i l_2 = lines.at(j);
      vector<Vec4i> ext_lines = extend_lines(l, l_2);

      if ((abs(ext_lines[0][2] - ext_lines[1][0]) > DMIN) && (abs(ext_lines[0][2] - ext_lines[1][0]) < DMAX))
      {
        if (getLength(l) >= getLength(m_aPrevLines[0]) && getLength(l_2) >= getLength(m_aPrevLines[1]))
        {
          m_aTracks[0] = ext_lines[0]; // right track
          m_aTracks[1] = ext_lines[1]; //left track
          m_aPrevLines[0] = l;
          m_aPrevLines[1] = l_2;
        }
      }
    }
  }

  /// Draw Lines
  line(m_imgOriginal, Point(m_aTracks[0][0], m_aTracks[0][1]), Point(m_aTracks[0][2], m_aTracks[0][3]), Scalar(0, 0, 255), 2, CV_AA);
  line(m_imgOriginal, Point(m_aTracks[1][0], m_aTracks[1][1]), Point(m_aTracks[1][2], m_aTracks[1][3]), Scalar(0, 0, 255), 2, CV_AA);
  /// Draw ROI
  line(m_imgOriginal, poly_points[0][0], poly_points[0][1], Scalar(255, 0, 0), 2, CV_AA);
  line(m_imgOriginal, poly_points[0][1], poly_points[0][2], Scalar(255, 0, 0), 2, CV_AA);
  line(m_imgOriginal, poly_points[0][2], poly_points[0][3], Scalar(255, 0, 0), 2, CV_AA);
}

void RailTrack::showWindow(const string &title, const Mat &image)
{
  cv::namedWindow(title, CV_WINDOW_AUTOSIZE);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
  cv::imshow(title, image);                   // show windows
}

Mat RailTrack::region_of_interest(const Mat &imgBin)
{
  Mat imgPoly(imgBin.size(), imgBin.type());
  Mat Result;
  int lineType = LINE_8;
  const Point* ppt[1] = {poly_points[0]};
  int npt[] = {4};

  imgPoly.setTo(0);
  fillPoly(imgPoly, ppt,npt,1, Scalar(255, 255, 255), lineType);
  bitwise_and(imgPoly, imgBin, Result);
  return Result;
}

vector<Vec4i> RailTrack::extend_lines(const Vec4i &l, const Vec4i &l_2)
{
  vector<Vec4i> final_lines;
  Vec4i extended_lines;
  Point intersection;

  float fSlope0 = getSlope(l);
  float fSlope1 = getSlope(l_2);
  float fYint0 = getYintersect(l);
  float fYint1 = getYintersect(l_2);
  intersection.y = (((fSlope0 * fYint1) - (fSlope1 * fYint0)) / (fSlope0 - fSlope1));
  intersection.x = ((fYint0 - fYint1) / (fSlope1 - fSlope0));

  int iYmax = m_imgOriginal.rows - 1;

  if (abs(intersection.y - 270) < 50)
  {
    extended_lines[0] = intersection.x;
    extended_lines[1] = intersection.y;
    extended_lines[2] = (iYmax - fYint0)/fSlope0;
    extended_lines[3] = iYmax;

    if (abs(abs(intersection.x - extended_lines[2]) - 175) < 50)
      final_lines.push_back(extended_lines);
    else
    {
      for (int i = 0; i < 4;i++)
        extended_lines[i] = 0;
      final_lines.push_back(extended_lines);
    }

    extended_lines[0] = (iYmax - fYint1)/fSlope1;
    extended_lines[1] = iYmax;
    extended_lines[2] = intersection.x;
    extended_lines[3] = intersection.y;

    if (abs(abs(intersection.x - extended_lines[0]) - 60) < 50)
      final_lines.push_back(extended_lines);
    else
    {
      for (int i = 0; i < 4;i++)
        extended_lines[i] = 0;
      final_lines.push_back(extended_lines);
    }
  }
  else
  {
    for (int i = 0; i < 4;i++) //sending zero
      extended_lines[i] = 0;
    final_lines.push_back(extended_lines);
    final_lines.push_back(extended_lines);
  }
  return final_lines;
}

void RailTrack::getCurves(const Mat &imgCanny)
{
  float cost_function;
  vector<Point> left_rail;
  vector<Point> right_rail;
  Point left_point;
  Point right_point;

  float fSlope0 = getSlope(m_aTracks[0]);
  float fSlope1 = getSlope(m_aTracks[1]);
  float fYint0 = getYintersect(m_aTracks[0]);
  float fYint1 = getYintersect(m_aTracks[1]);
  int intersection = (((fSlope0 * fYint1) - (fSlope1 * fYint0)) / (fSlope0 - fSlope1));

  int iWmax = m_aTracks[0][2] - m_aTracks[1][2];
  if (iWmax > 0)
  {
    left_rail.push_back(Point(m_aPrevLines[1][0], m_aPrevLines[1][1]));
    right_rail.push_back(Point(m_aPrevLines[0][2], m_aPrevLines[0][3]));

    for (int i = m_aPrevLines[1][1]; i > intersection; i--) //from bottom of the image to the top
    {
      cost_function = 200;
      for (int j = -1; j < 2; j++)
      {
        int current_cost = imgCanny.at<uchar>(Point((left_rail.at(left_rail.size() - 1).x + j), i));
        if (current_cost >= cost_function)
        {
          cost_function = current_cost;
          left_point = Point((left_rail.at(left_rail.size() - 1).x + j), i);
        }
      }
      if (cost_function > 200)
        left_rail.push_back(left_point);
    }

    for (int i = m_aPrevLines[0][3]; i > intersection; i--) //from bottom of the image to the top
    {
      cost_function = 200;
      for (int j = -1; j < 2; j++)
      {
        int current_cost = imgCanny.at<uchar>(Point((right_rail.at(right_rail.size() - 1).x + j), i));
        if (current_cost >= cost_function)
        {
          cost_function = current_cost;
          right_point = Point((right_rail.at(right_rail.size() - 1).x + j), i);
        }
      }
      if (cost_function > 200)
        right_rail.push_back(right_point);
    }

    /// Draw contours
    for( int i = 0; i < left_rail.size(); i++ )
      for (int j = -1; j < 2; j++)
        m_imgOriginal.at<Vec3b>(Point(left_rail.at(i).x + j, left_rail.at(i).y)) = Vec3b(0,255,0);
    for( int i = 0; i < right_rail.size(); i++ )
      for (int j = -1; j < 2; j++)
        m_imgOriginal.at<Vec3b>(Point(right_rail.at(i).x + j, right_rail.at(i).y)) = Vec3b(0,255,0);
  }
}

void RailTrack::setROI()
{
  if (m_aTracks[1][0] != -1 && m_aTracks[1][0] != 0)
    if((abs(prev_track_0 - m_aTracks[0][2]) < 5) && (abs(prev_track_1 - m_aTracks[1][0]) < 5))
    {
      /** Create some points */
      poly_points[0][0]  = Point((m_aTracks[1][0] - ROI_X), m_aTracks[1][1]);
      poly_points[0][1]  = Point((m_aTracks[1][2] - ROI_Y), (m_aTracks[1][3] - ROI_Y));
      poly_points[0][2]  = Point((m_aTracks[0][0] + ROI_Y), (m_aTracks[0][1] - ROI_Y));
      poly_points[0][3]  = Point((m_aTracks[0][2] + ROI_X), m_aTracks[0][3]);
    }
  prev_track_0 = m_aTracks[0][2];
  prev_track_1 = m_aTracks[1][0];
}

void RailTrack::track(const sensor_msgs::ImageConstPtr& msg)
{
  Mat imgOriginal = cv_bridge::toCvShare(msg, "bgr8")->image;

  Mat imgGrayscale;       // grayscale of input image
  Mat imgCanny;
  Mat imgROI;
  Mat imgPoly(imgOriginal.size(), imgOriginal.type());
  Mat Result;
  const Point* ppt[1] = {poly_points[0]};
  int npt[] = {4};

  m_imgOriginal = imgOriginal;
  cvtColor(m_imgOriginal, imgGrayscale, CV_BGR2GRAY);       // convert to grayscale
  double gray_mean = mean(imgGrayscale)[0];
  GaussianBlur(imgGrayscale, imgGrayscale, Size(9, 9), 0, 0, BORDER_DEFAULT);

  Canny(imgGrayscale, imgCanny, 80, m_canny);       // Get Canny Edge
  imgROI = region_of_interest(imgCanny);
  DoHough(imgROI);
  getCurves(imgCanny);
  setROI();

  if (line_number != 10)
    m_canny = gray_mean * 300 / 115;

  //showWindow("imgGrayscale", imgGrayscale);
  showWindow("imgCanny", imgCanny);
  //showWindow("imgROI", imgROI);
  showWindow("Tracked", m_imgOriginal);

  imgPoly.setTo(0);
  fillPoly(imgPoly, ppt,npt,1, Scalar(255, 255, 255), LINE_8);
  bitwise_and(imgPoly, imgOriginal, Result);
  sensor_msgs::ImagePtr im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Result).toImageMsg();

  msg_roi.roi_0.x = poly_points[0][0].x;
  msg_roi.roi_0.y = poly_points[0][0].y;
  msg_roi.roi_0.z = 1;
  msg_roi.roi_1.x = poly_points[0][1].x;
  msg_roi.roi_1.y = poly_points[0][1].y;
  msg_roi.roi_1.z = 1;
  msg_roi.roi_2.x = poly_points[0][2].x;
  msg_roi.roi_2.y = poly_points[0][2].y;
  msg_roi.roi_2.z = 1;
  msg_roi.roi_3.x = poly_points[0][3].x;
  msg_roi.roi_3.y = poly_points[0][3].y;
  msg_roi.roi_3.z = 1;
  msg_roi.roi_image = *im_msg;
  roi_pub.publish(msg_roi);
  //loop_rate.sleep();
  cv::waitKey(30);
}
