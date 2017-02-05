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

  /// initial region of interest
  poly_points[0][0]  = Point(0, 720);
  poly_points[0][1]  = Point(427, 360);
  poly_points[0][2]  = Point(853, 360);
  poly_points[0][3]  = Point(1280, 720);

  prev_track_0 = m_aTracks[0][2];
  prev_track_1 = m_aTracks[1][0];

  roi_pub = n.advertise<rail_track::Roi>("/rail_track/roi", 1);
  frame_sub = it.subscribe("/rail_track/frame", 1, &RailTrack::track, this);
  myfile.open ("lines.txt");

  while (ros::ok())
  {
    roi_pub.publish(msg_roi);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

RailTrack::~RailTrack()
{
  myfile.close();
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

Point RailTrack::getIntersection(const Vec4i &l, const Vec4i &l_2)
{
  Point intersection;
  float fSlope0 = getSlope(l);
  float fSlope1 = getSlope(l_2);
  float fYint0 = getYintersect(l);
  float fYint1 = getYintersect(l_2);
  intersection.y = (((fSlope0 * fYint1) - (fSlope1 * fYint0)) / (fSlope0 - fSlope1));
  intersection.x = ((fYint0 - fYint1) / (fSlope1 - fSlope0));
  return intersection;
}

double RailTrack::doSobel(const Mat &image)
{
  Mat Result;
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  /// Generate grad_x and grad_y
  Mat grad_x, grad_y;
  Mat abs_grad_x, abs_grad_y;

  /// Gradient X
  Sobel(image, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
  convertScaleAbs(grad_x, abs_grad_x);

  /// Gradient Y
  Sobel(image, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
  convertScaleAbs(grad_y, abs_grad_y);

  /// Total Gradient (approximate)
  addWeighted(abs_grad_x, 1, abs_grad_y, 0, 0, Result);
  //cv::Canny(image, Result, 100, 200);
  return mean(Result)[0];
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
      vector<Vec4i> ext_lines = extendLines(l, l_2);

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
  for (int i = 0; i < 3; i++)
    line(m_imgOriginal, poly_points[0][i], poly_points[0][i+1], Scalar(255, 0, 0), 2, CV_AA);
}

void RailTrack::showWindow(const string &title, const Mat &image)
{
  cv::namedWindow(title, CV_WINDOW_AUTOSIZE);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
  cv::imshow(title, image);                   // show windows
}

Mat RailTrack::setROI(const Mat &imgBin)
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

vector<Vec4i> RailTrack::extendLines(const Vec4i &l, const Vec4i &l_2)
{
  vector<Vec4i> final_lines;
  Vec4i extended_lines;
  Point intersection = getIntersection(l, l_2);
  float fSlope0 = getSlope(l);
  float fSlope1 = getSlope(l_2);
  float fYint0 = getYintersect(l);
  float fYint1 = getYintersect(l_2);

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
  Point left_point;
  Point right_point;
  vector<Point> left_rail;
  vector<Point> right_rail;
  Point intersection = getIntersection(m_aTracks[0], m_aTracks[1]);
  int iWmax = m_aTracks[0][2] - m_aTracks[1][0];

  left_curve = imgCanny.cols;
  right_curve = 0;

  if (iWmax > 0)
  {
    if (m_aPrevLines[1][1] == 0)
      left_rail.push_back(Point(m_aTracks[1][0], m_aTracks[1][1]));
    else
      left_rail.push_back(Point(m_aPrevLines[1][0], m_aPrevLines[1][1]));

    if (m_aPrevLines[0][3] == 0)
      right_rail.push_back(Point(m_aTracks[0][2], m_aTracks[0][3]));
    else
      right_rail.push_back(Point(m_aPrevLines[0][2], m_aPrevLines[0][3]));

    for (int i = m_aPrevLines[1][1]; i > intersection.y; i--) //from bottom of the image to the top
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
      {
        left_rail.push_back(left_point);
        left_curve = left_point.x;
      }
    }

    for (int i = m_aPrevLines[0][3]; i > intersection.y; i--) //from bottom of the image to the top
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
      {
        right_rail.push_back(right_point);
        right_curve = right_point.x;
      }
    }

    /// Draw curves
    for( int i = 0; i < left_rail.size(); i++ )
      for (int j = -1; j < 15; j++)
        m_imgOriginal.at<Vec3b>(Point(left_rail.at(i).x + j, left_rail.at(i).y)) = Vec3b(0,255,0);
    for( int i = 0; i < right_rail.size(); i++ )
      for (int j = -14; j < 2; j++)
        m_imgOriginal.at<Vec3b>(Point(right_rail.at(i).x + j, right_rail.at(i).y)) = Vec3b(0,255,0);
  }
}

void RailTrack::getROI()
{
  if (m_aTracks[1][0] != -1 && m_aTracks[1][0] != 0)
  {
    if((abs(prev_track_0 - m_aTracks[0][2]) < 20) && (abs(prev_track_1 - m_aTracks[1][0]) < 20))
    {
      /** Create some points */
      poly_points[0][0]  = Point((m_aTracks[1][0] - ROI_X), m_aTracks[1][1]);

      if(left_curve < m_aTracks[1][2])
        poly_points[0][1]  = Point((left_curve - ROI_Y), (m_aTracks[1][3] - ROI_Y));
      else
        poly_points[0][1]  = Point((m_aTracks[1][2] - ROI_Y), (m_aTracks[1][3] - ROI_Y));

      if(right_curve > m_aTracks[0][0])
        poly_points[0][2]  = Point((right_curve + ROI_Y), (m_aTracks[0][1] - ROI_Y));
      else
        poly_points[0][2]  = Point((m_aTracks[0][0] + ROI_Y), (m_aTracks[0][1] - ROI_Y));

      poly_points[0][3]  = Point((m_aTracks[0][2] + ROI_X), m_aTracks[0][3]);

      prev_track_0 = m_aTracks[0][2];
      prev_track_1 = m_aTracks[1][0];
    }
    if (updateTracks)
    {
      prev_track_0 = m_aTracks[0][2];
      prev_track_1 = m_aTracks[1][0];
      updateTracks = false;
    }
  }
}

void RailTrack::track(const sensor_msgs::ImageConstPtr& msg)
{
  Mat m_untouched = cv_bridge::toCvShare(msg, "bgr8")->image;
  Mat imgGrayscale;       // grayscale of input image
  Mat imgCanny;
  Mat imgROI;
  double gray_mean;
  double sobel_mean;

  m_imgOriginal = m_untouched;
  cvtColor(m_imgOriginal, imgGrayscale, CV_BGR2GRAY);       // convert to grayscale
  gray_mean = mean(imgGrayscale)[0];
  GaussianBlur(imgGrayscale, imgGrayscale, Size(9, 9), 0, 0, BORDER_DEFAULT);
  sobel_mean = doSobel(imgGrayscale);

  Canny(imgGrayscale, imgCanny, 80, m_canny);       // Get Canny Edge
  imgROI = setROI(imgCanny);
  DoHough(imgROI);
  getCurves(imgCanny);
  getROI();

  myfile << gray_mean << "|" << sobel_mean << "|" << line_number << "|" << m_canny << endl;
  cout << gray_mean << "\t" << sobel_mean << "\t" << line_number << "\t" << m_canny << endl;

  if (line_number != 10)
    m_canny = sobel_mean * 251 / 12;

  //showWindow("imgGrayscale", imgGrayscale);
  showWindow("imgCanny", imgCanny);
  //showWindow("imgROI", imgROI);
  showWindow("Tracked", m_imgOriginal);

  sensor_msgs::ImagePtr im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_untouched).toImageMsg();
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
  msg_roi.orig_image = *im_msg;
  waitKey(1);
}
