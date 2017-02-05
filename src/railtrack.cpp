#include "rail_track/railtrack.hpp"


RailTrack::RailTrack() :
  loop_rate(10)
{
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 4; j++)
      m_aTracks[i][j] = 0;

  lines_pub = n.advertise<geometry_msgs::Point>("lines", 1000);
  myfile.open ("lines.txt");
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

Mat RailTrack::DoHough(const Mat &dst)
{
  //  for (int i = 0; i < 2; i++)
  //    for (int j = 0; j < 4; j++)
  //      m_aTracks[i][j] = 0;

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
        m_aTracks[0] = ext_lines[0]; // right track
        m_aTracks[1] = ext_lines[1]; //left track
      }
    }
  }

  /// Draw Lines
  line(m_imgOriginal, Point(m_aTracks[0][0], m_aTracks[0][1]), Point(m_aTracks[0][2], m_aTracks[0][3]), Scalar(0, 0, 255), 2, CV_AA);
  line(m_imgOriginal, Point(m_aTracks[1][0], m_aTracks[1][1]), Point(m_aTracks[1][2], m_aTracks[1][3]), Scalar(0, 0, 255), 2, CV_AA);

  line(m_imgOriginal, poly_points[0][0], poly_points[0][1], Scalar(255, 0, 0), 2, CV_AA);
  line(m_imgOriginal, poly_points[0][1], poly_points[0][2], Scalar(255, 0, 0), 2, CV_AA);
  line(m_imgOriginal, poly_points[0][2], poly_points[0][3], Scalar(255, 0, 0), 2, CV_AA);

  return m_imgOriginal;
}

void RailTrack::showWindow(const string &title, const Mat &image)
{
  cv::namedWindow(title, CV_WINDOW_AUTOSIZE);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
  cv::imshow(title, image);                   // show windows
}

Mat RailTrack::region_of_interest(const Mat &imgBin)
{
  Mat imgPoly(imgBin.size(), imgBin.type());
  imgPoly.setTo(0);
  Mat Result;

  int lineType = LINE_8;
  const Point* ppt[1] = { poly_points[0] };
  int npt[] = { 4 };

  fillPoly(imgPoly, ppt,npt,1, Scalar( 255, 255, 255 ), lineType);

  bitwise_and(imgPoly, imgBin, Result);
  return Result;
}

void RailTrack::getContours(const Mat &imgCanny)
{
  vector<vector<Point>> contours;
  vector<vector<Point>> rail_contour;
  vector<Vec4i> hierarchy;
  float slope0 = getSlope(m_aTracks[0]);
  float slope1 = getSlope(m_aTracks[1]);
  float Yint0 = getYintersect(m_aTracks[0]);
  float Yint1 = getYintersect(m_aTracks[1]);

  /// Find contours
  findContours( imgCanny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  for( int i = 0; i< contours.size(); i++ )
  {
    if (contours.at(i).size() > 100)
    {
      for (int j = 0; j < contours.at(i).size(); j++) // anypoint lies on the line
      {
        if (contours.at(i).at(j).y == ((contours.at(i).at(j).x * slope0) + Yint0))
        {
          rail_contour.push_back(contours.at(i));
          break;
        }
        else if (contours.at(i).at(j).y == ((contours.at(i).at(j).x * slope1) + Yint1))
        {
          rail_contour.push_back(contours.at(i));
          break;
        }

        /*
        if ((contours.at(i).at(j).x == m_aTracks[0][0]) && (contours.at(i).at(j).y == m_aTracks[0][1]))
        {
          rail_contour.push_back(contours.at(i));
          break;
        }
        else if ((contours.at(i).at(j).x == m_aTracks[1][2]) && (contours.at(i).at(j).y == m_aTracks[1][3]))
        {
          rail_contour.push_back(contours.at(i));
          break;
        }
        */
      }
    }
  }

  /// Draw contours
  for( int i = 0; i < rail_contour.size(); i++ )
  {
    drawContours( m_imgOriginal, rail_contour, i, Scalar(0,255,0), 2, 8, hierarchy, 0, Point() );
  }

  contours.clear();
  rail_contour.clear();
  hierarchy.clear();
}

vector<Vec4i> RailTrack::extend_lines(Vec4i l, Vec4i l_2)
{
  vector<Vec4i> final_lines;
  Vec4i extended_lines;
  Point intersection;

  float fSlope1 = getSlope(l);
  float fSlope2 = getSlope(l_2);
  float fYint1 = getYintersect(l);
  float fYint2 = getYintersect(l_2);
  intersection.y = (((fSlope1 * fYint2) - (fSlope2 * fYint1)) / (fSlope1 - fSlope2));
  intersection.x = ((fYint1 - fYint2) / (fSlope2 - fSlope1));

  int iYmax = m_imgOriginal.size().height;

  if (abs(intersection.y - 270) < 20)
  {
    extended_lines[0] = intersection.x;
    extended_lines[1] = intersection.y;
    extended_lines[2] = (iYmax - fYint1)/fSlope1;
    extended_lines[3] = iYmax;

    if (abs(abs(intersection.x - extended_lines[2]) - 182) < 20)
      final_lines.push_back(extended_lines);
    else
    {
      for (int i = 0; i < 4;i++)
        extended_lines[i] = 0;
      final_lines.push_back(extended_lines);
    }

    extended_lines[0] = (iYmax - fYint2)/fSlope2;
    extended_lines[1] = iYmax;
    extended_lines[2] = intersection.x;
    extended_lines[3] = intersection.y;

    if (abs(abs(intersection.x - extended_lines[0]) - 56) < 20)
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
    for (int i = 0; i < 4;i++)
      extended_lines[i] = 0;
    final_lines.push_back(extended_lines);
    final_lines.push_back(extended_lines);
  }
  return final_lines;
}

void RailTrack::track(Mat &imgOriginal)
{
  Mat imgGrayscale;       // grayscale of input image
  Mat imgCanny;
  Mat imgROI;
  Mat imgHough;

  m_imgOriginal = imgOriginal;

  //m_imgOriginal = DoCrop(m_imgOriginal);		// Cropping the image as we dont need the upper part
  cvtColor(m_imgOriginal, imgGrayscale, CV_BGR2GRAY);       // convert to grayscale

  double mean = cv::mean(imgGrayscale)[0];

  GaussianBlur(imgGrayscale, imgGrayscale, Size(9, 9), 0, 0, BORDER_DEFAULT);

  if (line_number > 12)
    m_canny += 10;
  else if (line_number < 8)
    m_canny -= 10;

  Canny(imgGrayscale, imgCanny, 0.3*m_canny, m_canny);       // Get Canny Edge 80, 300

  imgROI = region_of_interest(imgCanny);
  imgHough = DoHough(imgROI);

  getContours(imgCanny);

  //showWindow("imgGrayscale", imgGrayscale);
  showWindow("imgCanny", imgCanny);
  //showWindow("imgROI", imgROI);
  showWindow("imgHough", imgHough);

  msg_tracks.x = 500;
  msg_tracks.y = 50;
  msg_tracks.z = 5;
  lines_pub.publish(msg_tracks);
  ros::spinOnce();
  loop_rate.sleep();
}
