#include "rail_track/railtrack.hpp"

RailTrack::RailTrack() :
  loop_rate(10)
{
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 4; j++)
      m_aLongestLines[i][j] = 0;
  m_P = Point(0,0);
  lines_pub = n.advertise<geometry_msgs::Point>("lines", 1000);
}

float RailTrack::getYintersect(const Vec4f &line)
{
  if (line[2] == line[0])
    return numeric_limits<float>::max();
  else
    return (((line[2] * line[1]) - (line[0] * line[3])) / (line[2] - line[0]));
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
      m_aLongestLines[i][j] = 0;

  vector<Vec4i> lines;

  HoughLinesP(dst, lines, 2, CV_PI / 180, 50, 50, 10);
  for (size_t i = 0; i < lines.size(); i++)
  {
    Vec4i l = lines[i];

    //line(m_imgOriginal, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 255), 2, CV_AA);
    //cout << "length: " << getLength(l) << "\tslope: " << getSlope(l) << "\tpoints: " << Point(l[0], l[1]) << "\t" << Point(l[2], l[3]) << endl;

    if (getSlope(l) >= 1) //to get the right track
    {
      for (size_t j = 0; j < lines.size(); j++)
      {
        Vec4i l_2 = lines[j];
        if (getSlope(l_2) <= -1) //to get the left track
        {
          if (((abs(l[0] - l_2[2]) > DMIN) && (abs(l[2] - l_2[0])) < DMAX))
          {
            if ((getLength(l) >= getLength(m_aLongestLines[0])) && (getLength(l_2) >= getLength(m_aLongestLines[1])))
            {
              //cout << abs(l[0] - l_2[2]) << "\t" << abs(l[2] - l_2[0]) << endl;
              m_aLongestLines[0] = l; // right track
              m_aLongestLines[1] = l_2; //left track
            }
          }
        }
      }
    }
  }
}

void RailTrack::showWindow(const string &title, const Mat &image)
{
  cv::namedWindow(title, CV_WINDOW_AUTOSIZE);	// note: you can use CV_WINDOW_NORMAL which allows resizing the window
  cv::imshow(title, image);                   // show windows
}

Mat RailTrack::getTrackFeatures()
{
  m_iYmax = 0;
  float fSlope1 = getSlope(m_aLongestLines[0]);
  float fSlope2 = getSlope(m_aLongestLines[1]);
  float fYint1 = getYintersect(m_aLongestLines[0]);
  float fYint2 = getYintersect(m_aLongestLines[1]);
  m_P.y = (((fSlope1 * fYint2) - (fSlope2 * fYint1)) / (fSlope1 - fSlope2));
  m_P.x = ((fYint1 - fYint2) / (fSlope2 - fSlope1));

  m_iYmax = m_imgOriginal.size().height;

  m_aTracks[0][0] = m_P.x;
  m_aTracks[0][1] = m_P.y;
  m_aTracks[0][2] = (m_iYmax - fYint1)/fSlope1;
  m_aTracks[0][3] = m_iYmax;
  m_aTracks[1][0] = (m_iYmax - fYint2)/fSlope2;
  m_aTracks[1][1] = m_iYmax;
  m_aTracks[1][2] = m_P.x;
  m_aTracks[1][3] = m_P.y;

  ///////////////////////////////////////////////////////////////////////////////
  // Drawing line on the image
  line(m_imgOriginal, Point(m_aLongestLines[0][0], m_aLongestLines[0][1]), Point(m_aLongestLines[0][2], m_aLongestLines[0][3]), Scalar(0, 0, 255), 2, CV_AA);
  line(m_imgOriginal, Point(m_aLongestLines[1][0], m_aLongestLines[1][1]), Point(m_aLongestLines[1][2], m_aLongestLines[1][3]), Scalar(0, 0, 255), 2, CV_AA);

  //line(m_imgOriginal, Point(m_aTracks[0][0], m_aTracks[0][1]), Point(m_aTracks[0][2], m_aTracks[0][3]), Scalar(0, 0, 255), 2, CV_AA);
  //line(m_imgOriginal, Point(m_aTracks[1][0], m_aTracks[1][1]), Point(m_aTracks[1][2], m_aTracks[1][3]), Scalar(0, 0, 255), 2, CV_AA);

  m_Wmax = abs(m_aTracks[0][2] - m_aTracks[1][0]);
  circle(m_imgOriginal, m_P, 1, Scalar(0, 255, 0), 3, CV_AA);
  //line(image, Point(m_aTracks[1][0], m_aTracks[1][1]), Point(m_aTracks[0][2], m_aTracks[0][3]), Scalar(255, 0, 0), 2, CV_AA);

  line(m_imgOriginal, poly_points[0][0], poly_points[0][1], Scalar(255, 0, 0), 2, CV_AA);
  line(m_imgOriginal, poly_points[0][1], poly_points[0][2], Scalar(255, 0, 0), 2, CV_AA);
  line(m_imgOriginal, poly_points[0][2], poly_points[0][3], Scalar(255, 0, 0), 2, CV_AA);

  return m_imgOriginal;
}

Mat RailTrack::region_of_interest(const Mat &imgCanny)
{
  Mat imgPoly(imgCanny.size(), imgCanny.type());
  imgPoly.setTo(0);
  Mat Result;

  int lineType = LINE_8;
  const Point* ppt[1] = { poly_points[0] };
  int npt[] = { 4 };

  fillPoly(imgPoly, ppt,npt,1, Scalar( 255, 255, 255 ), lineType);

  bitwise_and(imgPoly, imgCanny, Result);
  return Result;
}

void RailTrack::getContours(const Mat &imgCanny)
{
  vector<vector<Point>> contours;
  vector<vector<Point>> rail_contour;
  vector<Vec4i> hierarchy;

  min = Point(imgCanny.cols, imgCanny.rows);
  max = Point(0,0);

  /// Find contours
  findContours( imgCanny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  for( int i = 0; i< contours.size(); i++ )
  {
    for (int j = 0; j < contours.at(i).size(); j++)
    {
      if ((contours.at(i).at(j).x == m_aLongestLines[0][0]) && (contours.at(i).at(j).y == m_aLongestLines[0][1]))
      {
        rail_contour.push_back(contours.at(i));
        break;
      }
      else if ((contours.at(i).at(j).x == m_aLongestLines[1][2]) && (contours.at(i).at(j).y == m_aLongestLines[1][3]))
      {
        rail_contour.push_back(contours.at(i));
        break;
      }
    }
  }

  for( int i = 0; i< rail_contour.size(); i++ )
  {
    for (int j = 0; j < rail_contour.at(i).size(); j++)
    {
      if (rail_contour.at(i).at(j).x < min.x)
        min.x = rail_contour.at(i).at(j).x;
      else if (rail_contour.at(i).at(j).x > max.x)
        max.x = rail_contour.at(i).at(j).x;
      else if (rail_contour.at(i).at(j).y < min.y)
        min.y = rail_contour.at(i).at(j).y;
      else if (rail_contour.at(i).at(j).y > max.y)
        max.y = rail_contour.at(i).at(j).y;
    }
  }

  /// Draw contours
  for( int i = 0; i< rail_contour.size(); i++ )
  {
    drawContours( m_imgOriginal, rail_contour, i, Scalar(0,0,255), 2, 8, hierarchy, 0, Point() );
  }
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
  GaussianBlur(imgGrayscale, imgGrayscale, Size(5, 5), 0, 0, BORDER_DEFAULT);

  //imgBinary = DoSobel(imgGrayscale);						// Get Sobel Gradient
  Canny(imgGrayscale, imgCanny, 80, 200, 3);       // Get Canny Edge

  imgROI = region_of_interest(imgCanny);

  DoHough(imgROI);
  imgHough = getTrackFeatures();

  getContours(imgCanny);

  //showWindow("imgGrayscale", imgGrayscale);
  //showWindow("imgCanny", imgCanny);
  //showWindow("imgROI", imgROI);
  showWindow("imgHough", imgHough);

  msg_tracks.x = m_P.x;
  msg_tracks.y = m_P.y;
  msg_tracks.z = m_Wmax;
  lines_pub.publish(msg_tracks);
  ros::spinOnce();
  loop_rate.sleep();
}
