#ifndef RAILTRACK_H
#define RAILTRACK_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <limits>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include <time.h>

#define DMAX 115//250//140
#define DMIN 5//30//30

using namespace cv;
using namespace std;

class RailTrack
{
public:
    RailTrack();
    float getYintersect(const Vec4f &line);
    float getSlope(const Vec4f &line);
    float getLength(const Vec4f &line);
    void DoHough(const Mat &dst);
    void showWindow(const string &title, const Mat &image);
    Mat getTrackFeatures();
    Mat region_of_interest(const Mat &imgCanny);
    void getContours(const Mat &imgCanny);
    void track(Mat &imgOriginal);

private:
    Mat m_imgOriginal;
    Vec4i m_aLongestLines[2];// Choosing longest line that represents rail track
    int m_iYmax = 0;
    int m_Wmax = 0;
    Point m_P;

    ros::NodeHandle n;
    ros::Publisher lines_pub;
    ros::Rate loop_rate;
    geometry_msgs::Point msg_tracks;

protected:
    Vec4i m_aTracks[2];
    Point poly_points[1][4];
    Point min;
    Point max;

};

#endif // RAILTRACK_H
