/*!
 * \file	rail_track
 * \author	Abish Asphandiar <abish@uni-bremen.de> 3020632
 * \date	19-October-2016
 * \brief Initializing the object to subscribe
 */

#include "rail_track/railtrack.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "RailTrackerMain");
  RailTrack roi;
  return 0;
}
