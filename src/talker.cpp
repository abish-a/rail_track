#include "rail_track/railtrack_img.hpp"
#include "rail_track/railtrack_vid.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "RailTracker");

  if (argc != 2)
  {
    cout << "Please enter file name" << endl;
    return -1;
  }

  string path =  "/home/abish/catkin_ws/src/rail_track/" + string(argv[1]); /////// this needs to be dynamic

  if ((path.find(".mp4") != std::string::npos) || (path.find(".avi") != std::string::npos))
    RailTrack_VID vid_obstacle(path);
  else
  {
    path = "/home/abish/catkin_ws/src/rail_track/Normal/" + string(argv[1]);
    RailTrack_IMG img_obstacle(path);
  }

  return 0;
}
