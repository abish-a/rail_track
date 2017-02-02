#ifndef RAILTRACK_VID_H
#define RAILTRACK_VID_H

#include "railtrack.hpp"

class RailTrack_VID : public RailTrack
{
public:
    RailTrack_VID(const string &path);

private:
    VideoCapture m_video;
};

#endif // RAILTRACK_VID_H
