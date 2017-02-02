#ifndef RAILTRACK_IMG_H
#define RAILTRACK_IMG_H

#include "railtrack.hpp"

class RailTrack_IMG : public RailTrack
{
public:
    RailTrack_IMG(const string &path);

private:
    Mat m_imgOriginal;                   // input image
};

#endif // RAILTRACK_IMG_H
