#ifndef VIEWER_H
#define VIEWER_H

#include "util.h"
#include <pangolin/pangolin.h>

class Frame;
class MapPoint;

class Viewer
{
public:
    Viewer(int imgw, int imgh);

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

    void update(vector<Frame*>& frames, vector<MapPoint*>& mappoints, Mat video, std::vector<float> ground_param);

private:
    pangolin::OpenGlRenderState s_cam;
    pangolin::View d_cam;
    pangolin::View d_video;
    pangolin::OpenGlMatrix camT;
    pangolin::OpenGlMatrix viewcam;

    void drawPoints(vector<Frame*>& frames, vector<MapPoint*>& mappoints);
    void drawTrack(vector<Frame*>& frames, vector<MapPoint*>& mappoints);

};




#endif
