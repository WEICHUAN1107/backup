#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "util.h"

class Frame;

class MapPoint
{
public:
    MapPoint(int _pid, Frame* _f, Point2f _pdistort, Point2f _pundist);

    void putInFrame(Frame* _f, Point2f _pdistort, Point2f _pundist);

    bool isInFrame(Frame* _f)
    {
        for(auto &f: ownerframe)
        {
            if(f == _f) return true;
        }
        return false;
    }

    vector<Frame*> getOwnerFrames()
    {
        return ownerframe;
    }

    vector<Frame*> getOwnerKeyFrames();
    Frame* getRecentOwnerKeyFrame();
    Frame* getSeedFrame();

    Point2f pdistortInFrame(Frame* _f)
    {
        for(int i=ownerframe.size()-1;i>=0;i--)
        {
            if(_f == ownerframe[i]) return pdistort[i];
        }
        return Point2f(-1,-3);
    }

    Point2f pundistInFrame(Frame* _f)
    {
        for(int i=ownerframe.size()-1;i>=0;i--)
        {
            if(_f == ownerframe[i]) return pundist[i];
        }
        return Point2f(-1,-1);
    }

    void setpw(Point3d p)
    {
        pw = p;
    }

    Point3d getpw()
    {
        return pw;
    }

    int pid;
    Scalar color; // 显示用颜色
    int age;
    bool isBad; // 坏点 默认false
    bool isOnGround;

// private:

    Point3d pw; // 每个特征点的3d点坐标，未知为(0,0,0)
    vector<Frame*> ownerframe; // 在哪些帧中被观测到
    vector<Point2f> pdistort, pundist; // 观测帧对应的图像坐标
};




#endif

