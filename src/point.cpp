#include "point.h"
#include "frame.h"



MapPoint::MapPoint(int _pid, Frame* _f, Point2f _pdistort, Point2f _pundist)
{
    assert(_pdistort.x>0);
    pid = _pid;
    color = Scalar(255,0,255);
    pw = Point3d(0,0,0);
    age = 0;
    isBad = false;
    ownerframe.push_back(_f);
    pdistort.push_back(_pdistort);
    pundist.push_back(_pundist);
    isOnGround = false;
    _f->addMapPoint(this);
}


void MapPoint::putInFrame(Frame* _f, Point2f _pdistort, Point2f _pundist)
{
    assert(_pdistort.x>0);
    ownerframe.push_back(_f);
    pdistort.push_back(_pdistort);
    pundist.push_back(_pundist);
    _f->addMapPoint(this);
    age++;
}


vector<Frame*> MapPoint::getOwnerKeyFrames()
{
    vector<Frame*> ownerkeyframe;
    for(auto f: ownerframe)
    {
        if(f->isKF())
            ownerkeyframe.push_back(f);
    }
    return ownerkeyframe;
}


Frame* MapPoint::getRecentOwnerKeyFrame()
{
    for(int i=ownerframe.size()-1;i>=0;i--)
    {
        if(ownerframe[i]->isKF())
            return ownerframe[i];
    }
    return NULL;
}


Frame* MapPoint::getSeedFrame()
{
    if(ownerframe[0]->isKF())
    {
        return ownerframe[0];
    }
    else
    {
        for(int i=0;i<ownerframe.size();i++)
        {
            if(ownerframe[i]->isKF())
                return ownerframe[i];
        }
    }
    return NULL;
}


