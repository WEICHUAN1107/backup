#ifndef FRAME_H
#define FRAME_H

#include "util.h"


class MapPoint;

class Frame
{
public:
    Frame(int _fid, uchar* _imgdata, Size orisize, Size newsize, long long _timestamp);

    void addMapPoint(MapPoint* _mp) // 该函数不要显式调用，应由MapPoint维护
    {
        unique_lock<mutex> lock(MutexFramePts);
        // assert(count(framePts.begin(), framePts.end(), _mp) == 0);
        framePts.push_back(_mp);
    }

    void deleteMapPoint(MapPoint* _mp);

    vector<MapPoint*> getMapPoints()
    {
        unique_lock<mutex> lock(MutexFramePts);
        return framePts;
    }

    int mpsize()
    {
        unique_lock<mutex> lock(MutexFramePts);
        return framePts.size();
    }

    void setTcw(Mat _Tcw)
    {
        assert(this!=NULL);
        unique_lock<mutex> lock(MutexTcw);
        Tcw = _Tcw.clone();
        Twc = Tcw.inv();
        Rcw = Tcw(Range(0,3),Range(0,3));
        tcw = Tcw(Range(0,3),Range(3,4));
        twc = -Rcw.t() * tcw;
        twcPt = Point3d(twc.at<double>(0), twc.at<double>(1), twc.at<double>(2));
    }

    Mat getTcw()
    {
        assert(this!=NULL);
        unique_lock<mutex> lock(MutexTcw);
        return Tcw.clone();
    }

    Mat getTwc()
    {
        assert(this!=NULL);
        unique_lock<mutex> lock(MutexTcw);
        return Twc.clone();
    }

    Mat getRcw()
    {
        assert(this!=NULL);
        unique_lock<mutex> lock(MutexTcw);
        return Rcw.clone();
    }

    Mat gettcw()
    {
        assert(this!=NULL);
        
        unique_lock<mutex> lock(MutexTcw);

        return tcw.clone();
    }

    Vec3f gettwc()
    {
        assert(this!=NULL);
        Vec3f ret;
        unique_lock<mutex> lock(MutexTcw);
        ret[0] = tcw.at<double>(0);
        ret[1] = tcw.at<double>(1);
        ret[2] = tcw.at<double>(2);
        return ret;
    }

    Point3d getOwPt()
    {
        assert(this!=NULL);
        unique_lock<mutex> lock(MutexTcw);
        return twcPt;
    }

    void setKF(bool b)
    {
        assert(this!=NULL);
        // unique_lock<mutex> lock(Mutexiskf);
        iskf = b;
    }

    bool isKF()
    {
        assert(this!=NULL);
        // unique_lock<mutex> lock(Mutexiskf);
        return iskf;
    }

    int fid;
    long long timestamp;
    Mat im; // 每帧图像
    vector<Mat> impyrd;
    bool isStill; // 相对上一帧变化大不大，主要用于判断对极点是否可用
    float brightness;

private:
    bool iskf; // 是否为关键帧
    Mat Tcw; // 未知为空Mat
    Mat Twc, Rcw, tcw, twc;
    Point3d twcPt;
    vector<MapPoint*> framePts; // 该帧图像上有哪些MapPoint

    // mutex Mutexiskf;
    mutex MutexTcw;
    mutex MutexFramePts;
};







#endif