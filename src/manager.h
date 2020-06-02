#ifndef MANAGER_H
#define MANAGER_H

#include "util.h"


class Camera;
class Frame;
class MapPoint;

class Manager
{
public:
    Manager(Camera* _camera, vector<int> _safeborder, int _colgridN, int _rowgridN, float height);
    ~Manager();

    // frame control
    void createNewFrame(uchar* _imgdata, long long timestamp, bool useviewer);
    bool makeKF();
    void KFCulling();

    // tracking
    bool tracking();
    bool solvePnP(Mat& R, Mat& t);

    void calcEndpt();
    int computeRcv(uchar* _imgdata);

    

    // points match
    bool pointsFlow();
    void recoverOnePoint(MapPoint* mp, Mat Tcw);
    void recoverRecentLostPts();

    // point selection
    void addOrRemovePoints();

    // initialization
    bool initializeMap();

    // mapping
    void mappingOnePoint(MapPoint* mp, Mat cfTcw);
    void mapping();

    // BA
    void prepareBA();
    void BundleAdjustment();

    // io
    char* addFrameMsg(Frame* f);
    void outputMsg(char*& outdata);
    int getFramePoints(long long _timestamp, char* outdata);

    Mat getimgforshow();

    void getAllFrames(vector<Frame*>& v_out)
    {
        // unique_lock<mutex> lock(MutexallFrames);
        v_out.assign(allFrames.begin(), allFrames.end());
    }

    vector<Frame*> getAllKFs();

    vector<Frame*> getRecentFrames(int N)
    {
        vector<Frame*> ret;
        int cnt = 1;
        for(int i=allFrames.size()-1;i>=0;i--)
        {
            ret.push_back(allFrames[i]);
            if(cnt++ == N) break;
        }
        return ret;
    }

    void getRecentFrames(vector<Frame*>& outdata, int N)
    {
        int cnt = 1;
        for(int i=allFrames.size()-1; i>-1; --i)
        {
            outdata.push_back(allFrames[i]);
            if(cnt++ == N) break;
        }
    }

    Frame* getFrame(int i)
    {
        // unique_lock<mutex> lock(MutexallFrames);
        if(i < allFrames.size())
            return allFrames[i];
        else
            return NULL;
    }

    vector<Frame*> getRecentKFs(int N);


    // vector<MapPoint*> getAllMapPoints()
    // {
    //     // unique_lock<mutex> lock(MutexallMapPoints);
    //     return allMapPoints;
    // }

    void getAllMapPoints(vector<MapPoint*>& v_out)
    {
        // unique_lock<mutex> lock(MutexallMapPoints);
        v_out.assign(allMapPoints.begin(), allMapPoints.end());
    }

    MapPoint* getMapPoint(int i)
    {
        // unique_lock<mutex> lock(MutexallMapPoints);
        if(i < allMapPoints.size())
            return allMapPoints[i];
        else
            return NULL;
    }

    void setVelocity(Mat vel)
    {
        unique_lock<mutex> lock(MutexVelocity);
        velocity = vel.clone();
    }

    Mat getVelocity()
    {
        unique_lock<mutex> lock(MutexVelocity);
        return velocity.clone();
    }


    // ground
    bool updateGround();
    std::vector<float> getground()
    {
        return ground_param;
    }

    Mat getRcv();

    bool initialized;
    stringstream log;
    bool vanishing_founded;

    Point3f boundary1, boundary2;

    Mat R_cam_vec;

private:

    Frame* cf; // current frame
    Frame* lastf; // last frame
    Frame* lastkf; // last keyframe

    int cfid; // 当前帧号 cf = current frame
    long ptid; // 点的id

    Camera* camera;
    Mat K, Kinv;
    double cx, cy, fx, fy;

    Mat velocity;
    mutex MutexVelocity;
    int notKFcnt; // 已经连续几帧不是关键帧

    int imw, imh;
    int safetop, safebottom, safeleft, saferight; // 光流点出此范围会被丢掉
    int colgridN, rowgridN; // 水平和竖直方向分别划分几个格子来提取特征点

    Mat imgforshow;

    vector<Frame*> allFrames;
    vector<MapPoint*> allMapPoints;
    // mutex MutexallFrames;
    // mutex MutexallMapPoints;

    // used by BA
    int cfidBA;
    Frame* cfBA;
    Frame* lastkfBA;
    vector<Frame*> vKF;

    float cam_height;
    std::vector<float> ground_param;
    std::vector<Vec4f> history_plane;
    Point2f vanishing_pt;
    Point2f dist_vanishing_pt;
    vector<Point2f> pts_on_ground;
    unsigned counts;
    
};








#endif
