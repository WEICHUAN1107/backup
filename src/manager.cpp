#include "manager.h"
#include "frame.h"
#include "point.h"
#include "camera.h"


Manager::Manager(Camera* _camera, vector<int> _safeborder, int _colgridN, int _rowgridN, float height)
{
    camera = _camera;
    cf = NULL;
    lastf = NULL;
    lastkf = NULL;
    cfid = -1;
    ptid = 0;
    initialized = false;
    notKFcnt = 0;

    K = camera->KNew.clone();
    Kinv = K.inv();
    cx = K.at<double>(0,2);
    cy = K.at<double>(1,2);
    fx = K.at<double>(0,0);
    fy = K.at<double>(1,1);
    imw = camera->newsize.width;
    imh = camera->newsize.height;
    safetop = _safeborder[0];
    safebottom = imh - _safeborder[1];
    safeleft = _safeborder[2];
    saferight = imw - _safeborder[3];
    colgridN = _colgridN;
    rowgridN = _rowgridN;

    cam_height = height;
    // if(ground_param.size())
    //     ground_param.clear();
    ground_param.resize(5, -1);
    // ground_param.push_back(0);
    vanishing_founded = false;
    boundary1 = Point3f(0, 0.8*imh, 1).cross(Point3f(0.5*imw, 0.4*imh, 1));
    boundary1 /= norm(Point2f(boundary1.x, boundary1.y));
    if(boundary1.dot(Point3f(imw,imh,1))<0)
    {
        boundary1 *= -1;
    }

    boundary2 = Point3f(imw, 0.8*imh, 1).cross(Point3f(0.5*imw, 0.4*imh, 1));
    boundary2 /= norm(Point2f(boundary2.x, boundary2.y));
    if(boundary2.dot(Point3f(0,imh,1))<0)
    {
        boundary2 *= -1;
    }
    counts = 0;
}


Manager::~Manager()
{
    {
        // unique_lock<mutex> lock(MutexallFrames);
        for(auto &f: allFrames) { delete f; f = NULL; }
    }
    {
        // unique_lock<mutex> lock(MutexallMapPoints);
        for(auto &mp: allMapPoints) { delete mp; mp = NULL; }
    }
}


Mat Manager::getimgforshow()
{
    return imgforshow;
}


vector<Frame*> Manager::getRecentKFs(int N)
{
    if(N < 1) N = 1;
    vector<Frame*> ret;
    int cnt = 1;
    for(int i=allFrames.size()-1;i>=0;i--)
    {
        if(allFrames[i]->isKF())
        {
            ret.push_back(allFrames[i]);
            if(cnt++ == N) break;
        }
    }
    return ret;
}


vector<Frame*> Manager::getAllKFs()
{
    vector<Frame*> ret;
    for(auto f: allFrames)
    {
        if(f->isKF())
            ret.push_back(f);
    }
    return ret;
}






