#include "frame.h"
#include "point.h"

Frame::Frame(int _fid, uchar* _imgdata, Size orisize, Size newsize, long long _timestamp)
{
    fid = _fid;
    timestamp = _timestamp;
    iskf = false;
    isStill = true;
    Tcw = Mat();
    twcPt = Point3d(0,0,0);

    Mat img(orisize, CV_8UC1, _imgdata);
    resize(img, im, newsize);

    Mat tmp;
    resize(im, tmp, im.size()/2);
    impyrd.push_back(tmp);
    brightness = mean(tmp)[0]; // 计算亮度
    resize(tmp, tmp, im.size()/4);
    impyrd.push_back(tmp);
    resize(tmp, tmp, im.size()/8);
    impyrd.push_back(tmp);
}


void Frame::deleteMapPoint(MapPoint* _mp)
{
    unique_lock<mutex> lock(MutexFramePts);
    vector<MapPoint*>::iterator elem = find(framePts.begin(), framePts.end(), _mp);
    if(elem != framePts.end())
    {
        framePts.erase(elem);
    }
    else
    {
        cout << "\n\n\n\nTrying to delete a MapPoint which doesn't exist!!! " << _mp->pid << "\n\n\n\n" << endl;
    }
}

