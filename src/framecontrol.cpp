#include "manager.h"
#include "frame.h"
#include "point.h"
#include "camera.h"


void Manager::createNewFrame(uchar* _imgdata, long long timestamp, bool useviewer)
{
    cfid++; // TODO 在主线程之外不要通过cfid寻找某帧，可能出现于cfid++后，new Frame之前。

    if(timestamp == 0)
        timestamp = cfid*40 + 10000;

    cf = new Frame(cfid, _imgdata, camera->orisize, camera->newsize, timestamp);

    {
        // unique_lock<mutex> lock(MutexallFrames);
        allFrames.push_back(cf);
    }

    log.str("");
    log << "fid:" << cfid;

    if(cfid == 0)
        lastf = NULL;
    else
    {
        lastf = getFrame(cfid-1);
        if(lastf->isKF()) lastkf = lastf;
    }

    // 保留最近10个普通帧和最近10个关键帧的图像
    vector<Frame*> frames = getRecentFrames(12);
    if(frames.size() > 10 && !frames[10]->isKF())
    {
        frames[10]->im.release();
        for(auto &img: frames[10]->impyrd) img.release();
    }

    vector<Frame*> keyframes = getRecentKFs(12);
    if(keyframes.size() > 10)
    {
        keyframes[10]->im.release();
        for(auto &img: keyframes[10]->impyrd) img.release();
    }

    if(useviewer)
        cvtColor(cf->im, imgforshow, COLOR_GRAY2BGR);
}


// 决定是否建立关键帧
bool Manager::makeKF()
{
    if(cf->isStill) return false;

    Mat Tck = cf->getTcw() * lastkf->getTwc();
    Mat Rck, rck, tck;
    splitT(Tck, Rck, tck);
    Rodrigues(Rck, rck);
    if(norm(rck)<7/57.3 && norm(tck)<0.05) // 距离上一个关键帧太近且没什么旋转 TODO 绝对数值可能有问题
    {
        // cout << "not moving" << endl;
        return false;
    }

    bool vote = false;

    int shiftth = 50; // 几个像素 TODO 不同分辨率的相机画面自适应

    int cnt = 0;
    for(auto &mp: cf->getMapPoints())
    {
        if(!mp->isBad && mp->isInFrame(lastkf) && distance1(mp->pdistortInFrame(cf),mp->pdistortInFrame(lastkf))>shiftth)
            cnt++;
        if((float)cnt / cf->mpsize() > 0.2) // 超过一定比例的点的运动大于shiftth个像素 点越来越少的话，该条件也一直无法满足
        // if(cnt > 2) // 超过N个点的运动大于shiftth个像素
        {
            log << " KF_A. ";
            vote = true;
            break;
        }
    }

    // 特征点跟丢一大半了
    if(!vote && (float)cf->mpsize()/lastkf->mpsize() < 0.4)
    {
        log << " KF_E. ";
        vote = true;
    }

    if(!vote && notKFcnt < 5) // 关键帧不要距离太近
    {
        cf->setKF(false);
        notKFcnt++;
        return false;
    }

    if(!vote && cf->mpsize() < colgridN*rowgridN*0.4) // 特征点太少
    {
        log << " KF_B. ";
        vote = true;
    }

    if(!vote)
    {
        vector<MapPoint*> mps;
        int xmin=2000, xmax=0, ymin=2000, ymax=0;
        for(auto mp: cf->getMapPoints())
        {
            if(mp->getpw() == Point3d(0,0,0)) continue;
            Point2f pd = mp->pdistortInFrame(cf);
            if(pd.x < xmin) xmin = pd.x;
            if(pd.x > xmax) xmax = pd.x;
            if(pd.y < ymin) ymin = pd.y;
            if(pd.y > ymax) ymax = pd.y;
            if(!mp->isBad) mps.push_back(mp);
        }

        if(mps.size() < colgridN*rowgridN*0.3) // 3d点快要不够用了
        {
            log << " KF_C. ";
            vote = true;
        }

        if(xmax-xmin < (saferight-safeleft)*0.7 || ymax-ymin < (safebottom-safetop)*0.7) // 3d点在画面分布不够分散
        {
            log << " KF_D. ";
            vote = true;
        }
    }

    if(vote)
    {
        cf->setKF(true);
        notKFcnt = 0;
        return true;
    }
    else
    {
        cf->setKF(false);
        notKFcnt++;
        return false;
    }
}


void Manager::KFCulling()
{
    vector<Frame*> kfs = getRecentKFs(3);
    if(kfs.size() < 3) return;

    vector<MapPoint*> mps = kfs[1]->getMapPoints();
    int commoncnt = 0;
    for(auto &mp: mps)
    {
        if(mp->isInFrame(kfs[0]) && mp->isInFrame(kfs[2]))
            commoncnt++;
    }
    if((float)commoncnt/mps.size() > 0.7)
    {
        // cout << "culling " << kfs[1]->fid << endl;
        kfs[1]->setKF(false);
    }
}


void Manager::prepareBA()
{
    vKF = getRecentKFs(8); // 最近N个关键帧会被BA优化
    cfidBA = cfid;
    cfBA = cf;
    lastkfBA = lastkf;
    log << "  BA";
}





