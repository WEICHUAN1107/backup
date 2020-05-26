#include "manager.h"
#include "frame.h"
#include "point.h"
#include "opticalflow.h"
#include "camera.h"


bool Manager::pointsFlow()
{
    if(lastf == NULL) return true;

    Mat Tcw_motion, lastfTwc;
    Point3d lastfOw(0,0,0);
    if(!getVelocity().empty() && !lastf->getTcw().empty())
    {
        Tcw_motion = getVelocity() * lastf->getTcw();
        lastfOw = lastf->getOwPt();
        lastfTwc = lastf->getTwc();
    }

    // 光流跟踪上一帧的点
    vector<MapPoint*> mps = lastf->getMapPoints();
    if(mps.size() == 0) return false;
    vector<Point2f> prevpts, prevptscpy, guessflow, trackpts; // 与mps一一对应
    for(auto &mp: mps)
    {
        Point2f plast = mp->pdistortInFrame(lastf);
        prevpts.push_back(plast); // before
        if(mp->isInFrame(getFrame(cfid-2)))
            guessflow.push_back(plast - mp->pdistortInFrame(getFrame(cfid-2)));
        else
            guessflow.push_back(Point2f(-1,-1));
    }
    prevptscpy = prevpts;

    int Nsuccess = trackPoints(lastf->im, lastf->impyrd, cf->im, cf->impyrd, cf->brightness-lastf->brightness, prevpts, guessflow, trackpts, Mat());

    // 判断当前帧和上一帧是否几乎一样。一样的话对极点不可用。TODO 极慢速度会误认为静止
    int movecnt = 0;
    bool stillframe = true;
    for(int i=0;i<trackpts.size();i++)
    {
        if(trackpts[i].x>0 && distance1(trackpts[i], prevptscpy[i])>1)
        {
            movecnt++;
            if(movecnt > Nsuccess*0.1)
            {
                stillframe = false;
                break;
            }
        }
    }
    // std::cout << movecnt << " " << Nsuccess << std::endl;
    if(stillframe)
    {
        cf->isStill = true;
        if(!imgforshow.empty()) cv::circle(imgforshow, Point2f(30,30), 1, cv::Scalar(0,0,255), 20, cv::LINE_AA);
    }
    else
    {
        cf->isStill = false;
        if(!imgforshow.empty()) cv::circle(imgforshow, Point2f(30,30), 1, cv::Scalar(0,255,0), 20, cv::LINE_AA);
    }



    if(!imgforshow.empty())
    {
        Point2f p1 = Point2f(safeleft, safetop);
        Point2f p2 = Point2f(saferight, safetop);
        Point2f p3 = Point2f(saferight, safebottom);
        Point2f p4 = Point2f(safeleft, safebottom);
        cv::line(imgforshow, p1, p2, cv::Scalar(0,255,0), 1, cv::LINE_4);
        cv::line(imgforshow, p2, p3, cv::Scalar(0,255,0), 1, cv::LINE_4);
        cv::line(imgforshow, p3, p4, cv::Scalar(0,255,0), 1, cv::LINE_4);
        cv::line(imgforshow, p4, p1, cv::Scalar(0,255,0), 1, cv::LINE_4);
    }


    // 在当前帧中加入光流成功的点，放弃太拥挤的点
    int OFbegin = trackpts.size();
    int OFleft = 0;
    for(int i=0;i<trackpts.size();i++)
    {
        if(mps[i]->isBad) continue;

        // 特征移动到画面边缘时主动丢掉
        if(trackpts[i].x<safeleft || trackpts[i].y<safetop || trackpts[i].x>saferight || trackpts[i].y>safebottom) continue;

        // 成功跟踪的点
        mps[i]->putInFrame(cf, trackpts[i], camera->myundistortPoint(trackpts[i]));

        if(!imgforshow.empty())
        {
            // if(mps[i]->getpw() != Point3d(0,0,0))
            //     rectangle(imgforshow, Rect(trackpts[i].x-5,trackpts[i].y-5,11,11), mps[i]->color, 2, cv::LINE_4);
            // else
            //     rectangle(imgforshow, Rect(trackpts[i].x-4,trackpts[i].y-4,9,9), mps[i]->color, 1, cv::LINE_4);
            if(mps[i]->getpw() != Point3d(0,0,0))
                rectangle(imgforshow, Rect(trackpts[i].x-5,trackpts[i].y-5,11,11), Scalar(0,200,200), 2, cv::LINE_4);
            else
                rectangle(imgforshow, Rect(trackpts[i].x-4,trackpts[i].y-4,9,9), Scalar(200,200,0), 1, cv::LINE_4);
            // stringstream ss; ss << mps[i]->pid;
            // putText(imgforshow, ss.str(), Point2f(trackpts[i].x, trackpts[i].y-6), FONT_HERSHEY_PLAIN, 1, mps[i]->color, 1, LINE_AA);
        }

        OFleft++;
    }
    if(OFleft == 0) return false;
    log << "  OF:" << OFbegin << "pts," << OFbegin - OFleft << "lost,";
    if(cf->isStill) log << " still, ";
    return true;
}



void Manager::recoverOnePoint(MapPoint* mp, Mat Tcw)
{
    // 从上一次可见帧光流到当前帧，若光流后的2d位置跟3d重投影的位置差不多，则认为是同一个点
    Point3d pc = pointMultiply(Tcw, mp->getpw());
    if(pc.z <= 0) return; // 在相机后面
    pc = pointMultiply(K, pc);
    Point2f pd = camera->mydistortPoint(Point2f(pc.x/pc.z, pc.y/pc.z));
    Point2f pd0 = pd;
    if(pd.x>safeleft && pd.x<saferight && pd.y>safetop && pd.y<safebottom)
    {
        Frame* refframe = mp->getRecentOwnerKeyFrame();
        Point2f pt = mp->pdistortInFrame(refframe);
        OFForwardBackwardTracking(refframe->im, refframe->impyrd, cf->im, cf->impyrd, cf->brightness-refframe->brightness, pt, pd, Size(7,7), 0, 0.5, true);
        if(pd.x>safeleft && pd.x<saferight && pd.y>safetop && pd.y<safebottom && norm(pd0-pd)<10)
        {
            mp->putInFrame(cf, pd, camera->myundistortPoint(pd));
            mp->color = Scalar(0,128,255);
        }
    }
}


class recoverPtsInvoker : public ParallelLoopBody
{
public:
    recoverPtsInvoker(Manager* _m, vector<MapPoint*>& _mps, Mat& _Tcw): mps(_mps), Tcw(_Tcw) { m = _m; }

    void operator()(const Range& range) const
    {
        for(int i=range.start; i<range.end; i++)
            m->recoverOnePoint(mps[i], Tcw);
    }

private:
    vector<MapPoint*> mps;
    Mat Tcw;
    Manager* m;
};


// 光流跟丢的点或被遮挡的点，如果3d坐标已经收敛并且还在画面内，可以重新找回
void Manager::recoverRecentLostPts()
{
    if((float)cf->mpsize()/lastkf->mpsize() > 0.5) return;
    // cout << "recover points" << endl;

    vector<MapPoint*> recentLostPts;
    for(auto &kf: getRecentKFs(5))
    {
        for(auto &mp: kf->getMapPoints())
        {
            if( !mp->isBad && !mp->isInFrame(cf) && mp->getpw()!=Point3d(0,0,0) &&
                find(recentLostPts.begin(), recentLostPts.end(), mp)==recentLostPts.end())
            recentLostPts.push_back(mp);
        }
    }

    Mat Tcw = cf->getTcw();
    parallel_for_(Range(0, recentLostPts.size()), recoverPtsInvoker(this, recentLostPts, Tcw));
    // for(int i=0; i<recentLostPts.size(); i++) recoverOnePoint(recentLostPts[i], Tcw);
}








