#include "manager.h"
#include "frame.h"
#include "point.h"
#include "camera.h"

// #define cfBA cf // 在doProcess里做mapping时要取消注释此行 在bundle里做mapping就注释掉此行

void Manager::mappingOnePoint(MapPoint* mp, Mat cfTcw)
{
    if(mp->isBad) return;

    // 已收敛点排除错误
    if(mp->getpw()!=Point3d(0,0,0))
    {
        Point3d pc = pointMultiply(K, pointMultiply(cfTcw, mp->getpw()));
        Point2f pu(pc.x/pc.z, pc.y/pc.z);
        if(distance1(pu, mp->pundistInFrame(cfBA)) > 3)
        {
            // cout << 'b' << flush;
            mp->isBad = true;
        }
        return;
    }

    Frame* seedframe = mp->getSeedFrame();
    if(seedframe == cfBA || seedframe == NULL) return;

    Point2f p1 = mp->pundistInFrame(seedframe);
    Point2f p2 = mp->pundistInFrame(cfBA);
    Mat f1(Matx31d((p1.x-cx)/fx, (p1.y-cy)/fy, 1));
    Mat f2(Matx31d((p2.x-cx)/fx, (p2.y-cy)/fy, 1));

    Mat rfTwc = seedframe->getTwc();
    Mat Tcr = cfTcw * rfTwc, Rcr, tcr;
    splitT(Tcr, Rcr, tcr);
    Mat f3 = Rcr * f1;
    float parallax = acos(f3.dot(f2)/norm(f3)/norm(f2));
    if(parallax < 1/57.3) return; // 视差角大于1度就认为比较可靠了

    // d1*x1 = d2*R*x2+t
    Mat A(Matx32d(f3.at<double>(0), f2.at<double>(0),
                  f3.at<double>(1), f2.at<double>(1),
                  f3.at<double>(2), f2.at<double>(2)));
    Mat At = A.t();
    Mat AtA = At * A;
    Mat dd = -AtA.inv() * At * tcr;
    float depth0 = dd.at<double>(0);
    float depth1 = dd.at<double>(1);
    if(depth0 < 0 || depth1 > 0)
    {
        mp->isBad = true;
        return;
    }

    Point3d p3d_mono = pointMultiply(rfTwc, pointMultiply(Kinv, Point3d(p1.x*depth0, p1.y*depth0, depth0)));
    mp->setpw(p3d_mono);

    if(!imgforshow.empty())
    {
        Point2f p = mp->pdistortInFrame(cfBA);
        rectangle(imgforshow, Rect(p.x-7,p.y-7,15,15), mp->color, 2, cv::LINE_4);
    }
}

class mappingInvoker : public ParallelLoopBody
{
public:
    mappingInvoker(Manager* _m, vector<MapPoint*>& _mps, Mat& _Tcw): mps(_mps), Tcw(_Tcw) { m = _m; }

    void operator()(const Range& range) const
    {
        for(int i=range.start; i<range.end; i++)
            m->mappingOnePoint(mps[i], Tcw);
    }

private:
    vector<MapPoint*> mps;
    Mat Tcw;
    Manager* m;
};

void Manager::mapping()
{
    Mat Tcw = cfBA->getTcw();
    // for(auto &mp: cfBA->getMapPoints()) mappingOnePoint(mp, Tcw);
    vector<MapPoint*> mps = cfBA->getMapPoints();
    parallel_for_(Range(0, mps.size()), mappingInvoker(this, mps, Tcw));
}





