#include "util.h"
#include "opticalflow.h"
#include "subdiv.h"


void LKSingle2D(const Mat& prevImg, const Mat& nextImg, const float beta, const Point2f& prevPt_, Point2f& nextPt_,
            Size winSize, double inverseError)
{
    if(prevPt_.x<0 || nextPt_.x<0) return;

    const int WINMAX = 24;
    const int WW = WINMAX*WINMAX;
    assert(winSize.height < WINMAX && winSize.width < WINMAX);
    const uchar* previm = prevImg.data;
    const uchar* nextim = nextImg.data;
    const int imw = prevImg.cols;
    const int imh = prevImg.rows;
    const int winw = winSize.width;
    const int winh = winSize.height;

    Point2f halfWin((winw-1)*0.5f, (winh-1)*0.5f);

    Point2f prevPt = prevPt_ - halfWin;
    Point2f nextPt = nextPt_ - halfWin;

    Point2i iprevPt;
    iprevPt.x = floor(prevPt.x); // 左上角像素
    iprevPt.y = floor(prevPt.y);
    float a = prevPt.x - iprevPt.x;
    float b = prevPt.y - iprevPt.y;
    float ip00 = (1.f - a)*(1.f - b);
    float ip01 = a*(1.f - b);
    float ip10 = (1.f - a)*b;
    float ip11 = a*b;

    float a11=0, a12=0, a22=0;
    float Ix[WW], Iy[WW], I[WW];
    int beginx = fmax(0, 1-iprevPt.x);
    int endx = fmin(winw, imw-iprevPt.x-2);
    int beginy = fmax(0, 1-iprevPt.y);
    int endy = fmin(winh, imh-iprevPt.y-2);
    for(int y=beginy; y<endy; y++)
    {
        int y0 = (iprevPt.y+y)*imw;
        int y_1 = y0-imw;
        int y1 = y0+imw;
        int y2 = y1+imw;
        int winy = winw*y;
        for(int x=beginx; x<endx; x++)
        {
            int x0 = iprevPt.x+x;
            int dx00 = previm[x0+1+y0] - previm[x0-1+y0];
            int dx01 = previm[x0+2+y0] - previm[x0+y0];
            int dx10 = previm[x0+1+y1] - previm[x0-1+y1];
            int dx11 = previm[x0+2+y1] - previm[x0+y1];
            float ix = (dx00*ip00 + dx01*ip01 + dx10*ip10 + dx11*ip11);

            int dy00 = previm[x0+y1] - previm[x0+y_1];
            int dy01 = previm[x0+1+y1] - previm[x0+1+y_1];
            int dy10 = previm[x0+y2] - previm[x0+y0];
            int dy11 = previm[x0+1+y2] - previm[x0+1+y0];
            float iy = (dy00*ip00 + dy01*ip01 + dy10*ip10 + dy11*ip11);

            int n = winy+x;
            I[n] = previm[x0+y0]*ip00 +
                    previm[x0+1+y0]*ip01 +
                    previm[x0+y1]*ip10 +
                    previm[x0+1+y1]*ip11 + beta;

            Ix[n] = ix;
            Iy[n] = iy;

            a11 += ix*ix;
            a12 += ix*iy;
            a22 += iy*iy;
        }
    }

    float DD = a11*a22 - a12*a12;
    if(DD == 0)
    {
        nextPt_ = Point2f(-1,-1);
        return;
    }
    float D = 1./DD;

    for(int i=0;i<16;i++)
    {
        Point2i inextPt;
        inextPt.x = floor(nextPt.x);
        inextPt.y = floor(nextPt.y);
        a = nextPt.x - inextPt.x;
        b = nextPt.y - inextPt.y;
        float in00 = (1.f - a)*(1.f - b);
        float in01 = a*(1.f - b);
        float in10 = (1.f - a)*b;
        float in11 = a*b;

        float b1 = 0, b2 = 0;
        int beginx = fmax(0, 1-inextPt.x);
        int endx = fmin(winw, imw-inextPt.x-2);
        int beginy = fmax(0, 1-inextPt.y);
        int endy = fmin(winh, imh-inextPt.y-2);
        for(int y=beginy; y<endy; y++)
        {
            int y0 = (inextPt.y+y)*imw;
            int y1 = y0+imw;
            int winy = winw*y;
            for(int x=beginx; x<endx; x++)
            {
                int x0 = inextPt.x+x;
                int n = winy+x;
                float next = nextim[x0+y0]*in00 +
                             nextim[x0+1+y0]*in01 +
                             nextim[x0+y1]*in10 +
                             nextim[x0+1+y1]*in11;
                float it = next - I[n];
                b1 += Ix[n]*it;
                b2 += Iy[n]*it;
            }
        }
        Point2f delta( (a12*b2-a22*b1)*D, (a12*b1-a11*b2)*D );

        nextPt += delta;

        if( delta.ddot(delta) <= 0.001 ) break;
    }
    nextPt_ = nextPt + halfWin;

    if(isnan(nextPt_.x)) { nextPt_ = Point2f(-1,-1); return; }

    if(inverseError < 0)
        return;

    Point2f nextPt_backup = nextPt_;

    // backward
    prevPt = nextPt_ - halfWin;
    nextPt = prevPt_ - halfWin;

    iprevPt.x = floor(prevPt.x); // 左上角像素
    iprevPt.y = floor(prevPt.y);
    a = prevPt.x - iprevPt.x;
    b = prevPt.y - iprevPt.y;
    ip00 = (1.f - a)*(1.f - b);
    ip01 = a*(1.f - b);
    ip10 = (1.f - a)*b;
    ip11 = a*b;

    a11=0; a12=0; a22=0;
    beginx = fmax(0, 1-iprevPt.x);
    endx = fmin(winw, imw-iprevPt.x-2);
    beginy = fmax(0, 1-iprevPt.y);
    endy = fmin(winh, imh-iprevPt.y-2);
    for(int y=beginy; y<endy; y++)
    {
        int y0 = (iprevPt.y+y)*imw;
        int y_1 = y0-imw;
        int y1 = y0+imw;
        int y2 = y1+imw;
        int winy = winw*y;
        for(int x=beginx; x<endx; x++)
        {
            int x0 = iprevPt.x+x;
            int dx00 = nextim[x0+1+y0] - nextim[x0-1+y0];
            int dx01 = nextim[x0+2+y0] - nextim[x0+y0];
            int dx10 = nextim[x0+1+y1] - nextim[x0-1+y1];
            int dx11 = nextim[x0+2+y1] - nextim[x0+y1];
            float ix = (dx00*ip00 + dx01*ip01 + dx10*ip10 + dx11*ip11);

            int dy00 = nextim[x0+y1] - nextim[x0+y_1];
            int dy01 = nextim[x0+1+y1] - nextim[x0+1+y_1];
            int dy10 = nextim[x0+y2] - nextim[x0+y0];
            int dy11 = nextim[x0+1+y2] - nextim[x0+1+y0];
            float iy = (dy00*ip00 + dy01*ip01 + dy10*ip10 + dy11*ip11);

            int n = winy+x;
            I[n] = nextim[x0+y0]*ip00 +
                    nextim[x0+1+y0]*ip01 +
                    nextim[x0+y1]*ip10 +
                    nextim[x0+1+y1]*ip11 - beta;

            Ix[n] = ix;
            Iy[n] = iy;

            a11 += ix*ix;
            a12 += ix*iy;
            a22 += iy*iy;
        }
    }

    DD = a11*a22 - a12*a12;
    if(DD == 0)
    {
        nextPt_ = Point2f(-1,-1);
        return;
    }
    D = 1./DD;

    for(int i=0;i<16;i++)
    {
        Point2i inextPt;
        inextPt.x = floor(nextPt.x);
        inextPt.y = floor(nextPt.y);

        a = nextPt.x - inextPt.x;
        b = nextPt.y - inextPt.y;
        float in00 = (1.f - a)*(1.f - b);
        float in01 = a*(1.f - b);
        float in10 = (1.f - a)*b;
        float in11 = a*b;

        float b1 = 0, b2 = 0;
        int beginx = fmax(0, 1-inextPt.x);
        int endx = fmin(winw, imw-inextPt.x-2);
        int beginy = fmax(0, 1-inextPt.y);
        int endy = fmin(winh, imh-inextPt.y-2);
        for(int y=beginy; y<endy; y++)
        {
            int y0 = (inextPt.y+y)*imw;
            int y1 = y0+imw;
            int winy = winw*y;
            for(int x=beginx; x<endx; x++)
            {
                int x0 = inextPt.x+x;
                int n = winy+x;
                float next = previm[x0+y0]*in00 +
                            previm[x0+1+y0]*in01 +
                            previm[x0+y1]*in10 +
                            previm[x0+1+y1]*in11;
                float it = next - I[n];
                b1 += Ix[n]*it;
                b2 += Iy[n]*it;
            }
        }
        Point2f delta( (a12*b2-a22*b1)*D, (a12*b1-a11*b2)*D );

        nextPt += delta;

        if( delta.ddot(delta) <= 0.001 ) break;
    }
    Point2f backPt = nextPt + halfWin;

    if(isnan(backPt.x)) { nextPt_ = Point2f(-1,-1); return; }

    if(distance1(backPt, prevPt_) < inverseError)
    {
        nextPt_ = nextPt_backup;
    }
    else
    {
        nextPt_ = Point2f(-1,-1);
    }
}


class LKForwardBackwardInvoker : public ParallelLoopBody
{
public:
    LKForwardBackwardInvoker(const Mat& _prevImg, const Mat& _nextImg, const float _beta, const vector<Point2f>& _prevPts,
        vector<Point2f>& _nextPts, Size _winSize, double _inverseError):
        prevImg(_prevImg), nextImg(_nextImg), beta(_beta), prevPts(_prevPts), nextPts(_nextPts), winSize(_winSize),
        inverseError(_inverseError) {}

    void operator()(const Range& range) const
    {
        for(int i=range.start; i<range.end; i++)
            LKSingle2D(prevImg, nextImg, beta, prevPts[i], nextPts[i], winSize, inverseError);
    }

private:
    const Mat& prevImg;
    const Mat& nextImg;
    const vector<Point2f>& prevPts;
    const float beta;
    Size winSize;
    vector<Point2f>& nextPts;
    double inverseError;
};


void LKForwardBackwardTrack(const Mat& prevImg, const Mat& nextImg, const float beta, const vector<Point2f>& prevPts, vector<Point2f>& nextPts,
        Size winSize, double inverseError)
{
    assert(nextPts.size()==0 || nextPts.size()==prevPts.size());

    if(nextPts.size() == 0)
        nextPts = prevPts;

    parallel_for_(Range(0, prevPts.size()), LKForwardBackwardInvoker(prevImg, nextImg, beta, prevPts, nextPts, winSize, inverseError));
}

// 光流跟踪点,跟丢的或者觉得误差太大的点置为(-1,-1) 返回成功的个数
void OFForwardBackwardTracking(Mat& lastimg, vector<Mat>& lastimpyrd, Mat& img, vector<Mat>& impyrd, float beta, vector<Point2f>& pts,
                                vector<Point2f>& ptsout, Size winSize, int pyrd, double inverseError, bool dofinest)
{
    assert(pyrd <= lastimpyrd.size());

    if(ptsout.size()==0)
        ptsout = pts;
    else if(ptsout.size() != pts.size())
        cout << "ptsout.size() != pts.size()" << endl;

    int scale = 1 << pyrd;
    for(int i=0;i<pts.size();i++)
    {
        pts[i] /= scale;
        ptsout[i] /= scale;
    }

    for(int level=pyrd; level>0; level--)
    {
        LKForwardBackwardTrack(lastimpyrd[level-1], impyrd[level-1], beta, pts, ptsout, winSize, inverseError);
        for(int i=0;i<pts.size();i++)
        {
            pts[i] *= 2;
            ptsout[i] *= 2;
        }
    }
    if(dofinest) // 做最大的尺度
        LKForwardBackwardTrack(lastimg, img, beta, pts, ptsout, winSize, inverseError);
}

void OFForwardBackwardTracking(Mat& lastimg, vector<Mat>& lastimpyrd, Mat& img, vector<Mat>& impyrd, float beta, Point2f& pt,
                                Point2f& ptout, Size winSize, int pyrd, double inverseError, bool dofinest)
{
    assert(pyrd <= lastimpyrd.size());

    int scale = 1 << pyrd;
    pt /= scale;
    ptout /= scale;

    for(int level=pyrd; level>0; level--)
    {
        LKSingle2D(lastimpyrd[level-1], impyrd[level-1], beta, pt, ptout, winSize, inverseError);
        pt *= 2;
        ptout *= 2;
    }
    if(dofinest) // 做最大的尺度
        LKSingle2D(lastimg, img, beta, pt, ptout, winSize, inverseError);
}


int getCornerScore(Mat& img, Point2f pt)
{
    // return fabs(img.at<uchar>(pt+Point2f(2,0)) - img.at<uchar>(pt)) +
    //        fabs(img.at<uchar>(pt+Point2f(0,2)) - img.at<uchar>(pt)) +
    //        fabs(img.at<uchar>(pt+Point2f(0,-2)) - img.at<uchar>(pt)) +
    //        fabs(img.at<uchar>(pt+Point2f(-2,0)) - img.at<uchar>(pt));

    return fabs(img.at<uchar>(pt+Point2f(2,0)) - img.at<uchar>(pt+Point2f(-2,0))) +
           fabs(img.at<uchar>(pt+Point2f(0,2)) - img.at<uchar>(pt+Point2f(0,-2)));
}

Point2f _guessFlow(Point2f p, const vector<Point2f>& pts1, const vector<Point2f>& pts2)
{
    assert(pts1.size() == pts2.size());
    float mindis = 99999, secmindis = 99999;
    Point2f minflow(0,0), secminflow(0,0);
    for(int k=0;k<pts1.size();k++)
    {
        if(pts2[k].x < 0) continue;
        float dis = norm(p-pts1[k]);
        if(dis < mindis)
        {
            secmindis = mindis;
            mindis = dis;
            secminflow = minflow;
            minflow = pts2[k] - pts1[k];
        }
        else if(dis < secmindis)
        {
            secmindis = dis;
            secminflow = pts2[k] - pts1[k];
        }
    }
    Point2f pguess = p + (minflow*secmindis + secminflow*mindis) / (mindis+secmindis);
    return pguess;


    // Subdiv subdiv(Rect(0,0,imw,imh));
    // vector<Point2f> estflow;
    // for(int i=0;i<pts1.size();i++)
    // {
    //     if(pts2[i].x>0)
    //     {
    //         subdiv.insert(pts1[i]);
    //         estflow.push_back(pts2[i]-pts1[i]);
    //     }
    // }
    // ptsout.clear();
    // for(int i=0;i<pts.size();i++)
    // {
    //     int nearestid = subdiv.findNearest(pts[i], NULL) - 4;
    //     ptsout.push_back( pts[i] + estflow[nearestid] );
    // }

}

int trackPoints(Mat& img1, vector<Mat>& img1pyrd, Mat& img2, vector<Mat>& img2pyrd, float beta, vector<Point2f>& pts,
                vector<Point2f>& guessflow, vector<Point2f>& ptsout, Mat imgforshow)
{
    assert(!img1.empty() && !img1pyrd.empty() && !img2.empty() && !img2pyrd.empty());
    assert(img1.size() == img2.size());
    assert(img1pyrd.size() == img2pyrd.size());
    assert(pts.size() == guessflow.size());

    ptsout.assign(pts.size(), Point2f(-1,-1));

    int imw = img1.cols;
    int imh = img1.rows;

    struct FeatPoint
    {
        Point2f p, po;
        int score;
        int id;
        FeatPoint(int id, Point2f p, int score) : id(id), p(p), score(score) {}
        FeatPoint(int id, Point2f p, Point2f po, int score) : id(id), p(p), po(po), score(score) {}
    };
    vector<FeatPoint> freshPoints, flowedPoints;

    // 把所有点分成有预测和没有预测两类
    for(int i=0;i<pts.size();i++)
    {
        if(pts[i].x < 15 || pts[i].x > imw-15 || pts[i].y < 15 || pts[i].y > imh-15) continue;
        int score = getCornerScore(img1pyrd[0], pts[i]/2);
        if(guessflow[i] == Point2f(-1,-1))
            freshPoints.push_back(FeatPoint(i, pts[i], score));
        else
            flowedPoints.push_back(FeatPoint(i, pts[i], pts[i]+guessflow[i], score));
    }

    // 选择均匀分布的显著点并计算其光流
    int tabscale = 80; // 80: 8x6 grid
    int tabw = cvCeil((float)imw/tabscale);
    int tabh = cvCeil((float)imh/tabscale);
    vector<int> occupytable(tabw*tabh, 0);
    int Nprompts = 0;

    // 先从有预测的点中选择
    vector<float> dguesses, guessbiases;
    vector<Point2f> prompts, promptsout, promptsoutguess;
    vector<int> promptsid;
    sort(begin(flowedPoints), end(flowedPoints), [](const FeatPoint& lhs, const FeatPoint& rhs){return lhs.score > rhs.score;} );
    for(auto &mp: flowedPoints)
    {
        int x = floor(mp.p.x/tabscale);
        int y = floor(mp.p.y/tabscale);
        if(occupytable[y*tabw+x] == 0)
        {
            // cv::circle(imgforshow, mp.p, 10, cv::Scalar(255,0,0), 1, cv::LINE_AA);
            prompts.push_back(mp.p);
            promptsout.push_back(mp.po);
            promptsid.push_back(mp.id);
            Nprompts++;
            // if(Nprompts > 20) break;
            occupytable[y*tabw+x] = 1;
            if(x<tabw-1) occupytable[y*tabw+x+1] = 1;
            if(x>=1) occupytable[y*tabw+x-1] = 1;
            if(y<tabh-1) occupytable[(y+1)*tabw+x] = 1;
            if(y>=1) occupytable[(y-1)*tabw+x] = 1;
            // std::cout << x << " " << y << std::endl;
            // for(int i=0;i<tabw*tabh;i++)
            // {
            //     cout << occupytable[i] << " ";
            //     if((i+1)%tabw==0) cout << endl;
            // }
            // cout << endl;
        }
    }
    vector<Point2f> promptscopy = prompts;
    vector<Point2f> promptsoutcopy = promptsout;
    promptsoutguess = promptsout;
    OFForwardBackwardTracking(img1, img1pyrd, img2, img2pyrd, beta, prompts, promptsout, Size(7,7), 1, 0.5, true);
    // for(int i=0;i<prompts.size();i++)
    // {
    //     if(promptsout[i].x < 0) promptsout[i] = promptsoutcopy[i];
    //     else promptscopy[i] = Point2f(-1,-1);
    // }
    // OFForwardBackwardTracking(img1, img1pyrd, img2, img2pyrd, beta, promptscopy, promptsout, Size(7,7), 3, 0.5, true);

    for(int i=0;i<prompts.size();i++)
    {
        if(prompts[i].x < 0 || promptsout[i].x < 0) continue;
        pts[promptsid[i]] = Point2f(-1,-1); // 下面的光流跳过这些点
        ptsout[promptsid[i]] = promptsout[i];
        float dguess = distance2(prompts[i], promptsoutguess[i]);
        float dreal = distance2(prompts[i], promptsout[i]);
        float guessbias = dreal/dguess;
        if(guessbias > 0.01 && guessbias < 100) // 连续2帧以上重复会出现inf或nan，故加此限制 TODO: 有重复帧时，dguess会变成0，但实际上应该保留上一次的dguess
        {
            guessbiases.push_back(dreal/dguess);
            dguesses.push_back(dguess);
        }
    }

    // 计算预测与实际光流结果的位移比例，用于修正预测
    float guessbias = 1;
    if(!guessbiases.empty())
    {
        sort(guessbiases.begin(), guessbiases.end());
        guessbias = guessbiases[guessbiases.size()/2];
    }
    // std::cout << "guessbias: " << guessbias << std::endl;

    if(!imgforshow.empty())
    {
        stringstream ss; ss << guessbias;
        putText(imgforshow, ss.str(), Point2f(100,60), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255), 1, LINE_AA);
    }


    // 再从没有预测的新点中选择
    vector<Point2f> prompts2, prompts2out;
    vector<int> prompts2id;
    sort(begin(freshPoints), end(freshPoints), [](const FeatPoint& lhs, const FeatPoint& rhs){return lhs.score > rhs.score;} );
    for(auto &mp: freshPoints)
    {
        int x = floor(mp.p.x/tabscale);
        int y = floor(mp.p.y/tabscale);
        if(occupytable[y*tabw+x] == 0)
        {
            prompts2.push_back(mp.p);
            prompts2id.push_back(mp.id);
            // if(!imgforshow.empty())
            // {
            //     stringstream ss; ss << mp.score;
            //     putText(imgforshow, ss.str(), Point2f(mp.p.x, mp.p.y-6), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0), 2, LINE_AA);
            // }
            Nprompts++;
            // if(Nprompts > 20) break;
            occupytable[y*tabw+x] = 1;
            if(x<tabw-1) occupytable[y*tabw+x+1] = 1;
            if(x>=1) occupytable[y*tabw+x-1] = 1;
            if(y<tabh-1) occupytable[(y+1)*tabw+x] = 1;
            if(y>=1) occupytable[(y-1)*tabw+x] = 1;
            // std::cout << x << " " << y << std::endl;
            // for(int i=0;i<tabw*tabh;i++)
            // {
            //     cout << occupytable[i] << " ";
            //     if((i+1)%tabw==0) cout << endl;
            // }
            // cout << endl;
        }
    }
    OFForwardBackwardTracking(img1, img1pyrd, img2, img2pyrd, beta, prompts2, prompts2out, Size(11,11), 3, 0.5, true);
    for(int i=0;i<prompts2.size();i++)
    {
        if(prompts2[i].x < 0 || prompts2out[i].x < 0) continue;
        pts[prompts2id[i]] = Point2f(-1,-1); // 下面的光流跳过这些点
        ptsout[prompts2id[i]] = prompts2out[i];
    }

    // prompts2 合并入 prompts
    for(int i=0;i<prompts2.size();i++)
    {
        prompts.push_back(prompts2[i]);
        promptsout.push_back(prompts2out[i]);
    }










    // 根据显著点的可靠光流，预测并光流所有点

    // round 1
    for(int i=0;i<pts.size();i++)
    {
        if(pts[i].x<0) continue;
        if(guessflow[i] == Point2f(-1,-1))
            ptsout[i] = _guessFlow(pts[i], prompts, promptsout);
        else
            ptsout[i] = pts[i] + guessflow[i]*guessbias;
        if(!imgforshow.empty())
        {
            Point2f pg = ptsout[i];
            cv::line(imgforshow, pts[i], pg + Point2f(0,-5), cv::Scalar(255,0,0), 1, cv::LINE_4);
            rectangle(imgforshow, Rect(pg.x-4,pg.y-4,9,9), cv::Scalar(255,0,0), 1, cv::LINE_4);
        }
    }
    OFForwardBackwardTracking(img1, img1pyrd, img2, img2pyrd, beta, pts, ptsout, Size(7,7), 0, 0.5, true);
    // vector<Point2f> pts1 = pts, pts2 = ptsout;
    // for(int i=0;i<pts.size();i++)
    // {
    //     if(ptsout[i].x>0)
    //         pts[i] = Point2f(-1,-1); // 下面的光流跳过这些点
    // }

    // // round 2
    // for(int i=0;i<pts.size();i++)
    // {
    //     if(pts[i].x<0) continue;
    //     ptsout[i] = _guessFlow(pts[i], pts1, pts2);
    //     if(!imgforshow.empty())
    //     {
    //         Point2f pg = ptsout[i];
    //         cv::line(imgforshow, pts[i], pg + Point2f(0,-5), cv::Scalar(255,255,0), 1, cv::LINE_4);
    //         rectangle(imgforshow, Rect(pg.x-4,pg.y-4,9,9), cv::Scalar(255,255,0), 1, cv::LINE_4);
    //     }
    // }
    // OFForwardBackwardTracking(img1, img1pyrd, img2, img2pyrd, beta, pts, ptsout, Size(7,7), 0, 0.9, true);
    // for(int i=0;i<pts.size();i++)
    // {
    //     if(ptsout[i].x>0)
    //         pts[i] = Point2f(-1,-1); // 下面的光流跳过这些点
    // }

    // // round 3
    // for(int i=0;i<pts.size();i++)
    // {
    //     if(pts[i].x<0) continue;
    //     if(guessflow[i] == Point2f(-1,-1))
    //         continue;
    //     else
    //         ptsout[i] = pts[i];
    //     if(!imgforshow.empty())
    //     {
    //         Point2f pg = ptsout[i];
    //         cv::line(imgforshow, pts[i], pg + Point2f(0,-5), cv::Scalar(255,0,0), 1, cv::LINE_4);
    //         rectangle(imgforshow, Rect(pg.x-4,pg.y-4,9,9), cv::Scalar(255,0,0), 1, cv::LINE_4);
    //     }
    // }
    // OFForwardBackwardTracking(img1, img1pyrd, img2, img2pyrd, beta, pts, ptsout, Size(9,9), 0, 0.9, true);
    // for(int i=0;i<pts.size();i++)
    // {
    //     if(ptsout[i].x>0)
    //         pts[i] = Point2f(-1,-1); // 下面的光流跳过这些点
    // }






    int Nsuccess = 0;
    for(int i=0; i<pts.size(); i++)
    {
        if(ptsout[i].x > 0)
            Nsuccess++;
    }
    return Nsuccess;

}



