#include "manager.h"
#include "frame.h"
#include "point.h"
#include "subdiv.h"
#include "camera.h"


Point2f FAST16(Mat img)
{
    const int K = 8, N = 25; // 周围连续K个像素大于或小于中心像素值则是角点
    static int pixel[25];
    const int threshold1 = 30, threshold2 = 10;
    static uchar threshold_tab1[512], threshold_tab2[512];
    const int searchstep = 2; // 没必要逐个像素找
    static bool once = true;
    if(once)
    {
        once = false;
        static const int offsets16[][2] =
        {
            {0,  3}, { 1,  3}, { 2,  2}, { 3,  1}, { 3, 0}, { 3, -1}, { 2, -2}, { 1, -3},
            {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3,  1}, {-2,  2}, {-1,  3}
        };
        int m = 0;
        for( ; m < 16; m++ )
            pixel[m] = offsets16[m][0] + offsets16[m][1] * (int)img.step;
        for( ; m < 25; m++ )
            pixel[m] = pixel[m - 16];

        for(int i = -255; i <= 255; i++)
        {
            threshold_tab1[i+255] = (uchar)(i < -threshold1 ? 1 : i > threshold1 ? 2 : 0);
            threshold_tab2[i+255] = (uchar)(i < -threshold2 ? 1 : i > threshold2 ? 2 : 0);
        }
    }

    // 同时按两个阈值找，找到小的先记下来，如果最后没找到大的，就返回小的
    Point2f ret2(-1,-1);
    bool found2 = false;
    for(int i=3; i<img.rows-3; i+=searchstep)
    {
        const uchar* ptr = img.ptr<uchar>(i) + 3;
        for(int j=3; j<img.cols-3; j+=searchstep, ptr+=searchstep )
        {
            int v = ptr[0], d1 = 0, d2 = 0;

            const uchar* tab1 = &threshold_tab1[0] - v + 255;
            const uchar* tab2 = &threshold_tab2[0] - v + 255;

            d1 = tab1[ptr[pixel[0]]] | tab1[ptr[pixel[8]]];
            if(!found2)
                d2 = tab2[ptr[pixel[0]]] | tab2[ptr[pixel[8]]];
            if(d1 == 0 && d2 == 0) continue;

            if(d1 != 0)
            {
                d1 &= tab1[ptr[pixel[2]]] | tab1[ptr[pixel[10]]];
                d1 &= tab1[ptr[pixel[4]]] | tab1[ptr[pixel[12]]];
                d1 &= tab1[ptr[pixel[6]]] | tab1[ptr[pixel[14]]];
            }
            if(!found2 && d2 != 0)
            {
                d2 &= tab2[ptr[pixel[2]]] | tab2[ptr[pixel[10]]];
                d2 &= tab2[ptr[pixel[4]]] | tab2[ptr[pixel[12]]];
                d2 &= tab2[ptr[pixel[6]]] | tab2[ptr[pixel[14]]];
            }
            if(d1 == 0 && d2 == 0) continue;

            if(d1 != 0)
            {
                d1 &= tab1[ptr[pixel[1]]] | tab1[ptr[pixel[9]]];
                d1 &= tab1[ptr[pixel[3]]] | tab1[ptr[pixel[11]]];
                d1 &= tab1[ptr[pixel[5]]] | tab1[ptr[pixel[13]]];
                d1 &= tab1[ptr[pixel[7]]] | tab1[ptr[pixel[15]]];
            }
            if(!found2 && d2 != 0)
            {
                d2 &= tab2[ptr[pixel[1]]] | tab2[ptr[pixel[9]]];
                d2 &= tab2[ptr[pixel[3]]] | tab2[ptr[pixel[11]]];
                d2 &= tab2[ptr[pixel[5]]] | tab2[ptr[pixel[13]]];
                d2 &= tab2[ptr[pixel[7]]] | tab2[ptr[pixel[15]]];
            }

            if( d1 & 1 )
            {
                int vt = v - threshold1, count = 0;
                for(int k = 0; k < N; k++)
                {
                    int x = ptr[pixel[k]];
                    if(x < vt)
                    {
                        if( ++count > K )
                        {
                            return Point2f(j,i); // 找到一个就退出
                            break;
                        }
                    }
                    else
                        count = 0;
                }
            }

            if( d1 & 2 )
            {
                int vt = v + threshold1, count = 0;
                for(int k = 0; k < N; k++)
                {
                    int x = ptr[pixel[k]];
                    if(x > vt)
                    {
                        if( ++count > K )
                        {
                            return Point2f(j,i); // 找到一个就退出
                            break;
                        }
                    }
                    else
                        count = 0;
                }
            }

            if(!found2 && d2 & 1)
            {
                int vt = v - threshold2, count = 0;
                for(int k = 0; k < N; k++)
                {
                    int x = ptr[pixel[k]];
                    if(x < vt)
                    {
                        if( ++count > K )
                        {
                            ret2 = Point2f(j,i);
                            found2 = true;
                            break;
                        }
                    }
                    else
                        count = 0;
                }
            }

            if(!found2 && d2 & 2)
            {
                int vt = v + threshold2, count = 0;
                for(int k = 0; k < N; k++)
                {
                    int x = ptr[pixel[k]];
                    if(x > vt)
                    {
                        if( ++count > K )
                        {
                            ret2 = Point2f(j,i);
                            found2 = true;
                            break;
                        }
                    }
                    else
                        count = 0;
                }
            }
        }
    }
    return ret2;
}



void Manager::addOrRemovePoints()
{
    int colN = colgridN;
    int rowN = rowgridN;
    int safew = saferight - safeleft;
    int safeh = safebottom - safetop;
    float gridw = (float)safew/colN;
    float gridh = (float)safeh/rowN;

    // 统计每个格子内点的数目
    int gridPtsCnt[rowN][colN];
    for(auto &r: gridPtsCnt) for(auto &c: r) c = 0;
    for(auto &mp: cf->getMapPoints())
    {
        Point2f p = mp->pdistortInFrame(cf);
        int x = (p.x-safeleft)/gridw;
        int y = (p.y-safetop)/gridh;
        gridPtsCnt[y][x]++;
    }

    // 删掉距离太近的点 用delauney图避免两层遍历操作
    if(cf->mpsize() > rowgridN*colgridN*0.5)
    {
        float distanceth = fmin( gridw * (float)cf->mpsize()/colN/rowN , gridw ); // 点越多，删除间距越大
        Subdiv subdiv(Rect(0,0,imw,imh));
        vector<MapPoint*> mps = cf->getMapPoints(), tobeerased;
        for(auto &mp: mps)
            subdiv.insert(mp->pdistortInFrame(cf));
        vector<int> edgeIdxList;
        subdiv.getEdgeIdxList(edgeIdxList);
        for(int i=0;i<edgeIdxList.size();i++)
        {
            int v1 = subdiv.getEdgeVtx1(edgeIdxList[i]);
            int v2 = subdiv.getEdgeVtx2(edgeIdxList[i]);
            if(v1<0 || v2<0) continue;
            if(count(tobeerased.begin(), tobeerased.end(), mps[v1])!=0) continue;
            if(count(tobeerased.begin(), tobeerased.end(), mps[v2])!=0) continue;

            Point2f p1 = mps[v1]->pdistortInFrame(cf);
            Point2f p2 = mps[v2]->pdistortInFrame(cf);

            if(distance1(p1, p2) < distanceth)
            {
                if(mps[v1]->getpw()!=Point3d(0,0,0) && mps[v2]->getpw()==Point3d(0,0,0)) // 优先删掉没有3d的点
                    tobeerased.push_back(mps[v2]);
                else if(mps[v1]->getpw()==Point3d(0,0,0) && mps[v2]->getpw()!=Point3d(0,0,0))
                    tobeerased.push_back(mps[v1]);
                else if(mps[v1]->age < mps[v2]->age) // 两个都有3d了就保留年龄大的点
                    tobeerased.push_back(mps[v1]);
                else
                    tobeerased.push_back(mps[v2]);
            }
            // if(!imgforshow.empty())
            //     cv::line(imgforshow, p1, p2, cv::Scalar(255,255,255), 1, cv::LINE_8);
        }
        for(auto mp: tobeerased)
            cf->deleteMapPoint(mp);
    }

    // 增加点
    for(float y=safetop;y<safebottom-gridh+1;y+=gridh)
    {
        for(float x=safeleft;x<saferight-gridw+1;x+=gridw)
        {
            if(gridPtsCnt[lround((y-safetop)/gridh)][lround((x-safeleft)/gridw)] == 0)
            {
                // if(!imgforshow.empty()) imgforshow(Rect(x, y, gridw, gridh)) /= 2;
                Mat grid = cf->im(Rect(x, y, gridw, gridh));
                Point2f p = FAST16(grid);
                if(p != Point2f(-1,-1))
                {
                    Point2f bestpt = p + Point2f(x, y);
                    MapPoint* mp = new MapPoint(ptid++, cf, bestpt, camera->myundistortPoint(bestpt));
                    {
                        // unique_lock<mutex> lock(MutexallMapPoints);
                        allMapPoints.push_back(mp);
                    }
                    if(/*boundary1.dot(Point3f(bestpt.x, bestpt.y, 1))>0 && boundary2.dot(Point3f(bestpt.x, bestpt.y, 1))>0 &&*/ bestpt.y<800 && bestpt.y>500)
                    {
                        mp->isOnGround = true;
                        // if(!imgforshow.empty())
                        // {
                        //     circle(imgforshow, bestpt, 5, Scalar(0,0,200), -1);
                        // }
                    }
                    else if(!imgforshow.empty())
                    {
                        // circle(imgforshow, mp->pdistortInFrame(cf), 5, mp->color, 1, LINE_AA);
                        // rectangle(imgforshow, Rect(bestpt.x-6,bestpt.y-6,13,13), Scalar(255,10,0), 2, cv::LINE_4);
                        // stringstream ss; ss << mp->pid;
                        // putText(imgforshow, ss.str(), mp->pdistortInFrame(cf)+Point2f(0,-6), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1, 4);
                    }
                }
            }
        }
    }

    // 点太少了，增加格子密度再选一次
    if(cf->mpsize() < rowgridN*colgridN*0.5)
    {
        // cout << "select twice: " << cf->mpsize() << endl;
        colN = colgridN*3/2;
        rowN = rowgridN*3/2;
        float gridw = (float)safew/colN;
        float gridh = (float)safeh/rowN;

        // 统计每个格子内点的数目
        int gridPtsCnt1[rowN][colN];
        for(auto &r: gridPtsCnt1) for(auto &c: r) c = 0;
        for(auto &mp: cf->getMapPoints())
        {
            Point2f p = mp->pdistortInFrame(cf);
            int x = (p.x-safeleft)/gridw;
            int y = (p.y-safetop)/gridh;
            gridPtsCnt1[y][x]++;
        }

        // 增加点
        for(float y=safetop;y<safebottom-gridh+1;y+=gridh)
        {
            for(float x=safeleft;x<saferight-gridw+1;x+=gridw)
            {
                if(gridPtsCnt1[lround((y-safetop)/gridh)][lround((x-safeleft)/gridw)] == 0)
                {
                    // if(!imgforshow.empty()) imgforshow(Rect(x, y, gridw, gridh)) /= 2;
                    Mat grid = cf->im(Rect(x, y, gridw, gridh));
                    Point2f p = FAST16(grid);
                    if(p != Point2f(-1,-1))
                    {
                        Point2f bestpt = p + Point2f(x, y);
                        MapPoint* mp = new MapPoint(ptid++, cf, bestpt, camera->myundistortPoint(bestpt));
                        {
                            // unique_lock<mutex> lock(MutexallMapPoints);
                            allMapPoints.push_back(mp);
                        }
                        if(!imgforshow.empty())
                        {
                            // circle(imgforshow, mp->pdistortInFrame(cf), 5, mp->color, 1, LINE_AA);
                            rectangle(imgforshow, Rect(bestpt.x-4,bestpt.y-4,9,9), Scalar(100,255,0), 1, cv::LINE_4);
                            // stringstream ss; ss << mp->pid;
                            // putText(imgforshow, ss.str(), mp->pdistortInFrame(cf)+Point2f(0,-6), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1, 4);
                        }
                    }
                }
            }
        }
    }

}

