#include "manager.h"
#include "frame.h"
#include "point.h"
#include "camera.h"


using namespace std;
using namespace cv;


// 将img最上沿的中点放到showboard的x,y处，并旋转degree度
void stitchImage(Mat &showboard, Mat &img, double x, double y, double degree)
{
    Mat tmp;
    double w = sqrt(img.cols*img.cols/4.0 + img.rows*img.rows);
    Mat M = getRotationMatrix2D(Point2f(img.cols/2.0, 0), degree, 1);
    M.at<double>(0, 2) += w - img.cols/2.0;
    M.at<double>(1, 2) += w;
    warpAffine(img, tmp, M, Size(w*2,w*2), INTER_NEAREST, BORDER_CONSTANT, 0);

    if(x-w>showboard.cols || y-w>showboard.rows || x+w<0 || y+w<0) return; // 不在画面内

    int sx = x-w < 0 ? 0 : x-w; // showboard的左上角坐标
    int sy = y-w < 0 ? 0 : y-w;
    int ww = x-w < 0 ? w+x : ( x+w > showboard.cols ? w+showboard.cols-x : w*2); // 重叠区域的长宽
    int hh = y-w < 0 ? w+y : ( y+w > showboard.rows ? w+showboard.rows-y : w*2);
    int tx = x-w < 0 ? w-x : 0; // img左上角坐标
    int ty = y-w < 0 ? w-y : 0;
    Mat roi = showboard(Rect(sx, sy, ww, hh));
    // Mat showboardbefore = showboard.clone();
    tmp(Rect(tx, ty, ww, hh)).copyTo(roi, tmp(Rect(tx, ty, ww, hh))); // 把tmp自身当做mask，黑色部分不覆盖
    // showboard = max(showboard, showboardbefore);
    // showboard = showboard*0.04 + showboardbefore*0.96;
}

bool Manager::updateGround()
{
    // 只在关键帧计算地面
    // if(cf->isKF())
    {
        default_random_engine e(time(0));
        vector<MapPoint*> gmps;
        vector<Point3d> gpws;
        vector<Point3d> ows;
        vector<Point2f> ptsundist;

        // 找出最近的地面点
        vector<unsigned> all_indecies;
        // vector<Frame*> recent_frames;
        // getRecentFrames(recent_frames, 50);
        vector<Frame*> recent_frames = getRecentKFs(50); // 取最近50个关键帧轨迹拟合地平面，更多的话可能由于尺度漂移造成错误
        for(int i=0;i<recent_frames.size();i++)
        {
            for(auto &mp: recent_frames[i]->getMapPoints())
            {
                if(!mp->isBad && mp->isOnGround && mp->getpw()!=Point3d(0,0,0) && count(gmps.begin(), gmps.end(), mp)==0)
                {
                    gmps.push_back(mp);
                    gpws.push_back(mp->getpw());
                    ptsundist.push_back(mp->pundist[0]);
                }
            }
            // if(recent_frames[i]->isKF())
                ows.push_back(recent_frames[i]->getOwPt());
        }

        if(gpws.size()<30 || ows.size()<3) return true;

        for(int i=0; i<gpws.size(); ++i)
        {
            all_indecies.push_back(i);
        }

        uniform_int_distribution<unsigned> ground_ransac;
        std::vector<Point3d> best_pts(0);
        std::vector<int> id_indecies;
        float plane[4];
        for(int i=0; i<250; ++i)
        {
            vector<unsigned> available_indecies(all_indecies);
            vector<Point3d> base_points;
            for(int j=0; j<4; ++j)
            {
                ground_ransac.param(uniform_int_distribution<unsigned>::param_type{0,available_indecies.size()-1});
                unsigned id = ground_ransac(e);
                cout << "id: " << id << endl;
                base_points.push_back(gpws[available_indecies[id]]);
                available_indecies[id] = available_indecies[available_indecies.size()-1];
                available_indecies.pop_back();
            }
            
            fitPlane(base_points, plane);
            float A = plane[0];
            float B = plane[1];
            float C = plane[2];
            float D = plane[3];
            vector<Point3d> inliners;
            vector<int> indecies_in;
            for(int j=0; j<gpws.size(); ++j)
            {
                Point3d pt = gpws[j];
                float error = fabs(A*pt.x + B*pt.y + C*pt.z + D);
                cout << "height:" << D << endl;
                cout << "error: " << error << endl << endl;
                if(error<0.2*fabs(D))
                {
                    inliners.push_back(pt);
                    indecies_in.push_back(j);
                }
            }
            if(best_pts.size()<inliners.size())
            {
                best_pts.reserve(inliners.size());
                best_pts.assign(inliners.begin(), inliners.end());
                id_indecies.reserve(indecies_in.size());
                id_indecies.assign(indecies_in.begin(), indecies_in.end());
            }
        }
        fitPlane(best_pts, plane);
        if(pts_on_ground.size())
            pts_on_ground.clear();

        for(int i=0; i<id_indecies.size(); ++i)
        {
            pts_on_ground.push_back(ptsundist[id_indecies[i]]);
        }
        int flag = 1;
        if(plane[1] < 0)
        {
            flag = -1;
        }
        if(ground_param.size()) ground_param.clear();
        for(int i=0; i<4; ++i)
        {
            ground_param.push_back(flag*plane[i]);
        }
    }
    return true;
}