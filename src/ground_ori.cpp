#include "manager.h"
#include "frame.h"
#include "point.h"
#include "camera.h"


using namespace std;
using namespace cv;

bool Manager::updateGround()
{
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