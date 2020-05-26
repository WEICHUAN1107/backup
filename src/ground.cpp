#include "manager.h"
#include "frame.h"
#include "point.h"
#include "camera.h"


using namespace std;
using namespace cv;

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
        vector<Frame*> recent_frames;
        getRecentFrames(recent_frames, 50);

        // vector<Frame*> recent_frames = getRecentKFs(50); // 取最近50个关键帧轨迹拟合地平面，更多的话可能由于尺度漂移造成错误
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
            if(recent_frames[i]->isKF())
                ows.push_back(recent_frames[i]->getOwPt());
        }
        if(gpws.size()<10 || ows.size()<3) return true;

        Point3d centroid(0,0,0);
        for(auto &p: ows)
            centroid += p;
        centroid *= 1./ows.size();
        Mat ptsm(ows.size(), 3, CV_64F);
        for(int i=0;i<ows.size();i++)
        {
            ptsm.at<double>(i,0) = ows[i].x - centroid.x;
            ptsm.at<double>(i,1) = ows[i].y - centroid.y;
            ptsm.at<double>(i,2) = ows[i].z - centroid.z;
        }
        Mat W, U, V;
        SVD::compute(ptsm, W, U, V);
        W /= norm(W);

        float bestplane[4];
        bestplane[0] = 0;
        bestplane[1] = 0;
        bestplane[2] = 0;
        bestplane[3] = 0;
        if(W.at<double>(1) > 0.1)
        {
            bestplane[0] = V.at<double>(2,0);
            bestplane[1] = V.at<double>(2,1);
            bestplane[2] = V.at<double>(2,2);
            bestplane[3] = 0;

            vector<float> ds;
            for(auto &mp: gmps)
            {
                Point3d pc = mp->getpw();
                float d = bestplane[0]*pc.x+bestplane[1]*pc.y+bestplane[2]*pc.z+bestplane[3];
                ds.push_back(d);
            }
            sort(ds.begin(), ds.end());
            float meand = ds[ds.size()/2];
            bestplane[3] = -meand;
        }
        else
        {
            for(int i=0; i<gpws.size(); ++i)
            {
                all_indecies.push_back(i);
            }

            Point3d vec(0,0,0);
            if(ows.size()>8)
            {
                Point3d p0 = ows[0];
                Point3d p1 = ows[4];
                Point3d p2 = ows[8];
                vec = p0 - p1;
                if(norm(vec) < norm(p0-p2))
                    vec = p0 - p2;
            }
            else if(ows.size() > 2)
            {
                int id = 2.0*ows.size()/3>ows.size()?ows.size()-1:(int)2*ows.size()/3;
                vec = ows[0] - ows[id];
            }

            uniform_int_distribution<unsigned> ground_ransac;
            std::vector<Point3d> best_pts(0);
            std::vector<int> id_indecies;
            float plane[4];
            for(int i=0; i<0.3*all_indecies.size(); ++i)
            {
                vector<unsigned> available_indecies(all_indecies);
                vector<Point3d> base_points;
                for(int j=0; j<4; ++j)
                {
                    ground_ransac.param(uniform_int_distribution<unsigned>::param_type{0,available_indecies.size()-1});
                    unsigned id = ground_ransac(e);
                    base_points.push_back(gpws[available_indecies[id]]);
                    available_indecies[id] = available_indecies[available_indecies.size()-1];
                    available_indecies.pop_back();
                }

                if(vec != Point3d(0,0,0)) // 刚起步时距离短，方向容易有偏差
                    fitPlaneParallelToVector(base_points, vec, plane);
                else
                    fitPlane(base_points, plane);
                
                float A = plane[0];
                float B = plane[1];
                float C = plane[2];
                float D = plane[3];
                vector<Point3d> inliners;
                for(int j=0; j<gpws.size(); ++j)
                {
                    Point3d pt = gpws[j];
                    float error = fabs(A*pt.x + B*pt.y + C*pt.z + D);
                    if(error<0.2*fabs(D))
                    {
                        inliners.push_back(pt);
                    }
                }
                if(best_pts.size()<inliners.size())
                {
                    best_pts.reserve(inliners.size());
                    best_pts.assign(inliners.begin(), inliners.end());
                }
            }

            if(vec != Point3d(0,0,0)) // 刚起步时距离短，方向容易有偏差
                fitPlaneParallelToVector(best_pts, vec, bestplane);
            else
                fitPlane(best_pts, bestplane);
        }

        if(bestplane[1] < 0)
        {
            bestplane[0] *= -1;
            bestplane[1] *= -1;
            bestplane[2] *= -1;
            bestplane[3] *= -1;
        }
        cout << "bestplane: " << bestplane[0] << " " << bestplane[1] << " " << bestplane[2] << " " << bestplane[3] << endl;
        cout << "prevplane: " << ground_param[1+0] << " " << ground_param[1+1] << " " << ground_param[1+2] << " " << ground_param[1+3] << endl;

        history_plane.push_back(Vec4f(bestplane[0],bestplane[1],bestplane[2],bestplane[3]));

        Vec3f bestnorm(bestplane[0],bestplane[1],bestplane[2]);
        Vec3f groundnorm(ground_param[1],ground_param[2],ground_param[3]);
        cout << "bestnorm.dot(groundnorm): " << bestnorm.dot(groundnorm) << endl << endl;
        if(/*ground_param[2]==-1 && ground_param[1]==-1*/1)
        {
            ground_param[1] = bestplane[0];
            ground_param[2] = bestplane[1];
            ground_param[3] = bestplane[2];
            ground_param[4] = bestplane[3];
            counts++;
            if(counts >= 200)
                ground_param[0] = 1;
        }
        else if(1-bestnorm.dot(groundnorm) < 1e-4) // 两平面平行
        {
            ground_param[0] = 1;
            cout << "parallel\n";
        }
        else
        {
            
            // 记录修改前的值
            Vec3f g0(ground_param[1],ground_param[2],ground_param[3]);
            Vec3f Ow = cf->gettwc();
            Vec3f p0 = pointProj2Plane(Ow, Vec4f(ground_param[1],ground_param[2],ground_param[3],ground_param[4])); // 相机中心在地平面的投影
            Vec3f p1 = pointProj2Plane(Ow, Vec4f(bestplane[0],bestplane[1],bestplane[2],bestplane[3])); // 相机中心在地平面的投影
            Vec3f crosspt = p0*0.95 + p1*0.05;
            for(int i=0;i<3;i++)
                ground_param[i+1] = bestplane[i]*0.05 + ground_param[i+1]*0.95;

            // 代入上面的点求D
            ground_param[4] = -ground_param[1]*crosspt[0] - ground_param[2]*crosspt[1] - ground_param[3]*crosspt[2];
            // 等待地面法向量稳定时，保存
            if(ground_param[0]<0)
            {
                Vec3f g1(ground_param[1],ground_param[2],ground_param[3]);
                g0 /= norm(g0);
                g1 /= norm(g1);
                float change = acos(g0.dot(g1))*57.3;
                if(change<0.05)
                {
                    std::cout << "calib ground norm: " << g1 << std::endl;
                    ground_param[0] = 1;
                }
            }
        }
        double gnorm = sqrt (ground_param[1]*ground_param[1] + ground_param[2]*ground_param[2] + ground_param[3]*ground_param[3]);
        // ground_param[0] = -1;
        ground_param[1] /= gnorm;
        ground_param[2] /= gnorm;
        ground_param[3] /= gnorm;
        ground_param[4] /= gnorm;

        for(auto &mp: cf->getMapPoints())
        {
            Point3d pc = mp->getpw();
            if(!mp->isBad && pc!=Point3d(0,0,0))
            {
                float d = ground_param[1]*pc.x+ground_param[2]*pc.y+ground_param[3]*pc.z+ground_param[4];
                if(fabs(d) < fabs(ground_param[4])*0.2)
                    mp->isOnGround = true;
                else
                    mp->isOnGround = false;
            }
        }
    }
    return true;
}