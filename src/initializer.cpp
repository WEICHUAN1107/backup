#include "manager.h"
#include "frame.h"
#include "point.h"
#include "opticalflow.h"


// 分解E得到RT
bool find_transform(Mat& K, vector<Point2f>& p1, vector<Point2f>& p2, vector<MapPoint*>& mps, Mat& T)
{
    assert(p1.size() == p2.size());
    if(p1.size() < 8)
    {
        cout << "Too few points!" << endl;
        return false;
    }

    Mat mask;
    //根据匹配点求取本征矩阵，使用RANSAC，进一步排除失配点
    Mat E = findEssentialMat(p1, p2, K, RANSAC, 0.5, 1, mask); // 0.5, 1

    if(E.empty())
    {
        cout << "E is empty" << endl;
        return false;
    }

    int feasible_count = countNonZero(mask);
    // cout << "findEssentialMat inliers: " << feasible_count << " -in- " << p1.size() << endl;
    if(feasible_count <= 5 || ((float)feasible_count / p1.size()) < 0.6) //对于RANSAC而言，outlier数量大于50%时，结果是不可靠的
    {
        cout << "solving E with too many outliers: " << (float)feasible_count / p1.size() << endl;
        return false;
    }

    //分解E，获取相对变换
    Mat R, t;
    int pass_count = recoverPose(E, p1, p2, K, R, t, mask);
    T = combineT(R, t);
    
    Mat r;
    Rodrigues(R, r);
    cout << "r.t(): " << r.t() << endl;
    cout << "t.t(): " << t.t() << endl;

    // 先验知识：相机不会朝天看
    if(t.at<double>(2) < 0 && t.at<double>(1) < -0.1)
    {
        cout << "not correct: y < -0.1" << endl;
        return false;
    }

    // // 先验知识：相机主要是水平安装，因此会水平运动，不会竖直运动，所以运动方向与地面夹角不会大于45度
    if(fabs(t.at<double>(1)) > fabs(t.at<double>(2)))
    {
        cout << "not correct: y > z" << endl;
        return false;
    }

    // 先验知识：相机不应该左右平移
    // if(fabs(t.at<double>(0)) > fabs(t.at<double>(2)))
    // {
    //     cout << "not correct: x > z" << endl;
    //     return false;
    // }

    // 删掉outliers
    vector<Point2f> p1_copy = p1, p2_copy = p2;
    vector<MapPoint*> mps_copy = mps;
    p1.clear();
    p2.clear();
    mps.clear();
    for(int i=0; i<mask.rows; i++)
    {
        if(mask.at<uchar>(i) > 0)
        {
            p1.push_back(p1_copy[i]);
            p2.push_back(p2_copy[i]);
            mps.push_back(mps_copy[i]);
        }
    }

    return true;
}

bool Manager::initializeMap()
{
    const int shiftth = (saferight-safeleft)/20; // 640对应约30个像素

    Frame* refframe = NULL;
    for(auto &f: getRecentFrames(100)) // 从当前帧逆序往前
    {
        int cnt = 0;
        for(auto &mp: cf->getMapPoints())
        {
            if(mp->isInFrame(f) && distance1(mp->pdistortInFrame(cf),mp->pdistortInFrame(f))>shiftth)
                cnt++;
            if(cnt > 20) // 超过N个点的运动大于shiftth个像素
            {
                refframe = f;
                goto DONE;
            }
        }
    }

    DONE:

    if(refframe != NULL) // 像素移动距离够大了，初始化
    {
        // cout << "Trying to initialize..." << endl;
        // 取得匹配点对
        vector<Point2f> p1, p2;
        vector<MapPoint*> commonmp;
        for(auto &mp: cf->getMapPoints())
        {
            // 取出两帧共有的特征点
            if(mp->isInFrame(refframe))
            {
                p1.push_back(mp->pundistInFrame(refframe));
                p2.push_back(mp->pundistInFrame(cf));
                commonmp.push_back(mp);
                if(!imgforshow.empty())
                    circle(imgforshow, mp->pdistortInFrame(cf), 10, mp->color, 3, LINE_AA);
            }
        }

        Mat Tcw, Tcw0 = Mat::eye(4, 4, CV_64F);
        bool OK = find_transform(K, p1, p2, commonmp, Tcw);

        if(OK)
        {
            Mat Twc0 = Tcw0.inv();

            Mat Rcr, tcr;
            Mat Tcr = Tcw * Twc0;
            splitT(Tcr, Rcr, tcr);

            vector<Point3d> p3ds;
            vector<float> parallax;
            for(int i=0;i<p1.size();i++)
            {
                Mat f1(Matx31d((p1[i].x-cx)/fx, (p1[i].y-cy)/fy, 1));
                Mat f2(Matx31d((p2[i].x-cx)/fx, (p2[i].y-cy)/fy, 1));
                double depth, parallax_;
                depthFromTriangulation(Rcr, tcr, f1, f2, depth, parallax_);
                Point3d p3d = pointMultiply(Twc0, pointMultiply(Kinv, Point3d(p1[i].x*depth, p1[i].y*depth, depth)));
                p3ds.push_back(p3d);
                parallax.push_back(parallax_);
            }

            int cnt = 0;
            for(int i=0;i<commonmp.size();i++)
            {
                if(parallax[i] > 1) // 视差角太小的丢掉
                {
                    commonmp[i]->setpw(p3ds[i]);
                    cnt++;
                }
            }
            if(cnt < 10)
            {
                // cout << "too few good 3d pts." << endl;
                for(int i=0;i<commonmp.size();i++)
                    commonmp[i]->setpw(Point3d(0,0,0));
                return false;
            }

            for(int i=0;i<cfid;i++)
            {
                getFrame(i)->setTcw(Tcw0);
                getFrame(i)->setKF(false);
            }
            refframe->setKF(true);
            cf->setTcw(Tcw);
            cf->setKF(true);

            // 假设从refframe到cf是匀速的
            Mat velocityKF = Tcw * Twc0;
            float n = cf->fid - refframe->fid;
            Mat R, r, t;
            splitT(velocityKF, R, t);
            Rodrigues(R,r);
            r /= n;
            t /= n;
            Rodrigues(r,R);
            setVelocity(combineT(R,t));

            initialized = true;
            // cout << "Initialized!" << endl;
            // cout << "refframe->fid: " << refframe->fid << endl;
            // cout << "cf->fid: " << cf->fid << endl;
            return true;
        }
        else
        {
            // cout << "Initialize fail, retry..." << endl;
        }
    }
    return false;
}






