#include "optimizer.h"
#include "manager.h"
#include "frame.h"
#include "point.h"


bool Manager::solvePnP(Mat& R, Mat& t)
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    PinholeCamera* cam = new PinholeCamera(fx, fy, cx, cy); // 其他new的对象g2o会负责delete，但PinholeCamera是自定义类，需要手动delete
    vector<MapPoint*> mps = cf->getMapPoints();
    vector<int> ids;
    vector<EdgeProjectXYZ2UVPoseOnly*> edges;

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
             R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
             R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(R_mat, Eigen::Vector3d(t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0))));
    optimizer.addVertex(pose);

    // edges
    for(int i=0;i<mps.size();i++)
    {
        if(mps[i]->isBad) continue;
        Point3d pw = mps[i]->getpw();
        if(pw != Point3d(0,0,0))
        {
            EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
            Point2f p2d = mps[i]->pundistInFrame(cf);
            edge->setMeasurement( Eigen::Vector2d(p2d.x, p2d.y));
            edge->setId(i);
            edge->setVertex(0, pose);
            edge->camera_ = cam;
            edge->point_ = Eigen::Vector3d(pw.x, pw.y, pw.z);
            edge->setInformation( Eigen::Matrix2d::Identity());

            // RobustKernelCauchy 比 RobustKernelHuber 压缩更狠
            g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy; // 不同类型的核函数 参考g2o/g2o/core/robust_kernel_impl.cpp
            edge->setRobustKernel(rk);
            rk->setDelta(2);

            optimizer.addEdge(edge);
            edges.push_back(edge);
            ids.push_back(i);
        }
    }

    if(edges.size() < 5)
    {
        cout << "too few 3d points (" << edges.size() << ") to track!" << endl;
        delete cam;
        return false;
    }

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int Noutliers = 0;
    for(int i=0;i<edges.size();i++)
    {
        if(edges[i]->chi2() > 9)
        {
            mps[ids[i]]->isBad = true;
            Noutliers++;
        }
    }
    // if((float)Noutliers / edges.size() > 0.5)
    // {
    //     for(int i=0;i<edges.size();i++)
    //         cout << edges[i]->chi2() << ", " << flush;
    //     cout << endl;
    //     delete cam;
    //     return false; // 一半以上都是outliers 肯定有问题
    // }

    Mat Tcw;
    eigen2cv(Eigen::Isometry3d(pose->estimate()).matrix(), Tcw);
    cf->setTcw(Tcw);
    log << "  PnP:" << edges.size() << "pts," << Noutliers << "bad";

    // Mat velocity = cf->getTcw() * lastf->getTwc(); // 从上一帧到当前帧的变换
    // setVelocity(velocity);

    delete cam;
    return true;
}



void Manager::BundleAdjustment()
{
    if(vKF.size() < 2) return;

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block;
    std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverDense<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    // optimizer.clear();

    PinholeCamera* cam = new PinholeCamera(fx, fy, cx, cy); // 其他new的对象g2o会负责delete，但PinholeCamera是自定义类，需要手动delete
    vector<int> vPoseid, vPointid, vEdgePointid;
    vector<EdgeProjectXYZ2UVPoseAndPoint*> edges;
    // vPoseid.clear();
    // vPointid.clear();
    // vEdgePointid.clear();
    // edges.clear();
    for(int k=0;k<vKF.size();k++)
    {
        Frame* jf = vKF[k];
        int j = jf->fid;

        // vertex cam pose
        g2o::VertexSE3Expmap* vPose = new g2o::VertexSE3Expmap();
        Mat R = getFrame(j)->getRcw();
        Mat t = getFrame(j)->gettcw();
        Eigen::Matrix3d R_mat;
        R_mat << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                 R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                 R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);
        vPose->setId(j); // 所有vertex都要有唯一的id，哪怕是不同类型的vertex
        if(vKF.size() > 3) // 刚开始系统不稳定，fix一帧效果好。后面稳定了，fix两帧效果好
        {
            if(k==vKF.size()-1 || k==vKF.size()-2)// || k==vKF.size()-3) // fix最早的两个或三个，而非1个，可有效减小尺度漂移
                vPose->setFixed(true);
        }
        else
        {
            if(k==vKF.size()-1)
                vPose->setFixed(true);
        }
        vPose->setEstimate(g2o::SE3Quat(R_mat, Eigen::Vector3d(t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0))));
        optimizer.addVertex(vPose);
        vPoseid.push_back(j);

        for(auto mp: jf->getMapPoints())
        {
            // vertex points
            int id = cfidBA + mp->pid + 1; // 所有vertex都要有唯一的id，哪怕是不同类型的vertex
            if(!mp->isBad && mp->getpw()!=Point3d(0,0,0) && count(vPointid.begin(), vPointid.end(), id)==0)
            {
                g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
                vPoint->setEstimate(Eigen::Vector3d(mp->getpw().x, mp->getpw().y, mp->getpw().z));
                vPoint->setId(id);
                vPoint->setMarginalized(true);
                optimizer.addVertex(vPoint);
                vPointid.push_back(id);
            }

            // edges
            if(!mp->isBad && mp->getpw()!=Point3d(0,0,0))
            {
                EdgeProjectXYZ2UVPoseAndPoint* edge = new EdgeProjectXYZ2UVPoseAndPoint();
                edge->camera_ = cam;
                edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(id))); // point
                edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(j))); // pos
                edge->setMeasurement(Eigen::Vector2d(mp->pundistInFrame(jf).x, mp->pundistInFrame(jf).y));
                edge->setParameterId(0,0);
                edge->setInformation(Eigen::Matrix2d::Identity());

                // 不同类型的核函数 参考g2o/g2o/core/robust_kernel_impl.cpp    RobustKernelHuber
                g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
                edge->setRobustKernel(rk);
                rk->setDelta(1);

                optimizer.addEdge(edge);
                // edges.push_back(edge);
                // vEdgePointid.push_back(mp->pid);
            }
        }
    }

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // recover vPose
    for(auto &posid: vPoseid)
    {
        g2o::VertexSE3Expmap* vPose = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(posid));
        Mat T;
        eigen2cv(Eigen::Isometry3d(vPose->estimate()).matrix(), T);
        // unique_lock<mutex> lock(MutexallFrames);
        allFrames[posid]->setTcw(T);
    }

    // recover vPoints
    for(auto &ptid: vPointid)
    {
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(ptid));
        // unique_lock<mutex> lock(MutexallMapPoints);
        allMapPoints[ptid-cfidBA-1]->setpw( Point3d(vPoint->estimate()[0], vPoint->estimate()[1], vPoint->estimate()[2]) );
    }

    Mat velocityKF = cfBA->getTcw() * lastkfBA->getTwc(); // 从上一帧到当前帧的变换
    float n = cfBA->fid - lastkfBA->fid;
    Mat R, r, t;
    splitT(velocityKF, R, t);
    Rodrigues(R,r);
    r /= n;
    t /= n;
    Rodrigues(r,R);
    setVelocity(combineT(R,t));

    delete cam;
}

