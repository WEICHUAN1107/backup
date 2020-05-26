#include "manager.h"
#include "frame.h"
#include "point.h"
#include "camera.h"


bool Manager::tracking()
{
    Mat lastfTcw = lastf->getTcw();
    if(lastfTcw.empty())
    {
        cout << "lastf->getTcw().empty()" << endl;
        return false;
    }

    if(cf->isStill)
    {
        cf->setTcw(lastfTcw);
        return true;
    }

    Mat R, t;
    Mat Tcw_motion;
    Mat vel = getVelocity();

    // 如果有运动模型，就用它预测当前帧位姿当做PnP优化初值 否则用上一帧的位姿当初值
    if(!vel.empty() && !cf->isStill && lastkf!=NULL && cf->fid-lastkf->fid<10) // 从上一个关键帧匀速模型，最多持续20帧
    {
        Mat lastkfTcw = lastkf->getTcw();
        Tcw_motion = vel * lastkfTcw;
        int n = cf->fid - lastkf->fid;
        if(n > 1)
        {
            for(int i=0;i<n-1;i++)
                Tcw_motion = vel * Tcw_motion;
        }
        splitT(Tcw_motion, R, t);
    }
    else if(!vel.empty() && !cf->isStill) // 从上一帧预测
    {
        Tcw_motion = vel * lastfTcw;
        splitT(Tcw_motion, R, t);
    }
    else
    {
        splitT(lastfTcw, R, t);
    }

    bool OK = solvePnP(R, t);

    if(!OK)
    {
        cout << "PnP fails. Track lost!" << endl;
        // sleep(9999999999);
        return false;
    }

    return true;
}



vector<float> xs, ys;
int goodcnt = 0;
Point2f lastendpt(0,0);
void Manager::calcEndpt()
{
    if(cf->isStill) return;

    vector<Frame*> kfs = getRecentKFs(5);
    if(kfs.size() > 1)
    {
        Point3d pc = pointMultiply(K, pointMultiply(cf->getTcw(), kfs[0]->getOwPt()));
        Point2f pu(pc.x/pc.z, pc.y/pc.z);
        xs.push_back(pu.x);
        ys.push_back(pu.y);
        if(xs.size() > 100)
        {
            xs.erase(xs.begin());
            ys.erase(ys.begin());
        }
        vector<float> sortxs = xs;
        vector<float> sortys = ys;
        sort(sortxs.begin(),sortxs.end());
        sort(sortys.begin(),sortys.end());
        Point2f endpt = Point2f(sortxs[sortxs.size()/2], sortys[sortys.size()/2]);
        Point2f dist_endpt = camera->mydistortPoint(endpt);

        putText(imgforshow, format("dist_endpt: (%.1f, %.1f) cnt: %d", dist_endpt.x, dist_endpt.y, goodcnt), Point(50,50), FONT_HERSHEY_COMPLEX, 1, Scalar(0,0,0), 4, LINE_AA);
        putText(imgforshow, format("dist_endpt: (%.1f, %.1f) cnt: %d", dist_endpt.x, dist_endpt.y, goodcnt), Point(50,50), FONT_HERSHEY_COMPLEX, 1, Scalar(0,0,255), 2, LINE_AA);
        cv::circle(imgforshow, Point2f(dist_endpt.x,dist_endpt.y), 50, cv::Scalar(0,255,255), 1, cv::LINE_AA);
        cv::line(imgforshow, Point2f(dist_endpt.x-100,dist_endpt.y-100), Point2f(dist_endpt.x+100,dist_endpt.y+100), cv::Scalar(0,255,255), 1, cv::LINE_AA);
        cv::line(imgforshow, Point2f(dist_endpt.x+100,dist_endpt.y-100), Point2f(dist_endpt.x-100,dist_endpt.y+100), cv::Scalar(0,255,255), 1, cv::LINE_AA);
        cv::line(imgforshow, Point2f(dist_endpt.x,dist_endpt.y-2000), Point2f(dist_endpt.x,dist_endpt.y+2000), cv::Scalar(0,255,255), 1, cv::LINE_4);
        cv::line(imgforshow, Point2f(dist_endpt.x+2000,dist_endpt.y), Point2f(dist_endpt.x-2000,dist_endpt.y), cv::Scalar(0,255,255), 1, cv::LINE_4);

        if(norm(lastendpt-endpt) < 5)
        {
            goodcnt++;
        }
        else
        {
            lastendpt = endpt;
            goodcnt = 0;
        }

        if(goodcnt > 60)
        {
            vanishing_founded = true;
            vanishing_pt = endpt;
            dist_vanishing_pt = dist_endpt;
            std::cout << "endpt: " << endpt << std::endl;
        }
    }
}

int Manager::getRcv(uchar* _imgdata)
{
    // if(ground_param[0]==-1 && ground_param[1]==-1) return -1;
    if(ground_param[0] < 0) return -1;
    // cout << "ground_param: " << ground_param[1+0] << " " << ground_param[1+1] << " " << ground_param[1+2] << " " << ground_param[1+3] << endl;
    Point3d r1;
    r1.x = ground_param[1];
    r1.y = ground_param[2];
    r1.z = ground_param[3];
    r1 /= norm(r1);

    Point3d r2 = pointMultiply(K.inv(), Point3d(vanishing_pt.x,vanishing_pt.y, 1));
    r2 /= norm(r2);
    cout << "r1: " << ground_param[1] << " " << ground_param[2] << " " << ground_param[3] << " " << ground_param[4] << endl;
    cout << "r1*r2: " << r1.dot(r2) << endl;
    if(fabs(r1.dot(r2))>0.017452406437283) return -1;
    cout << "theta: " << 57.3*acos(r1.dot(r2)) << endl;

    Point3d r0 = r1.cross(r2);
    r0 /= norm(r0);

    Mat Rcv = Mat::eye(3,3,CV_64F);
    Rcv.at<double>(0,0) = r0.x;
    Rcv.at<double>(1,0) = r0.y;
    Rcv.at<double>(2,0) = r0.z;

    Rcv.at<double>(0,1) = r1.x;
    Rcv.at<double>(1,1) = r1.y;
    Rcv.at<double>(2,1) = r1.z;

    Rcv.at<double>(0,2) = r2.x;
    Rcv.at<double>(1,2) = r2.y;
    Rcv.at<double>(2,2) = r2.z;
    cout << "\n\nRcv:\n" << Rcv << "\n\n\n";
    Point3d s3dc0 = pointMultiply(K*Rcv, Point3d(900,1000,2*5000)), e3dc0 = pointMultiply(K*Rcv, Point3d(-900,1000,2*5000));
    Point3d s3dc1 = pointMultiply(K*Rcv, Point3d(900,1000,2*4000)), e3dc1 = pointMultiply(K*Rcv, Point3d(-900,1000,2*4000));
    Point3d s3dc2 = pointMultiply(K*Rcv, Point3d(900,1000,2*3000)), e3dc2 = pointMultiply(K*Rcv, Point3d(-900,1000,2*3000));
    Point3d s3dc3 = pointMultiply(K*Rcv, Point3d(900,1000,2*2000)), e3dc3 = pointMultiply(K*Rcv, Point3d(-900,1000,2*2000));

    // cout << "s3dc0: " << s3dc0 << "\n";
    // cout << "e3dc0: " << e3dc0 << "\n";
    // cout << "s3dc1: " << s3dc1 << "\n";
    // cout << "e3dc1: " << e3dc1 << "\n";
    // cout << "s3dc2: " << s3dc2 << "\n";
    // cout << "e3dc2: " << e3dc2 << "\n";
    // cout << "s3dc3: " << s3dc3 << "\n";
    // cout << "e3dc3: " << e3dc3 << "\n";

    Point2f s2di0(s3dc0.x/s3dc0.z, s3dc0.y/s3dc0.z), e2di0(e3dc0.x/e3dc0.z, e3dc0.y/e3dc0.z);
    Point2f s2di1(s3dc1.x/s3dc1.z, s3dc1.y/s3dc1.z), e2di1(e3dc1.x/e3dc1.z, e3dc1.y/e3dc1.z);
    Point2f s2di2(s3dc2.x/s3dc2.z, s3dc2.y/s3dc2.z), e2di2(e3dc2.x/e3dc2.z, e3dc2.y/e3dc2.z);
    Point2f s2di3(s3dc3.x/s3dc3.z, s3dc3.y/s3dc3.z), e2di3(e3dc3.x/e3dc3.z, e3dc3.y/e3dc3.z);

    s2di0 = camera->mydistortPoint(s2di0);
    e2di0 = camera->mydistortPoint(e2di0);
    s2di1 = camera->mydistortPoint(s2di1);
    e2di1 = camera->mydistortPoint(e2di1);
    s2di2 = camera->mydistortPoint(s2di2);
    e2di2 = camera->mydistortPoint(e2di2);
    s2di3 = camera->mydistortPoint(s2di3);
    e2di3 = camera->mydistortPoint(e2di3);


    // cout << "s2di0: " << s2di0 << "\n";
    // cout << "e2di0: " << e2di0 << "\n";
    // cout << "s2di1: " << s2di1 << "\n";
    // cout << "e2di1: " << e2di1 << "\n";
    // cout << "s2di2: " << s2di2 << "\n";
    // cout << "e2di2: " << e2di2 << "\n";
    // cout << "s2di3: " << s2di3 << "\n";
    // cout << "e2di3: " << e2di3 << "\n";


    Mat img_show(Size(1920,1080), CV_8UC1, _imgdata), top_view = Mat::zeros(1080, 1920, CV_8UC3);
    cvtColor(img_show, img_show, COLOR_GRAY2BGR);
    line(img_show, s2di0, e2di0, Scalar(0,200,200), 2);
    line(img_show, s2di1, e2di1, Scalar(0,200,200), 2);
    line(img_show, s2di2, e2di2, Scalar(0,200,200), 2);
    line(img_show, s2di3, e2di3, Scalar(0,200,200), 2);
    line(img_show, s2di0, s2di3, Scalar(0,200,0), 2);
    line(img_show, e2di3, e2di0, Scalar(0,0,200), 2);


    cv::circle(img_show, Point2f(dist_vanishing_pt.x,dist_vanishing_pt.y), 50, cv::Scalar(0,255,255), 1, cv::LINE_AA);
    cv::line(img_show, Point2f(dist_vanishing_pt.x-100,dist_vanishing_pt.y-100), Point2f(dist_vanishing_pt.x+100,dist_vanishing_pt.y+100), cv::Scalar(0,255,255), 1, cv::LINE_AA);
    cv::line(img_show, Point2f(dist_vanishing_pt.x+100,dist_vanishing_pt.y-100), Point2f(dist_vanishing_pt.x-100,dist_vanishing_pt.y+100), cv::Scalar(0,255,255), 1, cv::LINE_AA);
    cv::line(img_show, Point2f(dist_vanishing_pt.x,dist_vanishing_pt.y-2000), Point2f(dist_vanishing_pt.x,dist_vanishing_pt.y+2000), cv::Scalar(0,255,255), 1, cv::LINE_4);
    cv::line(img_show, Point2f(dist_vanishing_pt.x+2000,dist_vanishing_pt.y), Point2f(dist_vanishing_pt.x-2000,dist_vanishing_pt.y), cv::Scalar(0,255,255), 1, cv::LINE_4);

    Mat Rc1v = Mat(Matx33d(1,0,0, 0,0,-1, 0,1,0));
    Mat R_top_ori = K*Rcv*Rc1v.t()*K.inv();

    for(int i=0; i<pts_on_ground.size(); ++i)
    {
        circle(img_show, pts_on_ground[i], 4, Scalar(0,200,0), -1);
    }

    // for(int i=0; i<top_view.rows; ++i)
    // {
    //     for(int j=0; j<top_view.cols; ++j)
    //     {
    //         Point3d pt_3d = pointMultiply(R_top_ori, Point3d(j,i,1));
    //         Point2d pt_2d = Point2d(pt_3d.x/pt_3d.z, pt_3d.y/pt_3d.z);
    //         if((2<pt_2d.x&&pt_2d.x<top_view.cols-2) && 2<pt_2d.y&&pt_2d.y<top_view.rows-2)
    //         {
    //             Point pt0 = Point(cvFloor(pt_2d.x), cvFloor(pt_2d.y));
    //             double x = 
    //             top_view.at<Vec3b>(i, j)[0] = 
    //         }
    //     }
    // }






    imwrite("img_show.jpg", img_show);
    resize(img_show, img_show, Size(), 0.5, 0.5);
    // while(1)
    {
        imshow("img_show",img_show);
        waitKey();
    }
cout << "line: " << __LINE__ << endl << endl;
sleep(99999);
exit(0);

    return 0;
}







