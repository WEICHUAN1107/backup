#include "viewer.h"
#include "point.h"
#include "frame.h"


Viewer::Viewer(int imgw, int imgh)
{
    int windoww = fmin(imgw*2, 1600), windowh = 1000;

    pangolin::CreateWindowAndBind("show",windoww,windowh);
    glEnable(GL_DEPTH_TEST);

    // 3d
    s_cam = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(windoww,windowh, 900,900, windoww*0.8, windowh*0.3, 0.2,5000),
        // pangolin::ModelViewLookAt(-10,-2,-80, 0,0,0, 0,-1,0)); // 摄像机从哪个点看向哪个点
        pangolin::ModelViewLookAt(0,-80,0, 0,0,0, 0,0,1)); // 摄像机从哪个点看向哪个点 俯视
        // pangolin::ModelViewLookAt(0,60,0, 0,0,0, 0,0,-1)); // 底视
        // pangolin::ModelViewLookAt(-150,0,0, 0,0,0, 0,-1,0)); // 侧视
        // pangolin::ModelViewLookAt(0,0,-1, 0,0,0, 0,-1,0)); // 相机第一视角
    static pangolin::Handler3D handler(s_cam);
    d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0).SetHandler(&handler);

    // 2d images
    // View& SetBounds(Attach bottom, Attach top, Attach left, Attach right, bool keep_aspect);
    // Aspect ratio allows us to constrain width and height whilst fitting within specified
    // bounds. A positive aspect ratio makes a view 'shrink to fit' (introducing empty bars),
    // whilst a negative ratio makes the view 'grow to fit' (cropping the view).
    // When fitting within the specified bounds, push to the top-left (as specified by SetLock).
    d_video = pangolin::Display("video").SetBounds(0, 0.5, 0, 0.5, imgw/(float)imgh).SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, 0.0, 1.0)
          .SetLayout(pangolin::LayoutOverlay)
          .AddDisplay(d_video)
          .AddDisplay(d_cam);

    camT.SetIdentity();
    viewcam.SetIdentity();

    // pangolin::DisplayBase().RecordOnRender("ffmpeg:[fps=30,bps=64000000,unique_filename]//screencap.avi"); // record
}

void Viewer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float w = 3;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

    #ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
    #else
        glMultMatrixd(Twc.m);
    #endif

    glLineWidth(2);
    glColor3f(1.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}



void Viewer::drawPoints(vector<Frame*>& frames, vector<MapPoint*>& mappoints)
{
    for(auto &mp: mappoints)
    {
        if(mp->isBad) continue;
        glPointSize(3);
        glBegin(GL_POINTS);
        glColor3f(mp->color[2]/255.0, mp->color[1]/255.0, mp->color[0]/255.0);
        glVertex3f(mp->getpw().x, mp->getpw().y, mp->getpw().z);
        // std::cout << mp->getpw() << "\n";
        glEnd();
    }
}


void Viewer::drawTrack(vector<Frame*>& frames, vector<MapPoint*>& mappoints)
{
    // draw keyframe track
    vector<Mat> twcs;
    vector<int> iskf;
    for(int i=0;i<frames.size();i++)
    {
        if( (frames[i]->isKF() || i==frames.size()-1) &&
            !frames[i]->getTcw().empty()
        )
        {
            Mat Rcw, tcw;
            splitT(frames[i]->getTcw(), Rcw, tcw);
            Mat twc = -Rcw.t() * tcw;
            twcs.push_back(twc);
            iskf.push_back(frames[i]->isKF());
        }
    }
    if(twcs.size() > 1)
    {
        for(int i=twcs.size()-2;i>=0;i--)
        {
            glPointSize(5);
            glBegin(GL_POINTS);
            glColor3f(1,1,0);
            glVertex3f(twcs[i].at<double>(0), twcs[i].at<double>(1), twcs[i].at<double>(2));
            glEnd();

            glLineWidth(1);
            glColor3f(1,0.5,0);
            glBegin(GL_LINES);
            glVertex3f(twcs[i].at<double>(0), twcs[i].at<double>(1), twcs[i].at<double>(2));
            glVertex3f(twcs[i+1].at<double>(0), twcs[i+1].at<double>(1), twcs[i+1].at<double>(2));
            glEnd();
        }
    }
}


void Viewer::update(vector<Frame*>& frames, vector<MapPoint*>& mappoints, Mat video,  std::vector<float> ground_param)
{
    s_cam.Follow(viewcam); // 画面跟随相机

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(!video.empty())
    {
        if(video.channels() == 1)
            cvtColor(video, video, COLOR_GRAY2BGR);
        d_video.Activate();
        glColor4f(1.0f,1.0f,1.0f,1.0f);
        pangolin::GlTexture tex1(video.cols,video.rows,GL_RGB,true,0,GL_RGB,GL_UNSIGNED_BYTE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // 如果图像宽度不能被4整除，就填充无用数据使得其能被4整除，否则画图会有问题
        tex1.Upload(video.data,GL_BGR,GL_UNSIGNED_BYTE);
        tex1.RenderToViewportFlipY();
    }

    d_cam.Activate(s_cam);

    pangolin::glDrawAxis(10);

    if(frames.size() == 0)
    {
        pangolin::FinishFrame();
        return;
    }

    float A, B, C, D;
    if(ground_param[1]!=-1 && ground_param[2]!=-1)
    {
        A = ground_param[0+1];
        B = ground_param[1+1];
        C = ground_param[2+1];
        D = ground_param[3+1];
        for(auto &mp: mappoints)
        {
            if(mp->isBad) continue;
            Point3d pt = mp->getpw();
            float error = fabs(A*pt.x + B*pt.y + C*pt.z + D);
            // cout << "pt: " << pt.x << " " << D << endl << endl;
            if(error < 0.2*fabs(D))
            {
                glPointSize(3);
                glBegin(GL_POINTS);
                glColor3f(0.0, 1.0, 0.0);
                glVertex3f(pt.x, pt.y, pt.z);
                glEnd();
            }
        }
    }

    drawPoints(frames, mappoints);

    // drawTrack(frames, mappoints);

    // 更新相机视角
    if(frames.size()>1)
    {
        static Mat lastTwc = Mat(Mat::eye(4,4,CV_64F));
        static Mat Twc, TwcFilter = Mat(Mat::eye(4,4,CV_64F));
        Mat curTcw = frames[frames.size()-1]->getTcw();
        if(!curTcw.empty())
            Twc = curTcw.inv();
        else
            Twc = lastTwc;

        TwcFilter = Twc*0.08 + TwcFilter*0.92;
        lastTwc = Twc.clone();

        camT.m[0] = Twc.at<double>(0,0);
        camT.m[1] = Twc.at<double>(1,0);
        camT.m[2] = Twc.at<double>(2,0);
        camT.m[3] = Twc.at<double>(3,0);
        camT.m[4] = Twc.at<double>(0,1);
        camT.m[5] = Twc.at<double>(1,1);
        camT.m[6] = Twc.at<double>(2,1);
        camT.m[7] = Twc.at<double>(3,1);
        camT.m[8] = Twc.at<double>(0,2);
        camT.m[9] = Twc.at<double>(1,2);
        camT.m[10] = Twc.at<double>(2,2);
        camT.m[11] = Twc.at<double>(3,2);
        camT.m[12] = Twc.at<double>(0,3);
        camT.m[13] = Twc.at<double>(1,3);
        camT.m[14] = Twc.at<double>(2,3);
        camT.m[15] = Twc.at<double>(3,3);

        viewcam.m[0] = TwcFilter.at<double>(0,0);
        viewcam.m[1] = TwcFilter.at<double>(1,0);
        viewcam.m[2] = TwcFilter.at<double>(2,0);
        viewcam.m[3] = TwcFilter.at<double>(3,0);
        viewcam.m[4] = TwcFilter.at<double>(0,1);
        viewcam.m[5] = TwcFilter.at<double>(1,1);
        viewcam.m[6] = TwcFilter.at<double>(2,1);
        viewcam.m[7] = TwcFilter.at<double>(3,1);
        viewcam.m[8] = TwcFilter.at<double>(0,2);
        viewcam.m[9] = TwcFilter.at<double>(1,2);
        viewcam.m[10] = TwcFilter.at<double>(2,2);
        viewcam.m[11] = TwcFilter.at<double>(3,2);
        viewcam.m[12] = TwcFilter.at<double>(0,3);
        viewcam.m[13] = TwcFilter.at<double>(1,3);
        viewcam.m[14] = TwcFilter.at<double>(2,3);
        viewcam.m[15] = TwcFilter.at<double>(3,3);
    }

    DrawCurrentCamera(camT);

    pangolin::FinishFrame();
}



