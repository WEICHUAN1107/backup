#include "manager.h"
#include "vrprocess.h"
#include "camera.h"

#define PANGOLIN

#ifdef PANGOLIN
#include "viewer.h"
#endif


typedef struct
{
    vector<int> safeborder;
    int colgridN, rowgridN;
    bool useviewer;
    float cam_height;
    Size window_size;

    #ifdef PANGOLIN
    Viewer* viewer = NULL;
    #endif
    Mat canvas;

    Camera* camera = NULL;
    Manager* manager = NULL;
    volatile bool doBA = false;
    volatile bool isstart = false;

    mutex Mutexvrpinit, Mutexvrpstop, Mutexvrpstart;
}VRP;

void* vrp_inithandle()
{
    VRP* handle = new VRP();
    return (void*)handle;
}

int vrp_releasehandle(void* handle)
{
    VRP* vrp = (VRP*)handle;
    vrp_stop(handle);
    delete vrp;
    return 0;
}

int vrp_init(void* handle, float _Komni[9], float _D[4], float _xi, int _oriw, int _orih, float _KNew[9], int _neww, int _newh,
            int _safeborder[4], int _colgridN, int _rowgridN, bool _useviewer, float height)
{
    VRP* vrp = (VRP*)handle;
    unique_lock<mutex> lock(vrp->Mutexvrpinit);

    Mat Komni(3,3,CV_32F,_Komni), D(1,4,CV_32F,_D);
    Komni.convertTo(Komni, CV_64F);
    Komni *= (float)_neww / _oriw;
    Komni.at<double>(2,2) = 1;
    D.convertTo(D, CV_64F);

    float invfov = (_Komni[0] + _Komni[4]) / 2 / sqrt(_oriw*_oriw + _orih*_orih);
    float focal = (500*invfov-200)*sqrt(_neww*_neww+_newh*_newh)/640; // 640px & invfov: 0.6~100, 1.0~300 // 视角越大，f越小；图像越大，f越大
    Mat K1(Matx33d(focal, 0, _neww/2.0, 0, focal, _newh/2.0, 0, 0, 1));
    std::cout << "K1: " << K1 << std::endl;

    Mat K(3, 3, CV_32F, _KNew);
    std::cout << "K: " << K << std::endl;
    K.convertTo(K, CV_64F);
    Size orisize = Size(_oriw, _orih);
    Size newsize = Size(_neww, _newh);

    vrp->safeborder.clear();
    vrp->safeborder.push_back(_safeborder[0]);
    vrp->safeborder.push_back(_safeborder[1]);
    vrp->safeborder.push_back(_safeborder[2]);
    vrp->safeborder.push_back(_safeborder[3]);
    vrp->colgridN = _colgridN;
    vrp->rowgridN = _rowgridN;
    vrp->useviewer = _useviewer;
    vrp->cam_height = height;
    vrp->window_size = newsize;

    vrp->camera = new Camera(Komni, D, _xi, K, orisize, newsize);

    #ifdef PANGOLIN
    if(vrp->useviewer && vrp->viewer==NULL)
        vrp->viewer = new Viewer(newsize.width, newsize.height);
    #endif
    return 0;
}

int vrp_start(void* handle)
{
    VRP* vrp = (VRP*)handle;
    unique_lock<mutex> lock(vrp->Mutexvrpstart);
    if(vrp->isstart || vrp->manager != NULL)
    {
        cout << "please stop before trying to start!" << endl;
        return -1;
    }
    vrp->manager = new Manager(vrp->camera, vrp->safeborder, vrp->colgridN, vrp->rowgridN, vrp->cam_height);
    vrp->doBA = false;
    vrp->isstart = true;
    cout << "vrp_started" << endl;
    return 0;
}

int vrp_stop(void* handle)
{
    VRP* vrp = (VRP*)handle;
    unique_lock<mutex> lock(vrp->Mutexvrpstop);
    if(!vrp->isstart)
    {
        cout << "already stopped" << endl;
        return 0;
    }
    vrp->isstart = false;
    if(vrp->useviewer) usleep(100000); // 给其他线程一点反应时间
    while(vrp->doBA)
    {
        cout << "waiting for BA in vrp_stop" << endl;
        usleep(1000);
    }
    delete vrp->manager;
    vrp->manager = NULL;
    return 0;
}

int vrp_doprocess(void* handle, uchar* _imgdata, long long _timestamp)
{
    VRP* vrp = (VRP*)handle;
    if(!vrp->isstart) return 1;
    if(vrp->manager == NULL && vrp->isstart)
    {
        cout << "vrp->manager is NULL in doprocess" << endl;
        return -1;
    }

    vrp->manager->createNewFrame(_imgdata, _timestamp, vrp->useviewer);

    if(vrp->manager->pointsFlow() == false)
    {
        cout << "pointsFlow fail!\n" << vrp->manager->log.str() << endl;
        while(vrp->doBA) usleep(1000);
        return -2;
    }

    if(!vrp->manager->initialized) // 初始化
    {
        vrp->manager->addOrRemovePoints();
        vrp->manager->initializeMap();
    }
    else
    {
        bool OK = vrp->manager->tracking();
        if(!OK)
        {
            cout << "tracking lost!\n" << vrp->manager->log.str() << endl;
            while(vrp->doBA) usleep(1000);
            return -2;
        }

        if(vrp->manager->vanishing_founded == false) vrp->manager->calcEndpt();
        else vrp->manager->getRcv(_imgdata);

        // vrp->manager->recoverRecentLostPts();
        // vrp->manager->updateGround();
        if(vrp->manager->makeKF()) // KF
        {
            bool donelastBA = false;
            thread waitingthread([&]() {
                while(vrp->doBA){ usleep(100); }
                vrp->manager->prepareBA();
                vrp->doBA = true;
                donelastBA = true;
            });
            waitingthread.detach();

            vrp->manager->addOrRemovePoints();

            while(!donelastBA)
                usleep(100);
            vrp->manager->updateGround();
        }
        // vrp->manager->updateGround();
    }
    // printf("%s\n", vrp->manager->log.str().c_str());
    return 0;
}

int vrp_bundle(void* handle)
{
    VRP* vrp = (VRP*)handle;
    if(!vrp->isstart) return 1;
    if(!vrp->doBA) return 2;
    if(vrp->manager == NULL && vrp->isstart)
    {
        cout << "vrp->manager is NULL in bundle" << endl;
        return -1;
    }

    if(vrp->doBA)
    {
        vrp->manager->BundleAdjustment();
        vrp->manager->mapping();
        vrp->manager->KFCulling();
        vrp->doBA = false;
    }

    return 0;
}

int refreshviewer(void* handle)
{
    VRP* vrp = (VRP*)handle;
    if(!vrp->isstart) { usleep(1000); return 1; }

    #ifdef PANGOLIN
        if(!vrp->viewer) return -1;
        vector<Frame*> _allFrames;
        vrp->manager->getAllFrames(_allFrames);
        vector<MapPoint*> _allMapPoints;
        vrp->manager->getAllMapPoints(_allMapPoints);
        vrp->viewer->update(_allFrames, _allMapPoints, vrp->manager->getimgforshow(), vrp->manager->getground());
        if(!vrp->canvas.empty()) { imshow("vrp->canvas",vrp->canvas); waitKey(1); }
    #else
        if(!vrp->manager->getimgforshow().empty()) imshow("show",vrp->manager->getimgforshow()); waitKey(10);
    #endif
    return 0;
}

void vrp_restart(void* handle)
{
    VRP* vrp = (VRP*)handle;
    if(vrp->useviewer) usleep(100000); // 给其他线程一点反应时间
    while(vrp->doBA)
    {
        cout << "waiting for BA in vrp_stop" << endl;
        usleep(1000);
    }
    if(vrp->manager)
        delete vrp->manager;
    vrp->manager = NULL;
    vrp->manager = new Manager(vrp->camera, vrp->safeborder, vrp->colgridN, vrp->rowgridN, vrp->cam_height);
    vrp->doBA = false;
    vrp->isstart = true;
}