#include "util.h"
#include "vrprocess.h"


VideoCapture cam;
Mat img;
vector<int> orisize;
void* handle = 0;
bool lost = false;
bool useviewer = true;

void init(string parampath, string datapath, int startframe = 0)
{
    Mat Komni, D, Kundist;
    double xi;
    vector<int> recsize, safeborder(4);
    int colgridN=16, rowgridN=9;
    float cam_height;

    // 读取参数
    FileStorage fs;
    if(!fs.open(parampath, FileStorage::READ)) { cout << "params YML file not found." << endl; exit(0);}
    fs["M1"] >> Komni;
    fs["D1"] >> D;
    fs["xi1"] >> xi;

    if(Kundist.empty()) Komni.copyTo(Kundist);
    fs["orisize"] >> orisize;
    fs["recsize"] >> recsize;
    fs["safeborder"] >> safeborder;
    fs["cam_height"] >> cam_height;
    if(safeborder.empty())
    {
        safeborder[0] = 10;
        safeborder[1] = 10;
        safeborder[2] = 10;
        safeborder[3] = 10;
        cout << "   warning: missing safeborder, the default value will be used" << endl;
    }
    fs["colgridN"] >> colgridN;
    fs["rowgridN"] >> rowgridN;
    fs.release();
    int safeborder_[4];
    for(int i=0;i<4;i++)
        safeborder_[i] = safeborder[i];
    float Komni_[9];
    Komni_[0] = Komni.at<double>(0,0);
    Komni_[1] = Komni.at<double>(0,1);
    Komni_[2] = Komni.at<double>(0,2);
    Komni_[3] = Komni.at<double>(1,0);
    Komni_[4] = Komni.at<double>(1,1);
    Komni_[5] = Komni.at<double>(1,2);
    Komni_[6] = Komni.at<double>(2,0);
    Komni_[7] = Komni.at<double>(2,1);
    Komni_[8] = Komni.at<double>(2,2);
    float D_[4];
    D_[0] = D.at<double>(0);
    D_[1] = D.at<double>(1);
    D_[2] = D.at<double>(2);
    D_[3] = D.at<double>(3);
    if(Kundist.empty()) Kundist = Komni;
    float Kundist_[9];
    Kundist_[0] = Kundist.at<double>(0,0);
    Kundist_[1] = Kundist.at<double>(0,1);
    Kundist_[2] = Kundist.at<double>(0,2);
    Kundist_[3] = Kundist.at<double>(1,0);
    Kundist_[4] = Kundist.at<double>(1,1);
    Kundist_[5] = Kundist.at<double>(1,2);
    Kundist_[6] = Kundist.at<double>(2,0);
    Kundist_[7] = Kundist.at<double>(2,1);
    Kundist_[8] = Kundist.at<double>(2,2);

    void* _handle = vrp_inithandle();

    char token[100];
    float* undistortmap = NULL;
    int ret = vrp_init(_handle, Komni_, D_, xi, orisize[0], orisize[1], Kundist_, recsize[0], recsize[1], safeborder_, colgridN, rowgridN, useviewer, cam_height);

    string tail = datapath.substr(datapath.size() - 3);
    if(tail == "mp4" || tail == "MP4" || tail == "avi" || tail == "AVI" || tail == "mov" || tail == "MOV" || tail == ".ts" || tail == ".TS" || tail == "264")
    {
        cam.open(datapath);
        cam.set(CAP_PROP_POS_FRAMES,  startframe*cam.get(CAP_PROP_FPS));
        std::cout << "Camera FPS: " << cam.get(CAP_PROP_FPS) << std::endl;
    }
    else
    {
        cout << "Open imagepath fail!" << endl;
    }

    handle = _handle;
}

int main(int argc, char* argv[])
{
    if(argc == 3)
    {
        init(argv[1], argv[2]);
    }
    else
    {
        cout << "Usage:\n./testvrp path_to_yml path_to_video" << endl;
        exit(0);
    }

    vrp_start(handle);

    thread BAthread([&]() {
        while(1)
        {
            if(vrp_bundle(handle) == -1) lost = true;
        }
    });
    BAthread.detach();

    cv::Mat R_cam_vec;

    while(1)
    {
        cam >> img;
        if(!img.empty())
        {
            Mat imggray = Mat::ones(orisize[1], orisize[0], CV_8UC1);
            if(img.channels() == 3) cvtColor(img, img, COLOR_BGR2GRAY);

            img(Rect(0,0,orisize[0], orisize[1])).copyTo(imggray);
            int ret = vrp_doprocess(handle, imggray.data, 0);
            if(ret == -2)
            {
                cout << "restart\n";
                vrp_restart(handle);
                lost = true;
            }
            else if(ret == 10)
            {
                R_cam_vec = vrp_getRcv(handle);
                std::cout << "\n\n\nR_cam_vec:\n" << R_cam_vec << "\n\n";
                std::cout << "calibrate ground done\n";
                vrp_stop(handle);
                break;
            }
        }
        else
        {
            cout << "Done!" << endl;
        }

        if(handle)
        {
            refreshviewer(handle);
        }
        usleep(100);
    }
    return 0;
}