#ifndef CAMERA_H
#define CAMERA_H

#include "util.h"


class Camera
{
public:

    Camera(Mat _Komni, Mat _D, double _xi, Mat _K, Size _orisize, Size _newsize)
    {
        Komni = _Komni.clone();
        D = _D.clone();
        xi = _xi;
        KNew = _K.clone();
        f1 = _Komni.at<double>(0,0);
        f2 = _Komni.at<double>(1,1);
        cx = _Komni.at<double>(0,2);
        cy = _Komni.at<double>(1,2);
        s = _Komni.at<double>(0,1);
        k1 = _D.at<double>(0);
        k2 = _D.at<double>(1);
        p1 = _D.at<double>(2);
        p2 = _D.at<double>(3);
        newsize = _newsize;
        orisize = _orisize;

        if(_D.at<double>(0)==0 && _xi==0)
        {
            cout << "No distortion!" << endl;
        }

        map1.release();
        map2.release();
        Mat R = Mat::eye(3, 3, CV_64F);
        _initUndistortRectifyMapOmni(_Komni, _D, _xi, R, _K, _newsize, CV_16SC2, map1, map2);

        mapx.release();
        mapy.release();
        mapx.create(_newsize.height,_newsize.width,CV_32F);
        mapy.create(_newsize.height,_newsize.width,CV_32F);
        for (int y=0; y<mapx.rows; y++)
        {
            float* pmapx = mapx.ptr<float>(y);
            float* pmapy = mapy.ptr<float>(y);
            for (int x=0; x<mapx.cols; x++)
            {
                Point2f pu = _undistortPoint(Point2f(x,y));
                pmapx[x] = pu.x;
                pmapy[x] = pu.y;
            }
        }
    }


    Point2f mydistortPoint(Point2f undistorted)
    {
        if(mapx.empty()) return undistorted;

        // Vec2s pd = map1.at<Vec2s>(cvRound(undistorted.y),cvRound(undistorted.x));
        // return Point2f(pd[0], pd[1]);

        // convert to camera coordinate
        Vec3d Xc(undistorted.x-KNew.at<double>(0,2), undistorted.y-KNew.at<double>(1,2), KNew.at<double>(0,0));

        // convert to unit sphere
        Vec3d Xs = Xc/norm(Xc);

        // convert to normalized image plane
        Vec2d xu = Vec2d(Xs[0]/(Xs[2]+xi), Xs[1]/(Xs[2]+xi));

        // add distortion
        Vec2d xd;
        double r2 = xu[0]*xu[0]+xu[1]*xu[1];
        double r4 = r2*r2;

        xd[0] = xu[0]*(1+k1*r2+k2*r4) + 2*p1*xu[0]*xu[1] + p2*(r2+2*xu[0]*xu[0]);
        xd[1] = xu[1]*(1+k1*r2+k2*r4) + p1*(r2+2*xu[1]*xu[1]) + 2*p2*xu[0]*xu[1];

        // convert to pixel coordinate
        Vec2d final;
        final[0] = f1*xd[0]+s*xd[1]+cx;
        final[1] = f2*xd[1]+cy;

        return  Point2f(final[0], final[1]);
    }


    Point2f myundistortPoint(Point2f distorted)
    {
        if(mapx.empty()) return distorted;
        #if 0
            Point2i idistorted;
            idistorted.x = cvFloor(distorted.x); // 左上角像素
            idistorted.y = cvFloor(distorted.y);
            float a = distorted.x - idistorted.x;
            float b = distorted.y - idistorted.y;
            float ip00 = (1.f - a)*(1.f - b);
            float ip01 = a*(1.f - b);
            float ip10 = (1.f - a)*b;
            float ip11 = a*b;
            Point2f ret = Point2f(mapx.at<float>(idistorted), mapy.at<float>(idistorted)) * ip00 +
                        Point2f(mapx.at<float>(idistorted+Point2i(1,0)), mapy.at<float>(idistorted+Point2i(1,0))) * ip01 +
                        Point2f(mapx.at<float>(idistorted+Point2i(0,1)), mapy.at<float>(idistorted+Point2i(0,1))) * ip10 +
                        Point2f(mapx.at<float>(idistorted+Point2i(1,1)), mapy.at<float>(idistorted+Point2i(1,1))) * ip11;
            return ret;
        #else
            return Point2f(mapx.at<float>(distorted), mapy.at<float>(distorted));
        #endif
    }

    int getUndistortMap(float* undistortmap)
    {
        if(undistortmap == NULL) return -1;
        if(!mapx.empty() && newsize!=Size(0,0))
        {
            memcpy(&undistortmap[0], mapx.data, newsize.height*newsize.width*sizeof(float));
            memcpy(&undistortmap[newsize.height*newsize.width], mapy.data, newsize.height*newsize.width*sizeof(float));
            return 0;
        }
        else
        {
            return -1;
        }
    }


    Mat undistortImg(Mat& img)
    {
        if(map1.empty()) return img;
        Mat imundistorted;
        remap(img, imundistorted, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
        return imundistorted;
    }

    Mat Komni, D, KNew;
    double xi, f1, f2, cx, cy, s, k1, k2, p1, p2;
    Size orisize, newsize;
    Mat map1, map2; // 校正图到原图
    Mat mapx, mapy; // 原图到校正图

private:

    void _initUndistortRectifyMapOmni(InputArray K, InputArray D, InputArray xi, InputArray R, InputArray P,
        const cv::Size& size, int m1type, OutputArray map1, OutputArray map2)
    {
        CV_Assert( m1type == CV_16SC2 || m1type == CV_32F || m1type <=0 );
        map1.create( size, m1type <= 0 ? CV_16SC2 : m1type );
        map2.create( size, map1.type() == CV_16SC2 ? CV_16UC1 : CV_32F );

        CV_Assert((K.depth() == CV_32F || K.depth() == CV_64F) && (D.depth() == CV_32F || D.depth() == CV_64F));
        CV_Assert(K.size() == Size(3, 3) && (D.empty() || D.total() == 4));
        CV_Assert(P.empty()|| (P.depth() == CV_32F || P.depth() == CV_64F));
        CV_Assert(P.empty() || P.size() == Size(3, 3) || P.size() == Size(4, 3));
        CV_Assert(R.empty() || (R.depth() == CV_32F || R.depth() == CV_64F));
        CV_Assert(R.empty() || R.size() == Size(3, 3) || R.total() * R.channels() == 3);
        CV_Assert(xi.total() == 1 && (xi.depth() == CV_32F || xi.depth() == CV_64F));

        cv::Vec2d f, c;
        double s;
        if (K.depth() == CV_32F)
        {
            Matx33f camMat = K.getMat();
            f = Vec2f(camMat(0, 0), camMat(1, 1));
            c = Vec2f(camMat(0, 2), camMat(1, 2));
            s = (double)camMat(0,1);
        }
        else
        {
            Matx33d camMat = K.getMat();
            f = Vec2d(camMat(0, 0), camMat(1, 1));
            c = Vec2d(camMat(0, 2), camMat(1, 2));
            s = camMat(0,1);
        }

        Vec4d kp = Vec4d::all(0);
        if (!D.empty())
            kp = D.depth() == CV_32F ? (Vec4d)*D.getMat().ptr<Vec4f>(): *D.getMat().ptr<Vec4d>();
        double _xi = xi.depth() == CV_32F ? (double)*xi.getMat().ptr<float>() : *xi.getMat().ptr<double>();
        Vec2d k = Vec2d(kp[0], kp[1]);
        Vec2d p = Vec2d(kp[2], kp[3]);
        cv::Matx33d RR  = cv::Matx33d::eye();
        if (!R.empty() && R.total() * R.channels() == 3)
        {
            cv::Vec3d rvec;
            R.getMat().convertTo(rvec, CV_64F);
            cv::Rodrigues(rvec, RR);
        }
        else if (!R.empty() && R.size() == Size(3, 3))
            R.getMat().convertTo(RR, CV_64F);

        cv::Matx33d PP = cv::Matx33d::eye();
        if (!P.empty())
            P.getMat().colRange(0, 3).convertTo(PP, CV_64F);
        else
            PP = K.getMat();

        cv::Matx33d iKR = (PP*RR).inv(cv::DECOMP_SVD);
        cv::Matx33d iK = PP.inv(cv::DECOMP_SVD);
        cv::Matx33d iR = RR.inv(cv::DECOMP_SVD);

        for (int i = 0; i < size.height; ++i)
        {
            float* m1f = map1.getMat().ptr<float>(i);
            float* m2f = map2.getMat().ptr<float>(i);
            short*  m1 = (short*)m1f;
            ushort* m2 = (ushort*)m2f;

            double _x = i*iKR(0, 1) + iKR(0, 2),
                   _y = i*iKR(1, 1) + iKR(1, 2),
                   _w = i*iKR(2, 1) + iKR(2, 2);
            for(int j = 0; j < size.width; ++j, _x+=iKR(0,0), _y+=iKR(1,0), _w+=iKR(2,0))
            {
                // project back to unit sphere
                double r = sqrt(_x*_x + _y*_y + _w*_w);
                double Xs = _x / r;
                double Ys = _y / r;
                double Zs = _w / r;
                // project to image plane
                double xu = Xs / (Zs + _xi),
                    yu = Ys / (Zs + _xi);
                // add distortion
                double r2 = xu*xu + yu*yu;
                double r4 = r2*r2;
                double xd = (1+k[0]*r2+k[1]*r4)*xu + 2*p[0]*xu*yu + p[1]*(r2+2*xu*xu);
                double yd = (1+k[0]*r2+k[1]*r4)*yu + p[0]*(r2+2*yu*yu) + 2*p[1]*xu*yu;
                // to image pixel
                double u = f[0]*xd + s*yd + c[0];
                double v = f[1]*yd + c[1];

                if( m1type == CV_16SC2 )
                {
                    int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
                    int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
                    m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
                    m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
                    m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
                }
                else if( m1type == CV_32FC1 )
                {
                    m1f[j] = (float)u;
                    m2f[j] = (float)v;
                }
            }
        }
    }


    Point2f _undistortPoint(Point2f distorted)
    {
        Vec2d pp((distorted.x*f2-cx*f2-s*(distorted.y-cy))/(f1*f2), (distorted.y-cy)/f2); // plane
        Vec2d pu = pp; // points without distortion

        // remove distortion iteratively
        for(int j=0; j<20; j++)
        {
            double r2 = pu[0]*pu[0] + pu[1]*pu[1];
            double r4 = r2*r2;
            pu[0] = (pp[0] - 2*p1*pu[0]*pu[1] - p2*(r2+2*pu[0]*pu[0])) / (1 + k1*r2 + k2*r4);
            pu[1] = (pp[1] - 2*p2*pu[0]*pu[1] - p1*(r2+2*pu[1]*pu[1])) / (1 + k1*r2 + k2*r4);
        }

        // project to unit sphere
        double r2 = pu[0]*pu[0] + pu[1]*pu[1];
        double a = (r2 + 1);
        double b = 2*xi*r2;
        double cc = r2*xi*xi-1;
        double Zs = (-b + sqrt(b*b - 4*a*cc))/(2*a);
        Vec3d Xw = Vec3d(pu[0]*(Zs + xi), pu[1]*(Zs + xi), Zs);

        // project back to sphere
        Vec3d Xs = Xw / cv::norm(Xw);

        // reproject to camera plane
        Mat ppu(Matx31d(Xs[0]/Xs[2], Xs[1]/Xs[2], 1.0));
        Mat pm = KNew * ppu;

        return Point2f(pm.at<double>(0), pm.at<double>(1));
    }

};


#endif
