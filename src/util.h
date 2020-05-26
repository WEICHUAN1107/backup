#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <iomanip>
#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <iomanip>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <fstream>

#include <dirent.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

// #include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;


Mat getimggrad(Mat& img, int ksize, double scale, int direction);

Mat getgrad(Mat& patch);

Mat combineT(Mat &R, Mat &t);

void splitT(Mat T, Mat &R, Mat &t);

Point3d pointMultiply(Mat T, Point3d p);

void saveImg(Mat &img, string path);

double distance1(const Point2f& a, const Point2f& b);

double distance2(const Point2f& a, const Point2f& b);

double distance2Matx31d(const Mat& a, const Mat& b);

double distance1(const Point3d& a, const Point3d& b);

double distance2(const Point3d& a, const Point3d& b);

void getHistNum(vector<float>& input, float min, float max, int N, vector<int>& output);

void getHistNumExpand(vector<float>& input, float min, float max, int N, int e, vector<int>& output);

long gettime();

int getKey();

Mat getpatch(Mat& img, Point2f center, int psize);

double sparseSAD(Mat &img1, Mat &img2);

//ax+by+cz+d=0
void fitPlane(const vector<Point3d>& points, float* plane);

void fitPlaneParallelToVector(const vector<Point3d>& points, Point3d vec, float* plane);

void depthFromTriangulation(const Mat& Rcr, const Mat& tcr, const Mat& fr, const Mat& fc, double& depth, double& parallax);

// ax+by+cz+d = 0
void pointProj2Plane(float p[3], float plane[4], float out[3]);
Point3f pointProj2Plane(Point3f p, vector<float> plane);
Point3f pointProj2Plane(Point3f p, float plane[4]);
Vec3f pointProj2Plane(Vec3f p, Vec4f plane);

Point2f pointTrans(Mat H, Point2f p);

Point3d linePlaneIntersection(Point3d l1, Point3d l2, vector<float> p);

#endif