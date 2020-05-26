#ifndef OPTICALFLOW_H
#define OPTICALFLOW_H

void OFForwardBackwardTracking(Mat& lastimg, vector<Mat>& lastimpyrd, Mat& img, vector<Mat>& impyrd, float beta, vector<Point2f>& pts,
                                vector<Point2f>& ptsout, Size winSize, int pyrd, double inverseError, bool dofinest);

void OFForwardBackwardTracking(Mat& lastimg, vector<Mat>& lastimpyrd, Mat& img, vector<Mat>& impyrd, float beta, Point2f& pt,
                                Point2f& ptout, Size winSize, int pyrd, double inverseError, bool dofinest=true);

int trackPoints(Mat& img1, vector<Mat>& img1pyrd, Mat& img2, vector<Mat>& img2pyrd, float beta, vector<Point2f>& pts,
                vector<Point2f>& guessflow, vector<Point2f>& ptsout, Mat imgforshow);
#endif




