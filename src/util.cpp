#include "util.h"

#define debug_line (std::cout << "line: " << __LINE__ << std::endl)


Mat getimggrad(Mat& img, int ksize, double scale, int direction) // 算一个方向的就够了
{
    Mat gradx, grady;
    Mat abs_gradx, abs_grady;
    if(direction == 1)
    {
        // 两种直方图均衡化
        // equalizeHist(img, img);
        // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        // clahe->apply(img, img);
        // medianBlur(img, img, 5 );
        // GaussianBlur(img, img, Size(5,5), 0);
        // boxFilter(img, img, CV_8UC1, Size(5,5));
        // GaussianBlur(img, img, Size(3,3), 0);
        Sobel( img, gradx, CV_16S, 1, 0, ksize, scale, 0, BORDER_DEFAULT );
        // threshold(gradx, gradx, 0, 65535, THRESH_TOZERO);
        convertScaleAbs( gradx, abs_gradx );
        // medianBlur(abs_gradx, abs_gradx, 5 );
        // Mat ret;
        // bilateralFilter(abs_gradx, ret, 20, 20, 20);
        // Mat kernel = (Mat_<float>(3, 3) << 0, -1, 0, 0, 5, 0, 0, -1, 0);
        // filter2D(abs_gradx, ret, CV_8UC1, kernel);
        return abs_gradx;
    }
    else if(direction == 2)
    {
        // GaussianBlur(img, img, Size(3,3), 0);
        // boxFilter(img, img, CV_8UC1, Size(5,5));
        Sobel( img, grady, CV_16S, 0, 1, ksize, scale, 0, BORDER_DEFAULT );
        convertScaleAbs( grady, abs_grady );
        return abs_grady;
    }
    else
    {
        // GaussianBlur(img, img, Size(3,3), 0);
        Sobel( img, gradx, CV_16S, 1, 0, ksize, scale, 0, BORDER_DEFAULT );
        Sobel( img, grady, CV_16S, 0, 1, ksize, scale, 0, BORDER_DEFAULT );
        convertScaleAbs( gradx, abs_gradx );
        convertScaleAbs( grady, abs_grady );
        Mat grad = abs_gradx*0.5 + abs_grady*0.5;
        // threshold(grad, grad, 50, 255, THRESH_TOZERO);
        // GaussianBlur(grad, grad, Size(5,5), 0);
        // dilate(grad, grad, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        return grad;
    }
}

Mat getgrad(Mat& patch)
{
    Mat grad, gradx, grady;
    Mat abs_gradx, abs_grady;
    Mat kernelx = (Mat_<float>(1, 2) << 1, -1);
    Mat kernely = (Mat_<float>(2, 1) << 1, -1);
    filter2D(patch, gradx, CV_32F, kernelx);
    filter2D(patch, grady, CV_32F, kernely);
    grad.create(patch.size(),CV_32F);
    for(int i=0;i<patch.rows;i++)
    {
        float* pgradx = gradx.ptr<float>(i);
        float* pgrady = grady.ptr<float>(i);
        float* pgrad = grad.ptr<float>(i);
        for(int j=0;j<patch.cols;j++)
            pgrad[j] = abs(pgradx[j])+abs(pgrady[j]);
    }
    return grad;
}

Mat combineT(Mat &R, Mat &t)
{
    Mat T = Mat::eye(4,4,CV_64F);
    R.copyTo(T(Range(0,3),Range(0,3)));
    t.copyTo(T(Range(0,3),Range(3,4)));
    return T;
}

void splitT(Mat T, Mat &R, Mat &t)
{
    R = T(Range(0,3),Range(0,3));
    t = T(Range(0,3),Range(3,4));
}

Point3d pointMultiply(Mat T, Point3d p)
{
    assert( (T.cols==4 && T.rows==4) || (T.cols==3 && T.rows==3) );
    if(T.cols == 4)
    {
        double x = T.at<double>(0,0)*p.x + T.at<double>(0,1)*p.y + T.at<double>(0,2)*p.z + T.at<double>(0,3);
        double y = T.at<double>(1,0)*p.x + T.at<double>(1,1)*p.y + T.at<double>(1,2)*p.z + T.at<double>(1,3);
        double z = T.at<double>(2,0)*p.x + T.at<double>(2,1)*p.y + T.at<double>(2,2)*p.z + T.at<double>(2,3);
        double w = T.at<double>(3,0)*p.x + T.at<double>(3,1)*p.y + T.at<double>(3,2)*p.z + T.at<double>(3,3);
        return Point3d(x/w, y/w, z/w);
    }
    else
    {
        double x = T.at<double>(0,0)*p.x + T.at<double>(0,1)*p.y + T.at<double>(0,2)*p.z;
        double y = T.at<double>(1,0)*p.x + T.at<double>(1,1)*p.y + T.at<double>(1,2)*p.z;
        double z = T.at<double>(2,0)*p.x + T.at<double>(2,1)*p.y + T.at<double>(2,2)*p.z;
        return Point3d(x, y, z);
    }
}

void saveImg(Mat &img, string path)
{
    static int num = 0;
    stringstream sout;
    sout << setfill('0')<<setw(6)<<num<<".png";
    path += sout.str();
    cout << path << endl;
    vector<int> compression_params;
    compression_params.push_back(IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    imwrite(path, img, compression_params);
    num++;
}


double distance1(const Point2f& a, const Point2f& b)
{
    return fabs(a.x-b.x) + fabs(a.y-b.y);
}

double distance2(const Point2f& a, const Point2f& b)
{
    return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}

double distance2Matx31d(const Mat& a, const Mat& b)
{
    return sqrt((a.at<double>(0)-b.at<double>(0))*(a.at<double>(0)-b.at<double>(0)) +
                (a.at<double>(1)-b.at<double>(1))*(a.at<double>(1)-b.at<double>(1)) +
                (a.at<double>(2)-b.at<double>(2))*(a.at<double>(2)-b.at<double>(2)));
}

double distance1(const Point3d& a, const Point3d& b)
{
    return fabs(a.x-b.x) + fabs(a.y-b.y) + fabs(a.z-b.z);
}

double distance2(const Point3d& a, const Point3d& b)
{
    return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}


// 把[min,max)分成N段，看input中每个元素属于哪一段，就把那个段的统计值+1，最后输出output为N个值，每个值是该段内input元素的数量
// 临界值归到右边的段，等于max的值忽略不统计，范围外的值也不统计
void getHistNum(vector<float>& input, float min, float max, int N, vector<int>& output)
{
    output.clear();
    for(int i=0;i<N;i++) output.push_back(0);
    for(int i=0;i<input.size();i++)
    {
        if(input[i]<min || input[i]>=max) continue;
        output[ int((input[i] - min)*N / (max - min)) ]++;
    }
}

// 跟上面类似，但不仅计算当前区间内的数量，还要加上左右各e个区间的数量
void getHistNumExpand(vector<float>& input, float min, float max, int N, int e, vector<int>& output)
{
    output.clear();
    for(int i=0;i<N;i++) output.push_back(0);
    for(int i=0;i<input.size();i++)
    {
        if(input[i]<min || input[i]>=max) continue;
        int idx = (input[i] - min)*N / (max - min);
        for(int j=-e;j<=e;j++)
        {
            if(idx+j>=0 && idx+j<N) output[idx+j]++;
        }
    }
}

Mat getpatch(Mat& img, Point2f center, int psize)
{
    Point2f topleft = center + Point2f(0.5-psize*0.5, 0.5-psize*0.5);
    Mat patch(Size(psize, psize), CV_32F);
    if(topleft.x < 0 || topleft.y < 0 || topleft.x+psize > img.cols || topleft.y+psize > img.rows)
        return Mat();

    Point2i itopleft;
    itopleft.x = cvFloor(topleft.x); // 左上角像素
    itopleft.y = cvFloor(topleft.y);
    float a = topleft.x - itopleft.x;
    float b = topleft.y - itopleft.y;
    float ip00 = (1.f - a)*(1.f - b);
    float ip01 = a*(1.f - b);
    float ip10 = (1.f - a)*b;
    float ip11 = a*b;

    for(int y=0;y<psize;y++)
    {
        const uchar* pimg = img.ptr<uchar>(itopleft.y+y);
        const uchar* pimg1 = img.ptr<uchar>(itopleft.y+y+1);
        float* ppatch = patch.ptr<float>(y);
        for(int x=0;x<psize;x++)
        {
            ppatch[x] = pimg[itopleft.x+x]*ip00 +
                        pimg[itopleft.x+x+1]*ip01 +
                        pimg1[itopleft.x+x]*ip10 +
                        pimg1[itopleft.x+x+1]*ip11;
        }
    }

    return patch;
}

double sparseSAD(Mat &patch1, Mat &patch2)
{
    assert(patch1.size() == patch2.size());

    double ret=0;
    int N=0;
    for(int y = 0; y < patch2.rows; y += 1)
    {
        float *data1 = patch1.ptr<float>(y);
        float *data2 = patch2.ptr<float>(y);
        for(int x = 0; x < patch2.cols; x += 1)
        {
            N++;
            ret += fabs(data1[x] - data2[x]);
            // ret += (data1[x] - data2[x])*(data1[x] - data2[x]);
        }
    }

    return ret / N;
}

struct timeval time_val;
struct timezone tz;
long gettime()
{
    gettimeofday(&time_val, &tz);
    return (time_val.tv_sec%10000)*1000000 + time_val.tv_usec;
}

int _kbhit()
{
    static const int STDIN = 0;
    static int initialized = 0;
    int bytesWaiting;
    if(!initialized)
    {
        // Use termios to turn off line buffering
        struct termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = 1;
    }
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

int getKey()
{
    if(_kbhit()) return getchar();
    else return 0;
}

//ax+by+cz+d=0 已经归一化，将点带入就是到平面的距离
void fitPlane(const vector<Point3d>& points, float* plane)
{
    // Estimate geometric centroid.
    Point3d centroid(0,0,0);
    for(auto &p: points)
        centroid += p;
    centroid *= 1./points.size();
    // Subtract geometric centroid from each point.
    Mat ptsm(points.size(), 3, CV_64F);
    for(int i=0;i<points.size();i++)
    {
        ptsm.at<double>(i,0) = points[i].x - centroid.x;
        ptsm.at<double>(i,1) = points[i].y - centroid.y;
        ptsm.at<double>(i,2) = points[i].z - centroid.z;
    }
    // Evaluate SVD of covariance matrix.
    Mat W, U, V;
    SVD::compute(ptsm, W, U, V);
    // Assign plane coefficients by singular vector corresponding to smallest singular value.
    plane[0] = V.at<double>(2,0);
    plane[1] = V.at<double>(2,1);
    plane[2] = V.at<double>(2,2);
    plane[3] = -(plane[0]*centroid.x + plane[1]*centroid.y + plane[2]*centroid.z);
    double pnorm = sqrt(plane[0]*plane[0] + plane[1]*plane[1] + plane[2]*plane[2]);
    plane[0] /= pnorm;
    plane[1] /= pnorm;
    plane[2] /= pnorm;
    plane[3] /= pnorm;
}


//ax+by+cz+d=0 已经归一化，将点带入就是到平面的距离
void fitPlaneParallelToVector(const vector<Point3d>& points, Point3d vec, float* plane)
{
    double u = vec.x;
    double v = vec.y;
    double w = vec.z;

    if(fabs(w) > fabs(u)) // 防止分母为零或接近零
    {
        // Estimate geometric centroid.
        Point2f centroid(0,0);
        for(auto &p: points)
            centroid += Point2f(p.x - u*p.z/w, p.y - v*p.z/w);
        centroid *= 1./points.size();
        // Subtract geometric centroid from each point.
        Mat ptsm(points.size(), 2, CV_64F);
        for(int i=0;i<points.size();i++)
        {
            ptsm.at<double>(i,0) = points[i].x - u*points[i].z/w - centroid.x;
            ptsm.at<double>(i,1) = points[i].y - v*points[i].z/w - centroid.y;
        }
        // Evaluate SVD of covariance matrix.
        Mat W, U, V;
        SVD::compute(ptsm, W, U, V);
        // Assign plane coefficients by singular vector corresponding to smallest singular value.
        plane[0] = V.at<double>(1,0);
        plane[1] = V.at<double>(1,1);
        plane[2] = -(plane[0]*u + plane[1]*v) / w;
        plane[3] = -(plane[0]*centroid.x + plane[1]*centroid.y);
    }
    else
    {
        // Estimate geometric centroid.
        Point2f centroid(0,0);
        for(auto &p: points)
            centroid += Point2f(p.y - v*p.x/u, p.z - w*p.x/u);
        centroid *= 1./points.size();
        // Subtract geometric centroid from each point.
        Mat ptsm(points.size(), 2, CV_64F);
        for(int i=0;i<points.size();i++)
        {
            ptsm.at<double>(i,0) = points[i].y - v*points[i].x/u - centroid.x;
            ptsm.at<double>(i,1) = points[i].z - w*points[i].x/u - centroid.y;
        }
        // Evaluate SVD of covariance matrix.
        Mat W, U, V;
        SVD::compute(ptsm, W, U, V);
        // Assign plane coefficients by singular vector corresponding to smallest singular value.
        plane[1] = V.at<double>(1,0);
        plane[2] = V.at<double>(1,1);
        plane[0] = -(plane[1]*v + plane[2]*w) / u;
        plane[3] = -(plane[1]*centroid.x + plane[2]*centroid.y);
    }
    double pnorm = sqrt(plane[0]*plane[0]+plane[1]*plane[1]+plane[2]*plane[2]);
    plane[0] /= pnorm;
    plane[1] /= pnorm;
    plane[2] /= pnorm;
    plane[3] /= pnorm;
}

void depthFromTriangulation(const Mat& Rcr, const Mat& tcr, const Mat& fr, const Mat& fc, double& depth, double& parallax)
{
    Mat f2 = Rcr * fr;
    Mat A(Matx32d(f2.at<double>(0), fc.at<double>(0),
                  f2.at<double>(1), fc.at<double>(1),
                  f2.at<double>(2), fc.at<double>(2)));
    Mat At = A.t();
    Mat AtA = At * A;
    Mat depth2 = -AtA.inv() * At * tcr;
    depth = depth2.at<double>(0);
    parallax = acos(f2.dot(fc)/norm(f2)/norm(fc))*57.3;
}

// ax+by+cz+d = 0
void pointProj2Plane(float p[3], float plane[4], float out[3])
{
    assert((plane[0]*plane[0]+plane[1]*plane[1]+plane[2]*plane[2])!=0);
    float t = (plane[0]*p[0] + plane[1]*p[1] + plane[2]*p[2] + plane[3]) / (plane[0]*plane[0] + plane[1]*plane[1] + plane[2]*plane[2]);
    // float foot[3];
    out[0] = p[0]-plane[0]*t;
    out[1] = p[1]-plane[1]*t;
    out[2] = p[2]-plane[2]*t;

}

Point3f pointProj2Plane(Point3f p, float plane[4])
{
    float p_[3], foot_[3];
    p_[0] = p.x;
    p_[1] = p.y;
    p_[2] = p.z;
    pointProj2Plane(p_, plane, foot_);
    return Point3f(foot_[0], foot_[1], foot_[2]);
}

Point3f pointProj2Plane(Point3f p, vector<float> plane)
{
    float plane_[4];
    plane_[0] = plane[0];
    plane_[1] = plane[1];
    plane_[2] = plane[2];
    plane_[3] = plane[3];

    return pointProj2Plane(p, plane_);
}

Vec3f pointProj2Plane(Vec3f p, Vec4f plane)
{
    float p_[3], plane_[4], foot_[3];
    p_[0] = p[0];
    p_[1] = p[1];
    p_[2] = p[2];
    plane_[0] = plane[0];
    plane_[1] = plane[1];
    plane_[2] = plane[2];
    plane_[3] = plane[3];
    pointProj2Plane(p_, plane_, foot_);
    return Vec3f(foot_[0], foot_[1], foot_[2]);
}

// 对2d点做单应变换
Point2f pointTrans(Mat H, Point2f p)
{
    assert(H.cols==3 && H.rows==3);
    Mat p0(Matx31d(p.x, p.y, 1));
    Mat p1 = H * p0;
    return Point2f(p1.at<double>(0)/p1.at<double>(2), p1.at<double>(1)/p1.at<double>(2));
}


// 直线(l1, l2) 与 平面 ax+by+cz+d=0 的交点
Point3d linePlaneIntersection(Point3d l1, Point3d l2, vector<float> p)
{
    Point3d v1 = l2 - l1;
    v1 /= norm(v1);
    double num = p[0]*l1.x + p[1]*l1.y + p[2]*l1.z + p[3];
    double den = p[0]*v1.x + p[1]*v1.y + p[2]*v1.z;
    double n = num/fabs(den); // 注意此处直线是有方向的
    Point3d p3d = l1 + n * v1;
    return p3d;
}


