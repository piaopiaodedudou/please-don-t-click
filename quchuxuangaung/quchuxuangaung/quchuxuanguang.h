#include<iostream>
#include<opencv2\opencv.hpp>
#include<stdarg.h>
#include<fstream>

using namespace std;
using namespace cv;

class GuidedFilterImpl;

class GuidedFilter
{
public:
    GuidedFilter(const cv::Mat &I, int r, double eps);
    ~GuidedFilter();

    cv::Mat filter(const cv::Mat &p, int depth = -1) const;

private:
    GuidedFilterImpl *impl_;
};

cv::Mat guidedFilter(const cv::Mat &I, const cv::Mat &p, int r, double eps, int depth = -1);

void CalRatio(vector<Mat>&, Mat&);

void MultiShow(string,int hangshu,int lieshu,Mat,...);

void DarkChannelFn(Mat&, Mat&);

void MinFilter(Mat&, int, Mat&);

void GetA(vector<Mat>, Mat&, double&);

void GetT(Mat&, double, Mat&);

Mat guidedFilter2(cv::Mat I, cv::Mat p, int r, double eps);

Mat fastGuidedFilter(cv::Mat I_org, cv::Mat p_org, int r, double eps, int s);

void GetTR(Mat& darkimg, double A, Mat& t);

void findelli(Mat&, vector<RotatedRect>&);
void matchlights(vector<RotatedRect>&, vector<vector<RotatedRect>>&);
void rangePoints(vector<RotatedRect>&);
void findtoplight(vector<vector<RotatedRect>>::iterator, vector<vector<RotatedRect>>&);
void findtoplight(vector<vector<RotatedRect>>&);
void combineelli(vector<vector<RotatedRect> >&);
void AccurateLights(vector<vector<RotatedRect>>&);
void combineContours(vector<vector<Point>>&, vector<vector<Point>>&);
void BiggerRect(Rect&, int, int);
void MakeRect(vector<vector<RotatedRect>>&, vector<Rect>&);
void DeleteVert(vector<Rect>&);
const int kersize = 7;
const double cutlim = 0.4;