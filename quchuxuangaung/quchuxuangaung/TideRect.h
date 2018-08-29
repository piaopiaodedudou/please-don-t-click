#include<iostream>
#include<opencv2\opencv.hpp>
#include<vector>
#include<fstream>
using namespace std;
using namespace cv;
struct MoveMessage{
	double widthRate;
	//double xRate;
	int width;
	int x;
};
struct LightMessage{
	int up;
	int left;
	int right;
};
void detectcars(vector<Rect>&);
void CombineRect(vector<Rect>& rect);

vector<Rect>& comHistRect(vector<Rect>&,vector<Rect>&);
//void comHistRect(vector<Rect>&,vector<Rect>&,vector<Rect>&);
void rangeRect(vector<Rect>&);
bool IsInsideRect(Rect&,Rect&);
double calOverlapRatio(Rect,Rect); //double calOverlapRatio(Rect&,Rect&)这个版本影响效率
vector<int> IsDisMiss(vector<Rect>&,vector<Rect>&,vector<Rect>&);
void IsMoving(vector<vector<Rect>>&,vector<vector<Rect>>&);
void TideHistInf(vector<vector<Rect>>&,vector<vector<Rect>>&); //未使用
vector<int> IsChanged(vector<vector<Rect>>&);
vector<MoveMessage> DecideMove(vector<vector<Rect>>&);
vector<MoveMessage> MatchRectMove(vector<Rect>&,vector<Rect>&);
void FindnewRect(vector<Rect>&,vector<Rect>&,vector<Rect>&);
void CalLightMask(Mat&,Mat&);
void GetLight(Mat&,vector<Rect>&);
void IsLightROI(vector<Mat>&,vector<LightMessage>&);
void CalLightOn(int,int&,int&);
double vmax(vector<double>);
const int sampleszie = 24;    //训练时使用的样本尺寸
const int changeRange = 10;   // 记录的历史帧数
const double multiscale = 1.3;//detectmutiscale中的scale 
const double scale = 1.5;     //尺寸变化的scale
const int Imgheight = 720;
const int Imgwidth = 1280;
const int onlimt = 3;
const int LightChangeNum = 15;//15