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
double calOverlapRatio(Rect,Rect); //double calOverlapRatio(Rect&,Rect&)����汾Ӱ��Ч��
vector<int> IsDisMiss(vector<Rect>&,vector<Rect>&,vector<Rect>&);
void IsMoving(vector<vector<Rect>>&,vector<vector<Rect>>&);
void TideHistInf(vector<vector<Rect>>&,vector<vector<Rect>>&); //δʹ��
vector<int> IsChanged(vector<vector<Rect>>&);
vector<MoveMessage> DecideMove(vector<vector<Rect>>&);
vector<MoveMessage> MatchRectMove(vector<Rect>&,vector<Rect>&);
void FindnewRect(vector<Rect>&,vector<Rect>&,vector<Rect>&);
void CalLightMask(Mat&,Mat&);
void GetLight(Mat&,vector<Rect>&);
void IsLightROI(vector<Mat>&,vector<LightMessage>&);
void CalLightOn(int,int&,int&);
double vmax(vector<double>);
const int sampleszie = 24;    //ѵ��ʱʹ�õ������ߴ�
const int changeRange = 10;   // ��¼����ʷ֡��
const double multiscale = 1.3;//detectmutiscale�е�scale 
const double scale = 1.5;     //�ߴ�仯��scale
const int Imgheight = 720;
const int Imgwidth = 1280;
const int onlimt = 3;
const int LightChangeNum = 15;//15