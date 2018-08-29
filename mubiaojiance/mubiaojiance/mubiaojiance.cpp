#include<iostream>
#include<opencv2\opencv.hpp>
#include"mubiaojiace.h"


using namespace std;
using namespace cv;

/************************************************
*��Ŀ��ֱ��ʹ�÷��������м��
************************************************/

int main()
{
	int testSumRect = 0;
	String cascadeName = "F:\\shejichengxv\\CarDetect\\CarDetect\\output_801_24_6easyNeg_tlfbest.xml";
	CascadeClassifier ccf;
	ccf.load(cascadeName);

	//VideoCapture cap("F:\\cutvideo\\�ռ�\\�ڳ���ת��ɲ��.mp4");
	VideoCapture cap("F:\\cutvideo\\ҹ��\\����ѣ��.mp4");
	//ofstream outfile("��������2.txt");
	//ofstream outfile("���θ���.txt");
	if(!cap.isOpened())
		return -1;
	if(!ccf.load(cascadeName))
	{
		cout<<"false to open the classifier"<<endl;
		return 0;
	}

	int numframe = cap.get(CV_CAP_PROP_FRAME_COUNT);
	int imgH = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	int imgW = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	int FPS = cap.get(CV_CAP_PROP_FPS);
	//cout<<"FPS = "<<FPS<<endl;
	//cout<<"Imgheight = "<<imgH<<endl;
	//cout<<"Imgwidth = "<<imgW<<endl;

	Mat frame(imgH,imgW,CV_8UC3,Scalar(0,0,0));
	Mat Gray(frame.size(),frame.type());
	vector<Rect> cars;
	//cap.set(CV_CAP_PROP_POS_FRAMES,800);//���ֺ�����
	//cap.set(CV_CAP_PROP_POS_FRAMES,120);
	for(int i = 0;i < numframe;++i)
	{
		
		cap >> frame;    
		double t = (double)getTickCount();
		cvtColor(frame,Gray,CV_BGR2GRAY);
		detectcars(frame,ccf,cars);
		vector<Mat> result;

		//ͳ�Ƽ�⵽�ľ�������
	    testSumRect = testSumRect + cars.size();

		GetLight(frame,cars);
		for(size_t j = 0;j < cars.size();++j)
		{
			Point pt1(cars[j].x,cars[j].y);
			Point pt2(cars[j].x+cars[j].width,cars[j].y+cars[j].height);
			rectangle(frame,pt1,pt2,Scalar(0,0,255));
		}
		imshow("result",frame);
		
		 
		cvMoveWindow("result",0,0);
		t = (double)getTickCount() - t;
		double T = t*1000./getTickFrequency();
		cout<<"T = "<<T<<endl;
		cout<<"֡����"<<i<<endl;
		waitKey(1);
	}
		//outfile<<"֡�� "<<numframe<<endl<<"���θ���"<<testSumRect<<endl;
}