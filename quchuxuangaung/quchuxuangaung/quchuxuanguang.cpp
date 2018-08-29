#include"quchuxuanguang.h"

//视频数据：E:\视频数据材料\SAIC_video\好好开车\20161013
int main()
{
	//Mat img1 = imread("751.jpg");
	//////Mat darkimg = imread("testdark.png");
	////Mat Rratio,Gratio;
	//////CalRatio(img1, Rratio, Gratio);
	//////imshow("Rratio",Rratio);
	////////imshow("Gratio",Gratio);
	//////imshow("yuantu",img1);
	//////waitKey(0);
	////vector<Mat> vimg1;
	////split(img1,vimg1);
	//Mat guideimg,Mimg,t;
	//////imshow("vimg1[2]",vimg1[2]);
	////Mimg = vimg1[2] - min(vimg1[0],vimg1[1]);
	////imshow("",Mimg);
	////double A,AR,AG,AB;
	//DarkChannelFn(img1,guideimg);
	//imshow("",guideimg);
	//////GetA(vimg1, Mimg, A);
	////A = 1;
	//////AR = 200.0;
	//////AG = 100.0;
	//////AB = 100.0;
	//////Mat TR,TG,TB;
	////GetT(Mimg, A, t);
	//////t = t*0.9;
	////vimg1[2].convertTo(vimg1[2],t.type(),1.0/255);
	////vimg1[1].convertTo(vimg1[1],t.type(),1.0/255);
	////vimg1[0].convertTo(vimg1[0],t.type(),1.0/255);

	////vimg1[2] = ((vimg1[2] - A)/(max(t,0.1))) + A;

	//////twice:
	//////Mimg = vimg1[2] - max(vimg1[0],vimg1[1]);
	//////A = 1;
	//////GetT(Mimg, A, t);
	//////vimg1[2] = ((vimg1[2] - A)/(max(t,0.1))) + A;
	//////

	//////vimg1[1] = ((vimg1[1] - A)/(max(t,0.1))) + A;	
	//////vimg1[0] = ((vimg1[0] - A)/(max(t,0.1))) + A;	

	//////GetT(Mimg, AR, TR);
	//////GetT(Mimg, AG, TG);
	//////GetT(Mimg, AB, TB);
	//////vimg1[2] = ((vimg1[2] - AR)/(max(TR,0.1))) + AR;
	//////vimg1[1] = ((vimg1[1] - AG)/(max(TG,0.1))) + AG;	
	//////vimg1[0] = ((vimg1[0] - AB)/(max(TB,0.1))) + AB;
	////int r = 15;
	////double eps = 0.0001;
	////eps *= 255 * 255;
	//////imshow("B",vimg1[0]);
	//////imshow("G",vimg1[1]);
	//////imshow("R",vimg1[2]);
	//////Mimg = vimg1[2] - max(vimg1[0],vimg1[1]);
	//////imwrite("testdark.png",Mimg);
	////merge(vimg1,guideimg);
	//////CalRatio(guideimg, Rratio, Gratio);
	////imshow("Rraio",Rratio);
	//////guideimg = guidedFilter(vimg1[2],vimg1[0],r,eps);
	////imshow("guideR",guideimg);
	//////guideimg.convertTo(guideimg,CV_8UC3,255);
	//////imwrite("R578.jpg",guideimg);
	//////imshow("D",);
	//////guideimg = guidedFilter(img1,img1,r,eps);
	//////guideimg = guidedFilter(img1,img1,r,eps);
	//////guideimg = guidedFilter(img1,img1,r,eps);
	//////imshow("guide",guideimg);
	////imshow("yuantu",img1);
	//////vector<Mat> vimg;
 //////   split(img1,vimg);
	//////minimg = min(vimg[0],min(vimg[1],vimg[2]));
	//////MultiShow("暗通道与红色分量",1,2,minimg,vimg[2]);
	//////imshow("暗通道",minimg);
	//////imshow("red",vimg[2]);
	//////imshow("G",vimg[1]);
	//////imshow("B",vimg[0]);
	//waitKey(0);


	////Mat result;

	////DarkChannelFn(img1,result);
	////imshow("minimg",result);
	////waitKey(0);

	VideoCapture cap("F:\\cutvideo\\夜间\\黑车照明.mp4");
	if(!cap.isOpened())
	{
		return -1;
	}

	int numframe = cap.get(CV_CAP_PROP_FRAME_COUNT);
	cout<<"zhenshu:"<<numframe<<endl;
	cap.set(CV_CAP_PROP_POS_FRAMES,700);

	Mat frame,result,Rratio;

	for(int i = 0;i < numframe;++i)
	{
		double t = (double)getTickCount();
		cap>>frame;
		
		DarkChannelFn(frame,result);
		cout<<"帧数："<<i<<endl;
		
		//CalRatio(result, Rratio, Gratio);
		//imwrite("751.jpg",frame);
		//result.convertTo(result,CV_8UC1,255);
		//cout<<"result:"<<result(Range(100,110),Range(11,110))<<endl;
		//imshow("frame",result);
		t = (double)getTickCount() - t;
		double T = t*1000./getTickFrequency();
		cout<<"T = "<<T<<endl;
		//imwrite("D1108.jpg",result);
		waitKey(0);
	}
}