/*******************************************************
*该程序用于对目标车辆的检测，是在VD_ZWK8_24的思想基础上
*进行的加速优化版本（包括对算法重新设计，部分不需要功能的
*删除，编程方面的优化）
*编写：李堃
*时间：3.03 - 3.16
*闪现消除思路来源：赵伟康
********************************************************/
#include"TideRect.h"

vector<Rect> ComRect;
vector<vector<Rect>> HistRect;
vector<vector<Rect>> OriHistRect;
vector<vector<LightMessage>> LightLR;
int Wrong = 0;

/*******************************************************
*车辆检测主体程序
********************************************************/
void detectcars(vector<Rect>& cars)
{
	rangeRect(cars);                                                             //对检测目标进行排序
    CombineRect(cars);                                                           //合并重复框

	cars = comHistRect(ComRect,cars);                                            //消除帧与帧之间的抖动(版本二)                
	vector<Rect> savecars(cars);
	
	if(HistRect.size() < changeRange)                                            //储存前十帧作为历史信息
	{
		HistRect.push_back(savecars);
	}
	else
	{
		vector<Rect> Vnew;
		FindnewRect(*(HistRect.end() - 1),cars,Vnew);                            //找到当前帧的变化的矩形（新出现或者刚消失）
		if(Vnew.size() != 0)
		{  
			IsMoving(HistRect,HistRect);                                          //暂时”消除历史帧中的闪灭“
			vector<int> result = IsDisMiss(*(HistRect.end() - 1),cars,Vnew);
			for(auto it = result.begin();it != result.end(); ++it)
			{
				if(!*it)
				{
					if(Wrong < 4)
					{
						if((HistRect.end() - 1)->size() > cars.size() || (HistRect.end() - 1)->size() == cars.size() )
						{
							for(auto iter = Vnew.begin(); iter != Vnew.end(); ++iter)
							{
								cars.push_back(*iter);
							}
							savecars = cars;
							Wrong++;
						}
						else
							Wrong = 0;
					}
				}
				else
				{
					if((HistRect.end() - 1)->size() < cars.size())
					{
						for(auto nit = Vnew.begin();nit != Vnew.end(); ++nit)
						{
							auto cit = cars.begin();
							while(cit != cars.end())
							{
								if(calOverlapRatio(*nit,*cit) > 0.8)
								{
									cit = cars.erase(cit);
								}
								else
								{
									++cit;
								}
							}
						}
					}
				}
			}
		}
		rangeRect(savecars);
		HistRect.push_back(savecars);
		HistRect.erase(HistRect.begin());                                         //更新历史信息
	}
	cout<<"矩形框的数量: "<<cars.size()<<endl;

}
/********************************************************
*对重复检测出来的目标框进行删除合并（只留大的）
*********************************************************/
void CombineRect(vector<Rect>& rect)
{
	if(rect.size() > 1)
	{
		auto iter = rect.begin();
		while(iter != rect.end() - 1)
		{
			if(calOverlapRatio(*iter,*(iter + 1)) > 0.5)
			{
				(iter + 1)->x = min(iter->x, (iter + 1)->x); //无法去除不包含的垂直的部分
				(iter + 1)->y = min(iter->y, (iter + 1)->y);                                     
				(iter + 1)->width = max(iter->width,(iter + 1)->width);        
				(iter + 1)->height = max(iter->height, (iter + 1)->height);
				iter = rect.erase(iter);
			}
			else
			{
				++iter;
			}
		}
	}
}
/********************************************************
*对一帧中的所有检测目标排序，用于准确进行帧与帧之间
*检测框的对比，进行消除抖动
*********************************************************/
void rangeRect(vector<Rect> &Rrect)
{
	if(Rrect.size() > 1)
	{
		Rect temprect;
		auto iter = Rrect.begin();
		while(iter != Rrect.end() - 1)
		{
			auto plusit = iter + 1;
			while(plusit != Rrect.end())
			{
				if(iter->x > plusit->x)
				{
					temprect = *(plusit);
					*(plusit) = *iter;
					*iter = temprect;
				}
				else
					++plusit;
			}
			++iter;

		}
	}
}
/********************************************************
*计算面积重叠率，用于判定消除抖动的条件（为何不能用取址来做）
*********************************************************/
double calOverlapRatio(Rect rect1,Rect rect2)
{
	Rect tempRect = rect1&rect2;
	double ratio = static_cast<double>(tempRect.area())/min(rect1.area(),rect2.area());
	return ratio;
}
/********************************************************
*判定新帧的检测框是否属于抖动(参数待定)
*********************************************************/
bool IsInsideRect(Rect& BigRect,Rect& SmallRect)
{	
	if((calOverlapRatio(BigRect,SmallRect) > 0.5))
	{
		return true;
	}
	else
	{
		return false;
	}
}
/********************************************************
*消除抖动（版本二）
*********************************************************/
vector<Rect>& comHistRect(vector<Rect>& comRect,vector<Rect>& newRect)
{
	if(comRect.size() == newRect.size())
	{
		auto citer = comRect.begin();
		auto niter = newRect.begin();
		while(citer != comRect.end() && niter != newRect.end())
		{
			if(IsInsideRect(*citer,*niter))
			{
				niter->x = niter->x*0.2 + citer->x*0.8;
				niter->y = niter->y*0.2 + citer->y*0.8;
				niter->height = niter->height*0.2 + citer->height*0.8;
				niter->width = niter->width*0.2 + citer->width*0.8;
			}
			++citer;
			++niter;
		}
		comRect = newRect;
	}
	comRect = newRect;
	return newRect;
}
/********************************************************
*判断目标是否应该消失，目标应该消失返回true()
*********************************************************/
//未使用，代码部分见mubiaojiance
/********************************************************
*师兄的延时算法去除闪现和闪灭，对偶尔出现的效果很好
*对新目标检测时产生的抖动效果有限，不理想
*********************************************************/
vector<int> IsDisMiss(vector<Rect>& HisRect,vector<Rect>& newRect,vector<Rect>& newrect)
{
	vector<int> result;
	if(HisRect.size() < newRect.size())
	{
		double avery;
		double height = 0;
		for(vector<Rect>::iterator iter = HisRect.begin();iter != HisRect.end(); ++iter)
		{
			height = height + iter->y;
		}
		avery = height/HisRect.size();
		auto nit = newrect.begin();
		while(nit != newrect.end())
		{
			if(abs(nit->y - avery) > nit->height)
			{
				result.push_back(1);
			}
			else
			{
				int sum = 0;
				for(auto vHistiter = HistRect.rbegin();vHistiter != HistRect.rbegin() + 5; ++vHistiter)
				{
					for(auto Histiter = vHistiter->begin();Histiter != vHistiter->end(); ++Histiter)
					{
						if(calOverlapRatio(*nit,*Histiter) > 0.8)
						{
							++sum;
						}
					}
				}
				if(sum == 5)
				{
					result.push_back(0);
				}
				else
				{
					result.push_back(1);
				}
			}
			++nit;
		}
	}
	else if(HisRect.size() > newRect.size())
	{
		auto nit = newrect.begin();
		while(nit != newrect.end())
		{
			int sum = 0;
			for(auto vHistiter = HistRect.rbegin();vHistiter != HistRect.rbegin() + 5; ++vHistiter)				
			{
				for(auto Histiter = vHistiter->begin();Histiter != vHistiter->end(); ++Histiter)
				{
					if(calOverlapRatio(*nit,*Histiter) > 0.8)
					{
						++sum;
					}
				}
			}
			if(sum == 5)
			{
				result.push_back(0);
			}
			else
			{
				result.push_back(1);
			}
			++nit;
		}
	}
	else
		result.push_back(0);
	return result;
}
/********************************************************
*对采集到的历史信息进行处理，消除闪现和闪灭(版本二)
*采用迭代器
*********************************************************/
void IsMoving(vector<vector<Rect>>& vRect,vector<vector<Rect>>& Hist)
{
	vector<int> changePt;
	changePt = IsChanged(vRect);
	auto ptit = changePt.begin();
	while(!changePt.empty() && ptit != changePt.end() - 1)
	{
		if((vRect.begin() + *ptit)->size() > (vRect.begin() + *(ptit + 1))->size() && (*ptit - *(ptit + 1) < 3))
		{
			for(int i = 1;i < *(ptit + 1) - *ptit + 1;++i)
			{
				*(vRect.begin() + *(ptit) + i) = *(vRect.begin() + *ptit);
			}
			if((vRect.begin() + *ptit)->size() == (vRect.begin() + *(ptit + 1))->size())
			{
				ptit = changePt.erase(ptit);
				if(ptit == changePt.end() - 1)
				{
					ptit = changePt.erase(ptit);
					break;
				}
			}
			else
			{
				++ptit;
			}
		}

		if(!changePt.empty() && ptit != changePt.end() - 1)
		{
			if((vRect.begin() + *ptit)->size() < ((vRect.begin() + *(ptit + 1))->size()))
			{
				double avery;
				double sum = 0;
				for(vector<Rect>::const_iterator frit = (vRect.begin() + *ptit)->begin();frit != (vRect.begin() + *ptit)->end();++frit)
				{
					sum = sum + frit->y;
				}
				avery = sum/((vRect.begin() + *ptit)->size());

				auto it = (vRect.begin() + *ptit + 1)->begin();
				while(it != (vRect.begin() + *ptit + 1)->end())
				{
					if(abs(it->y - avery) > it->height)
					{
						it = (vRect.begin() + *ptit + 1)->erase(it);
					}
					else
						++it;
				}
			}
		}
		++ptit;
	}
	Hist = vRect;
}
/********************************************************
*判断输入的向量中元素向量的尺寸是否出现变化
*********************************************************/
vector<int> IsChanged(vector<vector<Rect>>& vRect)
{
	vector<int> changePt;
	int i = 0;
	for(vector<vector<Rect>>::const_iterator iter = vRect.begin();iter != vRect.end() - 1; ++iter,++i)
	{
		if((*iter).size() != (*(iter + 1)).size())
		{
			changePt.push_back(i);
		}
	}
	return changePt;
}
/********************************************************
*对目标数量不变的情况下，对各个目标的长度以及
*起始点X的坐标变化进行线性拟合（最小二乘法）
*返回值为2*size的向量
*第一维是长度的拟合直线的斜率
*第二维是X坐标的拟合直线的斜率
*********************************************************/
vector<MoveMessage> DecideMove(vector<vector<Rect>>& HRect)
{
	int vsize;
	vector<double> num;
	for(vector<vector<Rect>>::const_iterator it = HRect.begin();it != HRect.end();++it)
	{
		num.push_back(it->size());
	}
	vsize = static_cast<int>(vmax(num));

	vector<int> mark;
	vector<vector<int> > vWidth(vsize),vX(vsize);   
	vector<MoveMessage> result;
	MoveMessage movemessage;

	auto iter = HRect.begin();
	while(iter != HRect.end() - 1)
	{
		if(iter->size() == (iter + 1)->size())
		{
			for(int i = 0;i < iter->size();++i)
			{
				if(calOverlapRatio((*iter)[i],(*(iter + 1))[i]) > 0.5)
				{
					vWidth[i].push_back((*iter)[i].width);
					vX[i].push_back((*iter)[i].x);
				}
				else if((HRect.end() - iter) > 4)
				{
					if(vWidth.begin()->size() != 0)
					{
						for(vector<vector<int>>::iterator wit = vWidth.begin(),xit = vX.begin();
							wit != vWidth.end(),xit != vX.end(); ++wit,++xit)
						{
							wit->clear();
							xit->clear();
						}
					}
					vector<vector<Rect>> tempvec(iter + 1,HRect.end());
					result = DecideMove(tempvec);
					return result;
				}
				else
				{
					if(vWidth.begin()->size() != 0)
					{
						for(vector<vector<int>>::iterator wit = vWidth.begin(),xit = vX.begin();
							wit != vWidth.end(),xit != vX.end(); ++wit,++xit)
						{
							wit->clear();
							xit->clear();
						}
					}
				}
			}
		}
		else if(HRect.end() - iter > 4)
		{
			if(vWidth.begin()->size() != 0)
			{
				for(vector<vector<int>>::iterator wit = vWidth.begin(),xit = vX.begin();
					wit != vWidth.end(),xit != vX.end(); ++wit,++xit)
				{
					wit->clear();
					xit->clear();
				}
				vector<vector<Rect>> tempvec(iter + 1,HRect.end());
				result = DecideMove(tempvec);
				return result;
			}
		}
		else
		{
			if(vWidth.begin()->size() != 0)
			{
				for(vector<vector<int>>::iterator wit = vWidth.begin(),xit = vX.begin();
					wit != vWidth.end(),xit != vX.end(); ++wit,++xit)
				{
					wit->clear();
					xit->clear();
				}
			}
		}
		++iter;
	}
	if(!vWidth.empty())
	{
		auto wit = vWidth.begin();
		while(wit != vWidth.end())
		{
			if(wit->size() == 0)
			{
				wit = vWidth.erase(wit);
			}else
				++wit;
		}
	}
	if(!vWidth.empty())
	{
		auto xit = vX.begin();
		while(xit != vX.end())
		{
			if(xit->size() == 0)
			{
				xit = vX.erase(xit);
			}else
				++xit;
		}
	}
	if(!vWidth.empty() && vWidth[0].size() > 4)
	{
		vector<Point2f> vPw/*,vPx*/;
		Point2f Pw/*,Px*/;

		for(vector<vector<int>>::const_iterator wit = vWidth.begin(),xit = vX.begin();
			wit != vWidth.end(),xit != vX.end();++wit,++xit)
		{
			for(int i = 0;i < wit->size();++i)
			{
				Pw.x = i;
				Pw.y = (*wit)[i];
				//Px.x = i;
				//Px.y = (*xit)[i];
				vPw.push_back(Pw);
				//vPx.push_back(Px);
			}
			Vec4f wlines/*,xlines*/;
			fitLine(vPw,wlines,CV_DIST_L2,0,0.01,0.01);
			//fitLine(vPx,xlines,CV_DIST_L2,0,0.01,0.01);
			movemessage.widthRate = (wlines[1]/wlines[0]);
			//movemessage.xRate = (xlines[1]/xlines[0]);
			movemessage.width = (*(wit->end() - 1));
			movemessage.x = *(xit->end() - 1);
			result.push_back(movemessage);
		}
	}
	return result;
}

/********************************************************
*运动信息的统计，精确匹配的统计，不是刷新帧数的统计
*********************************************************/

/********************************************************
*找出前后两帧中新出现的矩形
*********************************************************/
void FindnewRect(vector<Rect>& forerect,vector<Rect>& newrect,vector<Rect>& Vnew)
{	
	vector<double>ratio;
	if(forerect.size() != 0 && newrect.size() != 0)
	{
		if(forerect.size() == newrect.size() || forerect.size() < newrect.size())
		{
			for(auto/*vector<Rect>::const_iterator*/ niter = newrect.begin(); niter != newrect.end(); ++niter)
			{
				for(auto/*vector<Rect>::const_iterator*/ fiter = forerect.begin(); fiter != forerect.end(); ++fiter)
				{
					ratio.push_back(calOverlapRatio(*niter,*fiter));

				}
				if(vmax(ratio) < 0.3)
				{
					Vnew.push_back((*niter));
				}
				ratio.clear();
			}
		}
		else if(forerect.size() > newrect.size())
		{
			for(vector<Rect>::const_iterator fiter = forerect.begin(); fiter != forerect.end(); ++fiter)
			{
				for(vector<Rect>::const_iterator niter = newrect.begin(); niter != newrect.end(); ++niter)
				{
					ratio.push_back(calOverlapRatio(*fiter,*niter)); //为什么不能用迭代器表示地址呢？
				}
				if(vmax(ratio) < 0.3)
				{
					Vnew.push_back((*fiter));
				}
				ratio.clear();
			}
		}
	}
	else if(forerect.size() == 0 && newrect.size() != 0)
	{
		Vnew = newrect;
	}
	else if(forerect.size() != 0 && newrect.size() == 0)
	{
		Vnew = forerect;
	}
	else if(forerect.size() == 0 && newrect.size() == 0)
	{
		Vnew.clear();
	}
}
/********************************************************
*返回向量中的最大值
*********************************************************/
double vmax(vector<double> vec)
{
	double maxn = 0;
	for(vector<double>::const_iterator iter = vec.begin();iter != vec.end(); ++iter)
	{
		if(maxn < *iter)
		{
			maxn = *iter;
		}
	}
	return maxn;
}
/********************************************************
*对出现变化的矩形与前帧进行匹配，找到对应的运动信息
*********************************************************/
vector<MoveMessage> MatchRectMove(vector<Rect>& foreRect,vector<Rect>& newRect)
{
	vector<MoveMessage> result;
	vector<MoveMessage> xielv;
	xielv = DecideMove(HistRect);
	if(xielv.size() != 0)
	{
		for(vector<Rect>::iterator nit = newRect.begin();nit != newRect.end(); ++nit)
		{
			int num = 0;
			for(vector<Rect>::iterator itf = foreRect.begin();itf != foreRect.end(); ++itf,++num)
			{
				if(calOverlapRatio(*nit,*itf) > 0.6)
				{
					if(num < xielv.size())
					{
						result.push_back(xielv[num]);
					}
				}
			}
		}
	}
	return result;
}
/********************************************************
*检测灯和灯亮的位置，输出位置mask，对红色高分量情况敏感（即：
*对红色，黄色，紫色很敏感，对光照不敏感）
*********************************************************/
void CalLightMask(Mat& src,Mat& Mask)
{

	//ofstream ouf("Mask.txt");
	vector<Mat> vsrc;
	Mat img(src.size(),CV_32FC3),
		minussrc(src.size(),CV_32FC1),
		minsrc(src.size(),CV_8UC1),
		maxsrc(src.size(),CV_8UC1),R,G,B;

	src.convertTo(img,CV_32F);
	split(img,vsrc);
	normalize(vsrc[0],vsrc[0],1,0);
	normalize(vsrc[1],vsrc[1],1,0);
	normalize(vsrc[2],vsrc[2],1,0);

	min(vsrc[0],vsrc[1],minsrc);
	//absdiff(vsrc[2],minsrc,R);

	//absdiff(vsrc[1],minsrc,G);
	//R = vsrc[2] - minsrc;  //差值是否应该去绝对值待验证
	min(vsrc[2],minsrc,minsrc);

	absdiff(vsrc[2],minsrc,R);
	absdiff(vsrc[1],minsrc,G);
	absdiff(vsrc[0],minsrc,B);

	max(vsrc[0],vsrc[1],maxsrc);
	max(vsrc[2],maxsrc,maxsrc);

	absdiff(maxsrc,minsrc,minussrc);
	
	divide(R,(minussrc + 0.00001),Mask);
	Mat MaskG,MaskB;
	divide(G,(minussrc + 0.00001),MaskG);
	//divide(B,(minussrc + 0.00001),MaskB );
	threshold(Mask,Mask,0.995,255,0/*THRESH_TOZERO*/); //灯
	//imshow("R",Mask);
	threshold(MaskG,MaskG,0.3,255,0/*THRESH_TOZERO*/); 
	//imshow("G",MaskG);
	//threshold(MaskB,MaskB,0.05,255,1/*THRESH_TOZERO*/);
	//imshow("B",MaskB);
	//imwrite("mask.jpg",Mask);
	//waitKey(1);
	//ouf<<"Mask"<<Mask<<endl;
	//ouf<<vsrc[2].mul(Mask)<<endl;
	//threshold(maxsrc.mul(Mask),Mask,2,1,0);       //灯亮
	//imshow("MASK",MaskG);
	Mask = Mask.mul(MaskG);
	//Mask = Mask.mul(MaskB);
	Mat element = getStructuringElement( MORPH_RECT,
                                       Size(3,3));

	dilate(Mask,Mask,element);
	//imshow("",Mask);
	//minAreaRect()
	
}
/********************************************************
*对输入的帧中的选中的矩形部分生成Mask
*********************************************************/
void GetLight(Mat& frame,vector<Rect>& cars)
{
	vector<Mat> vMask;
	vector<LightMessage> islight;
	for(auto iter = cars.begin();iter != cars.end();++iter)
	{
		int bigpix = iter->width * 0.05;
		iter->x = (iter->x > bigpix) ? (iter->x - bigpix) : 0;
		iter->width = (iter->x + iter->width + 2*bigpix < frame.cols)?(iter->width + 2*bigpix):frame.cols - iter->x;
		iter->height = 0.6*iter->height;
		Mat Mask;
		CalLightMask(frame(*iter),Mask);
		vMask.push_back(Mask);
	}
	IsLightROI(vMask,islight);

	LightLR.push_back(islight);
	if(LightLR.size() == LightChangeNum + 1)
	{
		LightLR.begin() = LightLR.erase(LightLR.begin());
	}

	OriHistRect.push_back(cars);
	if(OriHistRect.size() == LightChangeNum + 1)
	{
		OriHistRect.erase(OriHistRect.begin());
	}

	for(int i = 0;i < cars.size();++i)
	{

		if((islight[i].up == 1 && islight[i].right == 1 && islight[i].left == 1) ||
			(islight[i].up == 1 && (/*islight[i].right == 2 ||*/ islight[i].right == 1) && islight[i].left == 0) ||
			(islight[i].up == 1 && islight[i].right == 0 && (/*islight[i].left == 2 ||*/ islight[i].left == 1)))
		{				
			putText(frame,"stop",Point(cars[i].x,cars[i].y),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),2);
		}
		if(islight[i].up == 0 && islight[i].left == 2 && (islight[i].right == 0 || islight[i].right == 1))
		{
			if(LightLR.size() == LightChangeNum )
			{
				int sumL,sumR;
				CalLightOn(i,sumL,sumR);
				//if((sumL>8 && sumL<12) ||(sumL<4))
				if(sumL<3)
				{
					putText(frame,"left",Point(cars[i].x,cars[i].y),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),2);
				}
			}
		}
		if(islight[i].up == 0 && (islight[i].left == 0 || islight[i].left == 1)&& islight[i].right == 2)
		{
			if(LightLR.size() == LightChangeNum )
			{
				int sumL,sumR;
				CalLightOn(i,sumL,sumR);
				/*if(sumR < onlimt)*/
				//if((sumR>8 && sumR<12) ||(sumR<4))
				if(sumR<3)
				{
					putText(frame,"right",Point(cars[i].x,cars[i].y),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),2);
				}
			}
		}
		if(islight[i].up == 1 && islight[i].left ==1 && islight[i].right == 2)
		{
			if(LightLR.size() == LightChangeNum )
			{
				int sumL,sumR;
				CalLightOn(i,sumL,sumR);
				/*if(sumR < onlimt)*/
				//if((sumR>8 && sumR<12) || (sumR<4))
				if((sumR<3))
				{
					putText(frame,"right&stop",Point(cars[i].x,cars[i].y),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),2);
				}
				else
					putText(frame,"stop",Point(cars[i].x,cars[i].y),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),2);
			}
			else{
				putText(frame,"stop",Point(cars[i].x,cars[i].y),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),2);
			}
		}
		if(islight[i].up == 1 && islight[i].left == 2 && islight[i].right == 1)
		{
			if(LightLR.size() == LightChangeNum )
			{
				int sumL,sumR;
				CalLightOn(i,sumL,sumR);
				//if((sumL>8 && sumL<12) ||(sumL<4))
				if((sumL<3))
				{
					putText(frame,"left&stop",Point(cars[i].x,cars[i].y),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),2);
				}
				else
					putText(frame,"stop",Point(cars[i].x,cars[i].y),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),2);
			}
			else{
				putText(frame,"stop",Point(cars[i].x,cars[i].y),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),2);
			}
		}
		if(islight[i].up == 0 && islight[i].left == 1 && islight[i].right == 1)
		{
			putText(frame,"position",Point(cars[i].x,cars[i].y),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,0),2);
		}
	}
}
/********************************************************
*将输入的Mask分成3部分分别检测灯亮的情况,若灯亮则返回1，反之0
*********************************************************/
void IsLightROI(vector<Mat>& vMask,vector<LightMessage>& out)
{
	int numberpix;
	for(auto iter = vMask.begin();iter != vMask.end();++iter)
	{
		Mat ROIup((*iter)(Rect(0.2*iter->cols,0,0.6*iter->cols,0.8*iter->rows))),
			ROIleft((*iter)(Rect(0,0.5*iter->rows,0.3*iter->cols,0.5*iter->rows))),
			ROIright((*iter)(Rect(0.6*iter->cols,0.5*iter->rows,0.4*iter->cols,0.5*iter->rows)));

		float TotalUp(ROIup.cols*ROIup.rows),
			TotalLeft(ROIleft.cols*ROIleft.rows),
			TotalRight(ROIright.cols*ROIright.rows);

		float CountUp(countNonZero(ROIup)),
			CountLeft(countNonZero(ROIleft)),
			CountRight(countNonZero(ROIright));
		cout<<"up = "<<CountUp<<endl;
		cout<<"left = "<<CountLeft<<endl;
		cout<<"right = "<<CountRight<<endl;

		float /*RatioUp(CountUp/TotalUp),*/
			RatioLeft(CountLeft/TotalLeft),
			RatioRight(CountRight/TotalRight);
		LightMessage message;
		//if(iter->cols <200)
		//{
		//	numberpix = 10;
		//}else if(iter->cols >= 200 && iter->cols < 400){
		//	numberpix = 20;
		//}else{
		//	numberpix = 30;
		//}
		if(CountUp > 5)
		{
			message.up = 1;
		}
		else{
			message.up = 0;
		}
		if(CountLeft > 30)
		{
			if(CountLeft - CountRight < 90)
			{
				message.left = 1;
			}
			else /*if(CountLeft > 150)*/{
				message.left = 2;
			}
		}
		else{
			message.left = 0;
		}
		if(CountRight > 30)
		{
			if(CountRight - CountLeft < 90)
			{
				message.right = 1;
			}
			else /*if(CountRight > 150)*/{
				message.right = 2;
			}
		}
		else{
			message.right = 0;
		}
		out.push_back(message);
		cout<<"numR - numL = "<<CountRight - CountLeft<<endl;
	}

}
/********************************************************
*统计尾灯在16帧内亮暗的情况
*********************************************************/
void CalLightOn(int num,int& Lminus,int& Rminus)
{
	int LsumON = 0,
		LsumOFF = 0,
		RsumON = 0,
		RsumOFF = 0;

	auto enditer = LightLR.rbegin();
	auto endHiter = OriHistRect.rbegin();
	auto iter = LightLR.rbegin() + 1;
	auto Hiter = OriHistRect.rbegin() + 1;

	while(iter != LightLR.rend() && Hiter != OriHistRect.rend())
	{
		for(int i = 0;i < Hiter->size();++i)
		{
			if(calOverlapRatio((*endHiter)[num],(*Hiter)[i]) > 0.8)
			{
				
				if((*iter)[i].left == 2)
				{
					LsumON++;
				}
				if((*iter)[i].left != 2)
				{
					LsumOFF++;
				}
				if((*iter)[i].right == 2)
				{
					RsumON++;
				}
				if((*iter)[i].right != 2)
				{
					RsumOFF++;
				}
			}
		}
		++iter;
		++Hiter;
	}
	if(LsumON > 4 && LsumOFF > 4)
	{
		Lminus = LsumON - LsumOFF;
	}
	if(RsumON > 4 && RsumOFF > 4)
	{
		Rminus = RsumON - RsumOFF;
	}
	else{
		Lminus = 10;
		Rminus = 10;
	}
	//cout<<"L = "<<Lminus<<endl;
	cout<<"R = "<<Rminus<<endl;
}
