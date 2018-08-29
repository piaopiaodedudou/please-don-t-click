#include"quchuxuanguang.h"
#include"TideRect.h"

/*The interface consists of one simple function guidedFilter and a class GuidedFilter. 
If you have multiple images to filter with the same guidance image then use GuidedFilter class to avoid extra computations on initialization stage. 
The code supports single-channel and 3-channel (color) guidance images and CV_8U, CV_8S, CV_16U, CV_16S, CV_32S, CV_32F and CV_64F data types*/

vector<vector<vector<RotatedRect>>> Hist;
static cv::Mat boxfilter(const cv::Mat &I, int r)
{
    cv::Mat result;
    cv::blur(I, result, cv::Size(r, r));
    return result;
}

static cv::Mat convertTo(const cv::Mat &mat, int depth)
{
    if (mat.depth() == depth)
        return mat;

    cv::Mat result;
    mat.convertTo(result, depth);
    return result;
}

class GuidedFilterImpl
{
public:
    virtual ~GuidedFilterImpl() {}

    cv::Mat filter(const cv::Mat &p, int depth);

protected:
    int Idepth;

private:
    virtual cv::Mat filterSingleChannel(const cv::Mat &p) const = 0;
};

class GuidedFilterMono : public GuidedFilterImpl
{
public:
    GuidedFilterMono(const cv::Mat &I, int r, double eps);

private:
    virtual cv::Mat filterSingleChannel(const cv::Mat &p) const;

private:
    int r;
    double eps;
    cv::Mat I, mean_I, var_I;
};

class GuidedFilterColor : public GuidedFilterImpl
{
public:
    GuidedFilterColor(const cv::Mat &I, int r, double eps);

private:
    virtual cv::Mat filterSingleChannel(const cv::Mat &p) const;

private:
    std::vector<cv::Mat> Ichannels;
    int r;
    double eps;
    cv::Mat mean_I_r, mean_I_g, mean_I_b;
    cv::Mat invrr, invrg, invrb, invgg, invgb, invbb;
};


cv::Mat GuidedFilterImpl::filter(const cv::Mat &p, int depth)
{
    cv::Mat p2 = convertTo(p, Idepth);

    cv::Mat result;
    if (p.channels() == 1)
    {
        result = filterSingleChannel(p2);
    }
    else
    {
        std::vector<cv::Mat> pc;
        cv::split(p2, pc);

        for (std::size_t i = 0; i < pc.size(); ++i)
            pc[i] = filterSingleChannel(pc[i]);

        cv::merge(pc, result);
    }

    return convertTo(result, depth == -1 ? p.depth() : depth);
}

GuidedFilterMono::GuidedFilterMono(const cv::Mat &origI, int r, double eps) : r(r), eps(eps)
{
    if (origI.depth() == CV_32F || origI.depth() == CV_64F)
        I = origI.clone();
    else
        I = convertTo(origI, CV_32F);

    Idepth = I.depth();

    mean_I = boxfilter(I, r);
    cv::Mat mean_II = boxfilter(I.mul(I), r);
    var_I = mean_II - mean_I.mul(mean_I);
}

cv::Mat GuidedFilterMono::filterSingleChannel(const cv::Mat &p) const
{
    cv::Mat mean_p = boxfilter(p, r);
    cv::Mat mean_Ip = boxfilter(I.mul(p), r);
    cv::Mat cov_Ip = mean_Ip - mean_I.mul(mean_p); // this is the covariance of (I, p) in each local patch.

    cv::Mat a = cov_Ip / (var_I + eps); // Eqn. (5) in the paper;
    cv::Mat b = mean_p - a.mul(mean_I); // Eqn. (6) in the paper;

    cv::Mat mean_a = boxfilter(a, r);
    cv::Mat mean_b = boxfilter(b, r);

    return mean_a.mul(I) + mean_b;
}

GuidedFilterColor::GuidedFilterColor(const cv::Mat &origI, int r, double eps) : r(r), eps(eps)
{
    cv::Mat I;
    if (origI.depth() == CV_32F || origI.depth() == CV_64F)
        I = origI.clone();
    else
        I = convertTo(origI, CV_32F);

    Idepth = I.depth();

    cv::split(I, Ichannels);

    mean_I_r = boxfilter(Ichannels[0], r);
    mean_I_g = boxfilter(Ichannels[1], r);
    mean_I_b = boxfilter(Ichannels[2], r);

    // variance of I in each local patch: the matrix Sigma in Eqn (14).
    // Note the variance in each local patch is a 3x3 symmetric matrix:
    //           rr, rg, rb
    //   Sigma = rg, gg, gb
    //           rb, gb, bb
    cv::Mat var_I_rr = boxfilter(Ichannels[0].mul(Ichannels[0]), r) - mean_I_r.mul(mean_I_r) + eps;
    cv::Mat var_I_rg = boxfilter(Ichannels[0].mul(Ichannels[1]), r) - mean_I_r.mul(mean_I_g);
    cv::Mat var_I_rb = boxfilter(Ichannels[0].mul(Ichannels[2]), r) - mean_I_r.mul(mean_I_b);
    cv::Mat var_I_gg = boxfilter(Ichannels[1].mul(Ichannels[1]), r) - mean_I_g.mul(mean_I_g) + eps;
    cv::Mat var_I_gb = boxfilter(Ichannels[1].mul(Ichannels[2]), r) - mean_I_g.mul(mean_I_b);
    cv::Mat var_I_bb = boxfilter(Ichannels[2].mul(Ichannels[2]), r) - mean_I_b.mul(mean_I_b) + eps;

    // Inverse of Sigma + eps * I
    invrr = var_I_gg.mul(var_I_bb) - var_I_gb.mul(var_I_gb);
    invrg = var_I_gb.mul(var_I_rb) - var_I_rg.mul(var_I_bb);
    invrb = var_I_rg.mul(var_I_gb) - var_I_gg.mul(var_I_rb);
    invgg = var_I_rr.mul(var_I_bb) - var_I_rb.mul(var_I_rb);
    invgb = var_I_rb.mul(var_I_rg) - var_I_rr.mul(var_I_gb);
    invbb = var_I_rr.mul(var_I_gg) - var_I_rg.mul(var_I_rg);

    cv::Mat covDet = invrr.mul(var_I_rr) + invrg.mul(var_I_rg) + invrb.mul(var_I_rb);

    invrr /= covDet;
    invrg /= covDet;
    invrb /= covDet;
    invgg /= covDet;
    invgb /= covDet;
    invbb /= covDet;
}

cv::Mat GuidedFilterColor::filterSingleChannel(const cv::Mat &p) const
{
    cv::Mat mean_p = boxfilter(p, r);

    cv::Mat mean_Ip_r = boxfilter(Ichannels[0].mul(p), r);
    cv::Mat mean_Ip_g = boxfilter(Ichannels[1].mul(p), r);
    cv::Mat mean_Ip_b = boxfilter(Ichannels[2].mul(p), r);

    // covariance of (I, p) in each local patch.
    cv::Mat cov_Ip_r = mean_Ip_r - mean_I_r.mul(mean_p);
    cv::Mat cov_Ip_g = mean_Ip_g - mean_I_g.mul(mean_p);
    cv::Mat cov_Ip_b = mean_Ip_b - mean_I_b.mul(mean_p);

    cv::Mat a_r = invrr.mul(cov_Ip_r) + invrg.mul(cov_Ip_g) + invrb.mul(cov_Ip_b);
    cv::Mat a_g = invrg.mul(cov_Ip_r) + invgg.mul(cov_Ip_g) + invgb.mul(cov_Ip_b);
    cv::Mat a_b = invrb.mul(cov_Ip_r) + invgb.mul(cov_Ip_g) + invbb.mul(cov_Ip_b);

    cv::Mat b = mean_p - a_r.mul(mean_I_r) - a_g.mul(mean_I_g) - a_b.mul(mean_I_b); // Eqn. (15) in the paper;

    return (boxfilter(a_r, r).mul(Ichannels[0])
          + boxfilter(a_g, r).mul(Ichannels[1])
          + boxfilter(a_b, r).mul(Ichannels[2])
          + boxfilter(b, r));  // Eqn. (16) in the paper;
}

GuidedFilter::GuidedFilter(const cv::Mat &I, int r, double eps)
{
    CV_Assert(I.channels() == 1 || I.channels() == 3);

    if (I.channels() == 1)
        impl_ = new GuidedFilterMono(I, 2 * r + 1, eps);
    else
        impl_ = new GuidedFilterColor(I, 2 * r + 1, eps);
}

GuidedFilter::~GuidedFilter()
{
    delete impl_;
}

cv::Mat GuidedFilter::filter(const cv::Mat &p, int depth) const
{
    return impl_->filter(p, depth);
}

cv::Mat guidedFilter(const cv::Mat &I, const cv::Mat &p, int r, double eps, int depth)
{
    return GuidedFilter(I, r, eps).filter(p, depth);
}

Mat guidedFilter2(cv::Mat I, cv::Mat p, int r, double eps)  
{  
  /* 
  % GUIDEDFILTER   O(1) time implementation of guided filter. 
  % 
  %   - guidance image: I (should be a gray-scale/single channel image) 
  %   - filtering input image: p (should be a gray-scale/single channel image) 
  %   - local window radius: r 
  %   - regularization parameter: eps 
  */  
   
	if(I.rows != p.rows || I.cols != p.cols ){ cout<<"wrong"<<endl;return I;}
  cv::Mat _I;  
  I.convertTo(_I, CV_64FC1);  
  I = _I;  
   
  cv::Mat _p;  
  p.convertTo(_p, CV_64FC1);  
  p = _p;  
   
  //[hei, wid] = size(I);  
  int hei = I.rows;  
  int wid = I.cols;  
   
  //N = boxfilter(ones(hei, wid), r); % the size of each local patch; N=(2r+1)^2 except for boundary pixels.  
  cv::Mat N;  
  cv::boxFilter(cv::Mat::ones(hei, wid, I.type()), N, CV_64FC1, cv::Size(r, r));  
   
  //mean_I = boxfilter(I, r) ./ N;  
  cv::Mat mean_I;  
  cv::boxFilter(I, mean_I, CV_64FC1, cv::Size(r, r));  
    
  //mean_p = boxfilter(p, r) ./ N;  
  cv::Mat mean_p;  
  cv::boxFilter(p, mean_p, CV_64FC1, cv::Size(r, r));  
   
  //mean_Ip = boxfilter(I.*p, r) ./ N;  
  I.mul(p);
  cv::Mat mean_Ip;  
  cv::boxFilter(I.mul(p), mean_Ip, CV_64FC1, cv::Size(r, r));  
   
  //cov_Ip = mean_Ip - mean_I .* mean_p; % this is the covariance of (I, p) in each local patch.  
  cv::Mat cov_Ip = mean_Ip - mean_I.mul(mean_p);  
   
  //mean_II = boxfilter(I.*I, r) ./ N;  
  cv::Mat mean_II;  
  cv::boxFilter(I.mul(I), mean_II, CV_64FC1, cv::Size(r, r));  
   
  //var_I = mean_II - mean_I .* mean_I;  
  cv::Mat var_I = mean_II - mean_I.mul(mean_I);  
   
  //a = cov_Ip ./ (var_I + eps); % Eqn. (5) in the paper;     
  cv::Mat a = cov_Ip/(var_I + eps);  
   
  //b = mean_p - a .* mean_I; % Eqn. (6) in the paper;  
  cv::Mat b = mean_p - a.mul(mean_I);  
   
  //mean_a = boxfilter(a, r) ./ N;  
  cv::Mat mean_a;  
  cv::boxFilter(a, mean_a, CV_64FC1, cv::Size(r, r));  
  mean_a = mean_a/N;  
   
  //mean_b = boxfilter(b, r) ./ N;  
  cv::Mat mean_b;  
  cv::boxFilter(b, mean_b, CV_64FC1, cv::Size(r, r));  
  mean_b = mean_b/N;  
   
  //q = mean_a .* I + mean_b; % Eqn. (8) in the paper;  
  cv::Mat q = mean_a.mul(I) + mean_b;  
   
  return q;  
}

cv::Mat fastGuidedFilter(cv::Mat I_org, cv::Mat p_org, int r, double eps, int s)  
{  
	/* 
	% GUIDEDFILTER   O(N) time implementation of guided filter. 
	% 
	%   - guidance image: I (should be a gray-scale/single channel image) 
	%   - filtering input image: p (should be a gray-scale/single channel image) 
	%   - local window radius: r 
	%   - regularization parameter: eps 
	*/  
	

	cv::Mat I,_I;  
	I_org.convertTo(_I, CV_64FC1/*, 1.0 / 255*/);  

	resize(_I,I,Size(),1.0/s,1.0/s,1);  


	      
	cv::Mat p,_p;  
	p_org.convertTo(_p, CV_64FC1/*, 1.0 / 255*/);  
	//p = _p;  
	resize(_p, p, Size(),1.0/s,1.0/s,1);  

	//[hei, wid] = size(I);      
	int hei = I.rows;  
	int wid = I.cols;  

	r = (22 * r + 1)/s+1;//因为opencv自带的boxFilter（）中的Size,比如9x9,我们说半径为4     

	//mean_I = boxfilter(I, r) ./ N;      
	cv::Mat mean_I;  
	cv::boxFilter(I, mean_I, CV_64FC1, cv::Size(r, r));  

	//mean_p = boxfilter(p, r) ./ N;      
	cv::Mat mean_p;  
	cv::boxFilter(p, mean_p, CV_64FC1, cv::Size(r, r));  

	//mean_Ip = boxfilter(I.*p, r) ./ N;      
	cv::Mat mean_Ip;  
	cv::boxFilter(I.mul(p), mean_Ip, CV_64FC1, cv::Size(r, r));  

	//cov_Ip = mean_Ip - mean_I .* mean_p; % this is the covariance of (I, p) in each local patch.      
	cv::Mat cov_Ip = mean_Ip - mean_I.mul(mean_p);  

	//mean_II = boxfilter(I.*I, r) ./ N;      
	cv::Mat mean_II;  
	cv::boxFilter(I.mul(I), mean_II, CV_64FC1, cv::Size(r, r));  

	//var_I = mean_II - mean_I .* mean_I;      
	cv::Mat var_I = mean_II - mean_I.mul(mean_I);  

	//a = cov_Ip ./ (var_I + eps); % Eqn. (5) in the paper;         
	cv::Mat a = cov_Ip / (var_I + eps);  

	//b = mean_p - a .* mean_I; % Eqn. (6) in the paper;      
	cv::Mat b = mean_p - a.mul(mean_I);  

	//mean_a = boxfilter(a, r) ./ N;      
	cv::Mat mean_a;  
	cv::boxFilter(a, mean_a, CV_64FC1, cv::Size(r, r));  
	Mat rmean_a;  
	resize(mean_a, rmean_a, Size(I_org.cols, I_org.rows),1);  

	//mean_b = boxfilter(b, r) ./ N;      
	cv::Mat mean_b;  
	cv::boxFilter(b, mean_b, CV_64FC1, cv::Size(r, r));  
	Mat rmean_b;  
	resize(mean_b, rmean_b, Size(I_org.cols, I_org.rows),1);  

	//q = mean_a .* I + mean_b; % Eqn. (8) in the paper;      
	cv::Mat q = rmean_a.mul(_I) + rmean_b;  

	return q;  
}  

void CalRatio(vector<Mat>& vsrc,Mat& Rratio)
{
	Mat minussrc, minsrc, maxsrc,R;
	normalize(vsrc[0],vsrc[0],1,0);
	normalize(vsrc[1],vsrc[1],1,0);
	normalize(vsrc[2],vsrc[2],1,0);
    min(vsrc[0],vsrc[1],minsrc);
	min(vsrc[2],minsrc,minsrc);

	absdiff(vsrc[2],minsrc,R);

	max(vsrc[0],vsrc[1],maxsrc);
    max(vsrc[2],maxsrc,maxsrc);

	absdiff(maxsrc,minsrc,minussrc);
	
	divide(R,(minussrc + 0.00001),Rratio);
	
	threshold(Rratio,Rratio,0.998,255,0);

	//Rratio.convertTo(Rratio,CV_8UC1);//寻找边界
	vector<vector<Point>> output;
	vector<Vec4i> h;
	Mat img2 = Mat::zeros(Rratio.rows,Rratio.cols,CV_8UC1);



	Mat element = getStructuringElement( MORPH_RECT,
                                       Size(2,2));

	//morphologyEx(Rratio,Rratio,MORPH_CLOSE,element);
	//morphologyEx(Rratio,Rratio,MORPH_OPEN,element);//误检相对少些，但是目标变得比较碎
	erode(Rratio,Rratio,element);                 //使用较大的核进行膨胀与进行两次膨胀相比找不出大差别（5,5）（3,3）
	Mat element1 = getStructuringElement( MORPH_RECT,
                                       Size(4,4));
	dilate(Rratio, Rratio, element1);
	//imshow("Rratio",Rratio);
}

void MultiShow(string str,int hangshu,int lieshu,Mat img1,...)
{
	int col = img1.cols;
	int row = img1.rows;
	Mat out(row*hangshu,col*lieshu,img1.type());
	va_list ap;
	va_start(ap,lieshu);

	for(int i = 0;i < hangshu;++i)
	{
		for(int j = 0;j < lieshu;++j)
		{
			Mat temp;
			temp = va_arg(ap,Mat);
			temp.copyTo(out(Rect(j*col,i*row,col,row)));
		}
	}
	va_end(ap);
	imshow(str,out);
}    
void DarkChannelFn(Mat& src, Mat& result)
{
	//获得暗通道图像
	//src.convertTo(src,CV_32FC3,1.0/255);
	resize(src,src,Size(src.cols*0.5,src.rows*0.5));
	vector<Mat> vsrc;
	Mat darkImg, t, minblur, Mimg, t1, darkImgf;
	double A;
	split(src, vsrc);
	darkImg = min(vsrc[0], min(vsrc[1], vsrc[2]));
	//imshow("darkimg", darkImg);
	//对暗通道图像进行最小值滤波
	//MinFilter(darkImg, kersize, minblur);

	GetA(vsrc, darkImg, A);
	GetT(darkImg, A, t);//T有问题

	vsrc[0].convertTo(vsrc[0], CV_32F, 1.0/255);
	vsrc[1].convertTo(vsrc[1], CV_32F, 1.0/255);
	vsrc[2].convertTo(vsrc[2], CV_32F, 1.0/255);

	vsrc[0] = ((vsrc[0] - A)/(max(t, 0.1))) + A;
	vsrc[1] = ((vsrc[1] - A)/(max(t, 0.1))) + A;
	vsrc[2] = ((vsrc[2] - A)/(max(t, 0.1))) + A;	


	//Mimg = vsrc[2] - min(vsrc[0],vsrc[1]);

	//Mimg.convertTo(Mimg,CV_8U,255);


	//GetTR(Mimg, 1, t1);

	//vsrc[2] = ((vsrc[2] - 1)/(max(t1,0.1))) + 1;

	Mat Rratio;
	vector<RotatedRect> vpt;
	vector<vector<RotatedRect>> vvpt;
	vector<Mat> cvsrc;
	vector<Rect> vrect;
	for(auto iter = vsrc.begin();iter != vsrc.end();++iter)
	{
		Mat temp(iter->clone());
		cvsrc.push_back(temp);
	}

	CalRatio(cvsrc,Rratio);
	//imshow("Rratio",Rratio);
	Mat ROI = Rratio(Range(cvRound(cutlim*Rratio.rows), Rratio.rows), Range(0, Rratio.cols));
	findelli(ROI,vpt);
	matchlights(vpt,vvpt);
	combineelli(vvpt);
	AccurateLights(vvpt);
	findtoplight(vvpt);
	for(auto iter = vvpt.begin(); iter != vvpt.end(); ++iter)
	{
		for(auto it = iter->begin(); it != iter->end(); ++it)
		{
			it->center.y = it->center.y + cutlim*Rratio.rows;
		}
	}
	MakeRect(vvpt, vrect);
	//auto rrit = vrect.begin();
	//while(rrit != vrect.end())
	//{
	//	if(rrit->width > 200)
	//	{
	//		rrit = vrect.erase(rrit);
	//	}
	//	else
	//	{
	//		++rrit;
	//	}
	//}
	//merge(vsrc,result);
	
	for(auto iter = vvpt.begin(); iter != vvpt.end();++iter)
	{

		if(iter->size() == 1 && iter->begin()->size.area() > 2000)
		{
			Mimg = vsrc[2] - min(vsrc[0],vsrc[1]);

			Mimg.convertTo(Mimg,CV_8U,255);


			GetTR(Mimg, 1, t1);

			vsrc[2] = ((vsrc[2] - 1)/(max(t1,0.1))) + 1;
			CalRatio(vsrc,Rratio);
			//imshow("Rratioin",Rratio);
			vpt.clear();
			findelli(Rratio,vpt);
			
			vvpt.clear();
			matchlights(vpt,vvpt);
			combineelli(vvpt);//没看到啥效果，暂时留着吧
			AccurateLights(vvpt);
			findtoplight(vvpt);
			vrect.clear();
			MakeRect(vvpt, vrect);
			//auto rit = vrect.begin();
			//while(rit != vrect.end())
			//{
			//	if(rit->width > 200)
			//	{
			//		rit = vrect.erase(rit);
			//	}
			//	else
			//	{
			//		++rit;
			//	}
			//}
			break;
		}
	}

	//for(auto iter = vrect.begin();iter != vrect.end();++iter)
	//{
	//	rectangle(src, *iter, Scalar(0,255,0),1); 
	//}

    detectcars(vrect);
	//DeleteVert(vrect);
	vector<vector<Point>> showresult;
	for(auto iter = vvpt.begin(); iter != vvpt.end(); ++iter)
	{
		vector<Point> temppt;
		for(auto it = iter->begin(); it != iter->end(); ++it)
		{
			ellipse(src, *it, Scalar(0, 0, 255),1);
			temppt.push_back(it->center);
		}
		showresult.push_back(temppt);
		temppt.clear();
	}
	for(auto iter = showresult.begin();iter != showresult.end();++iter)
	{
		if(iter->size() > 1)
		{
			auto it = iter->begin();
			while(it != iter->end())
			{
				auto nit = it + 1;
				while(nit != iter->end())
				{
					line(src, *it, *nit, Scalar(0,0,255),2);
					++nit;
				}
				++it;
			}
		}
	}
	for(auto iter = vrect.begin();iter != vrect.end();++iter)
	{
		rectangle(src, *iter, Scalar(255,255,255),2);
	}

	imshow("after",src);
	//imwrite("误检1.jpg",src);
}
void MinFilter(Mat& src, int kersize, Mat& DarkImg)
{
	//阅读文献D:\360安全浏览器下载\O(1)最大值最小值算法，并实现
	//先用opencv自带的最大值最小值函数看效果
    //[1] --检测原始图像
    if(src.channels()!=1)
        return;
    if(src.depth()>8)
        return;
    //[1]

    //int r=(ksize-1)/2; //核半径
    //初始化目标图像
    DarkImg = Mat::zeros(src.rows,src.cols,CV_8UC1);
    //[3] --最小值滤波
    for(int i=0;i<src.rows;i++)
        for(int j=0;j<src.cols;j++)
        {
            //[1] --初始化滤波核的上下左右边界
            int top = i - kersize;
            int bottom = i + kersize;
            int left = j - kersize;
            int right = j + kersize;
            //[1]

            //[2] --检查滤波核是否超出边界
            if(i - kersize < 0)
                top = 0;
            if(i + kersize > src.rows)
                bottom = src.rows;
            if(j - kersize < 0)
                left = 0;
            if(j + kersize > src.cols)
                right = src.cols;
            //[2]

            //[3] --求取模板下的最小值
            Mat ImROI = src(Range(top,bottom),Range(left,right));
            double minval,maxval;
            minMaxLoc(ImROI, &minval, &maxval, 0, 0);
			DarkImg.at<uchar>(i,j) = minval;
            //[3]
        }
    //[3]
}
void GetA(vector<Mat> vsrc,Mat& darkimg, double& A)
{
	Mat Darkimg,Mask;
	darkimg.copyTo(Darkimg);

	cv::sort(Darkimg,Darkimg,CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING);
	cv::sort(Darkimg,Darkimg,CV_SORT_EVERY_ROW + CV_SORT_DESCENDING);
	//cout<<"sort = "<<Darkimg(Range(0,30),Range(0,30))<<endl;
	double limtpercent = darkimg.cols*darkimg.rows*0.001;
	//cout<<"limper = "<<limtpercent<<endl;
	int numcol = std::sqrt(limtpercent);
	//cout<<"numcol = "<<numcol<<endl;
	Scalar limpix = mean(Darkimg(Rect(0,0,numcol,numcol)));
	//cout<<"lmpix = "<<limpix<<endl;
	//imshow("putin",darkimg);
	threshold(darkimg,Mask,limpix[0],1.0,THRESH_BINARY);
	//imshow("Mask",Mask*255);
	double minvalue,maxvalue;
	Mat MaskB = vsrc[0].mul(Mask);
	Mat MaskG = vsrc[1].mul(Mask);
	Mat MaskR = vsrc[2].mul(Mask);
	Mat lightvalue = (MaskB + MaskG + MaskR)/3;
	minMaxLoc(lightvalue,&minvalue,&maxvalue);
	float num = countNonZero(lightvalue);
	Scalar sumpix = sum(lightvalue);
	A = sumpix[0]/num/maxvalue;
	//cout<<"a"<<A<<endl;
	if(A > 220.0)
	{
		A = 220.0;
	}
}
void GetT(Mat& darkimg, double A, Mat& t)
{
	Mat resizeImg;
	//darkimg.copyTo(resizeImg);
	resize(darkimg,resizeImg,Size(darkimg.cols*0.5,darkimg.rows*0.5));
	resizeImg.convertTo(resizeImg,CV_32F);

	t = 1 - /*0.9**/(resizeImg/A)/255.0;
	//Mat out, p(it);
	//int r = 15;
	//double eps = 0.0001;
	//eps *= 255 * 255;
	//out = guidedFilter(darkimg,p,r,eps);
	//out = guidedFilter2(t,p,r,eps);
	//out = fastGuidedFilter(t, p, r, eps, 2);
	//导向滤波
	//imshow("guide",out);
	resize(t,t,darkimg.size());
}

void GetTR(Mat& darkimg, double A, Mat& t)
{
	Mat resizeImg;
	//darkimg.copyTo(resizeImg);
	resize(darkimg,resizeImg,Size(darkimg.cols*0.5,darkimg.rows*0.5));
	resizeImg.convertTo(resizeImg,CV_32F);

	t = 1 - 0.9*(resizeImg/A)/255.0; // mimg
	//t = 1 - 0.22*(resizeImg/A)/255.0; //max- mimg

	//Mat out, p(it);
	//int r = 15;
	//double eps = 0.0001;
	//eps *= 255 * 255;
	//out = guidedFilter(darkimg,p,r,eps);
	//out = guidedFilter2(t,p,r,eps);
	//out = fastGuidedFilter(t, p, r, eps, 2);
	//导向滤波
	//imshow("guide",out);
	resize(t,t,darkimg.size());
}
//加上最小值滤波和导向滤波可以比较好的保留低亮度区域细节

void findelli(Mat& img,vector<RotatedRect>& vpt)
{
	//ofstream outfile1("Rratio.txt");
	//ofstream outfile2("img8UC1.txt");
	//outfile1<<img<<endl;
	img.convertTo(img,CV_8UC1);
	//outfile2<<img<<endl;
	vector<vector<Point>> output;
	vector<vector<Point>> out;
	vector<Vec4i> h;
	
	findContours(img, output, h, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	//combineContours(output, out);
	
	auto iter = output.begin();
	while(iter != output.end())
	{
		if(iter->size() < 30)   //如何准确对很小的框进行筛选？框太大怎进行滤波，框太小则删除
		{
			iter = output.erase(iter);
		}
		else
		{
			RotatedRect elli = fitEllipse(*iter); 
			//ellipse(show, elli, Scalar(255),1);
			vpt.push_back(elli);
			//circle(show, elli.center, 1, Scalar(255));
			//cout<<"area::"<<elli.size.area()<<endl;
			++iter;
		}
	}
	//drawContours(img, output, -1, Scalar(255), 1);
	//imshow("",img);
}

void rangePoints(vector<RotatedRect>& inputpt)
{
	auto iter = inputpt.begin();
	while(iter != inputpt.end() - 1)
	{
		auto nextiter = iter + 1;
		while(nextiter != inputpt.end())
		{
			if(iter->center.x > nextiter->center.x)
			{
				RotatedRect temppt;
				temppt = *iter;
				*iter = *nextiter;
				*nextiter = temppt;
			}
			++nextiter;
		}
		++iter;
	}
}

void matchlights(vector<RotatedRect>& points, vector<vector<RotatedRect>>& vcars)
{
	if(points.size() != 0)
	{	
		rangePoints(points);
	}
	auto iter = points.begin();
	while(iter != points.end())
	{
		vector<RotatedRect> vp;
		vp.push_back(*iter);
		auto nextiter = iter + 1;
		while(nextiter != points.end())
		{
			if(abs(iter->center.y - nextiter->center.y) < 0.2*min(iter->size.height, nextiter->size.height) //设置合适的 阈值是个问题 
				&& abs(iter->center.x - nextiter->center.x) < 350)
			{
				if((iter->size.area() < 3000 && nextiter->size.area() < 3000) 
					/*&& max(iter->size.area(), nextiter->size.area()) / min(iter->size.area(), nextiter->size.area()) < 3*/)
				{
					vp.push_back(*nextiter);
					nextiter = points.erase(nextiter);
				}
				else
				{
					++nextiter;
				}
			}
			else
				++nextiter;
		}
		vcars.push_back(vp);
		vp.clear();
		iter = points.erase(iter);
	}
}

void combineelli(vector<vector<RotatedRect>>& input)
{
	auto iter = input.begin();
	while(iter != input.end())
	{
		if(iter->size() > 1)
		{
			auto it = iter->begin();
			//while(it != iter->end())
			{
				auto nit = it + 1;
				while(nit != iter->end())
				{
					
					if(abs(it->center.x - nit->center.x) < max(it->size.width, nit->size.width))
					{
						RotatedRect temp;
						temp.center.x = 0.5*(it->center.x + nit->center.x);
						temp.center.y = 0.5*(it->center.y + nit->center.y);
						temp.size.height = max(it->size.height, nit->size.height);
						temp.size.width = it->size.width + nit->size.width;
						temp.angle = min(it->angle, nit->angle);
						
						it = iter->insert(it,temp);
						
						iter->erase(it + 1);
						
						nit = iter->erase(it + 1);
						
	
					}
					else
					{
						++it;
						++nit;
					}
				}
			}
		}
		++iter;
	}
}

void findtoplight(vector<vector<RotatedRect>>::iterator iter,vector<vector<RotatedRect>>& input)
{
	double centerdis = abs(iter->begin()->center.x - (iter->begin() + 1)->center.x);
	double centerx = (iter->begin()->center.x + (iter->begin() + 1)->center.x)/2;

	auto it = input.begin();
	while(it != input.end())
	{
		if(it != iter)
		{
			auto nit = it->begin();
			while(nit != it->end())
			{
				if((abs(nit->center.x - centerx) < 0.2*centerdis) && (nit->center.y < iter->begin()->center.y) 
					&&(abs(nit->center.y - iter->begin()->center.y) < centerdis))
				{
					iter->push_back(*nit);
					nit = it->erase(nit);
				}
				else
				{
					++nit;
				}
			}
			++it;
		}
		else
		{
			++it;
		}
	}
}

void AccurateLights(vector<vector<RotatedRect>>& input)
{
	auto viter = input.begin();
	while(viter != input.end())
	{
		if(viter->size() == 2) 
		{
			//if(viter != input.end())
			//{
			//	findtoplight(viter, input);
			//}
			++viter;
		}
		if(viter != input.end() && viter->size() > 2)
		{
			if(viter->size()%2 == 0)//暂且认为偶数个的时候全是车灯
			{
				vector<RotatedRect> temp;
				auto iter = viter->begin();
				while(iter != viter->end())
				{
					temp.push_back(*iter);
					iter = viter->erase(iter);
					temp.push_back(*iter);
					iter = viter->erase(iter);

					viter = input.insert(viter, temp);
					temp.clear();
					//if(viter != input.end())
					//{
					//	findtoplight(viter, input);
					//}
					++viter;
				}
				//--viter; //用于删除空向量,调用第一个if
			}
			if (viter->size()%2 == 1)
			{
				//通过面积
				
				auto iter = viter->begin();
				while(iter != viter->end())
				{
					vector<RotatedRect> temp;
					auto nextiter = iter + 1;
					while(nextiter != viter->end())
					{
						if(iter->size.area()/nextiter->size.area() > 0.8)
						{
							temp.push_back(*nextiter);   //删除一个元素后，元素后面的所有元素的迭代器全部失效
							nextiter = viter->erase(nextiter); //要倒着删除
							temp.push_back(*iter);
							iter = viter->erase(iter);
;
							break;
						}
						else
						{
							++nextiter;
						}
					}
					if(!temp.empty())
					{
						viter = input.insert(viter, temp);
						//if(viter != input.end())
						//{
						//	findtoplight(viter, input);
						//}
						++viter;
					}
					else
					{
						iter = viter->erase(iter);
					}
				}
			}

		}
		if(viter != input.end() && (viter->empty() /*|| viter->size() == 1*/))//判定条件有问题，如果是大面积呢？如果正确匹配顶灯只剩一个呢？
		{                                                                 //奇数包括一中会剩下的部分，顶灯匹配后剩下的部分如何处理比较合适？
			viter = input.erase(viter);
		}
		if(viter != input.end() && viter->size() == 1)
		{
			//if(viter->begin()->size.area() < 1000)
			//{
			//	viter = input.erase(viter);
			//}
			//else
			//{
				++viter;
			//}
		}
	}
}

void findtoplight(vector<vector<RotatedRect>>& input)
{
	auto iter = input.begin();
	while(iter != input.end())
	{
		if(iter->size() > 1)
		{
			double centerdis = abs(iter->begin()->center.x - (iter->begin() + 1)->center.x);
			double centerx = (iter->begin()->center.x + (iter->begin() + 1)->center.x)/2;

			auto it = input.begin();
			while(it != input.end())
			{
				if(it != iter)
				{
					auto nit = it->begin();
					while(nit != it->end())
					{
						if((abs(nit->center.x - centerx) < 0.2*centerdis) && (nit->center.y < iter->begin()->center.y) 
							&&(abs(nit->center.y - iter->begin()->center.y) < centerdis))
						{
							iter->push_back(*nit);
							nit = it->erase(nit);
						}
						else
						{
							++nit;
						}
					}
					++it;
				}
				else
				{
					++it;
				}
			}
			++iter;
		}
		else if(iter->empty())
		{
			iter = input.erase(iter);
		}
		else
		{
			++iter;
		}
	}
}

void combineContours(vector<vector<Point>>& output, vector<vector<Point>>& out)
{
	vector<Point> aveP;
	for(auto iter = output.begin();iter != output.end();++iter)
	{
		double sumx = 0;
		double sumy = 0;
		for(auto it = iter->begin();it != iter->end();++it)
		{
			sumx += it->x;
			sumy += it->y;
		}

		double avex = sumx/iter->size();
		double avey = sumy/iter->size();
		aveP.push_back(Point(avex, avey));
	}

	auto Piter = aveP.begin();
	auto iter = output.begin();
	while(Piter != aveP.end() - 1)
	{
		vector<Point> temp;
		auto nPiter = Piter + 1;
		auto niter = iter + 1;
		while(nPiter != aveP.end())
		{
			if(abs(Piter->x - nPiter->x ) + abs(Piter->y - nPiter->y) < 10)
			{
				temp.insert(temp.end(), iter->begin(), iter->end());
				temp.insert(temp.end(), niter->begin(), niter->end());
			}
			++niter;
			++nPiter;
		}
		if(!temp.empty())
		{
			out.push_back(temp);
		}
		else
		{
			out.push_back(*iter);
		}
		++iter;
		++Piter;
		temp.clear();
	}
}

void BiggerRect(Rect& rect, int xlim, int ylim)
{
	rect.x = (rect.x - 0.1*rect.width > 0) ? (rect.x - 0.1*rect.width):0;
	rect.y = (rect.y - 0.2*rect.width > 0) ? (rect.y - 0.2*rect.width):0;
	rect.width = (rect.x + 1.2*rect.width > xlim) ? xlim - rect.x:1.2*rect.width;
	rect.height = ylim - rect.y;
}

void MakeRect(vector<vector<RotatedRect>>& input, vector<Rect>& output)
{
	for(auto Riter = input.begin();Riter != input.end();++Riter)
	{
		if(Riter->size() > 1)
		{
			Rect rect;
			vector<Point2f> vp;
			Point2f Pfour[4];
			double minx, miny,maxx, maxy;
			for(auto iter = Riter->begin();iter != Riter->end();++iter)
			{
				Point2f Pfour[4];
				iter->points(Pfour);
				for(int i = 0;i < 4;++i)
				{
					vp.push_back(Pfour[i]);
				}
			}
			minx = vp.begin()->x;
			maxx = vp.begin()->x;
			miny = vp.begin()->y;
			maxy = vp.begin()->y;
			for(auto it = vp.begin() + 1;it != vp.end();++it)
			{
				if(minx > it->x)
				{
					minx = it->x;
				}
				if(miny > it->y)
				{
					miny = it->y;
				}
				if(maxx < it->x)
				{
					maxx = it->x;
				}
				if(maxy < it->y)
				{
					maxy = it->y;
				}
			}
			rect.x = minx;
			rect.y = miny;
			if(Riter->size() == 2)
			{
				rect.width = maxx - minx;
				rect.height = 2*(maxy - miny);
			}
			else
			{
				rect.width = maxx - minx;
				rect.height = maxy - miny;
			}
			output.push_back(rect);
		}
	}
}

void DeleteVert(vector<Rect>& input)
{
	if(input.size() > 1)
	{
		auto iter = input.begin();
		while(iter != input.end() - 1)
		{
			if(abs(iter->x - (iter + 1)->x) < 10 && abs(iter->x + iter->width - (iter + 1)->x - (iter + 1)->width) < 10)
			{
				(iter + 1)->x = min(iter->x, (iter + 1)->x);
				(iter + 1)->y = min(iter->y, (iter + 1)->y);
				(iter + 1)->width = max(iter->width, (iter + 1)->width);
				(iter + 1)->height = max(iter->height, (iter + 1)->width);
				iter = input.erase(iter);
			}
			else
				++iter;
		}
	}
}