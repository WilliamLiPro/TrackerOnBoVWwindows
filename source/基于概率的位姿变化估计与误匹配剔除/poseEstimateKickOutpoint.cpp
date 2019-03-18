/***************************************************************
	>程序：基于概率的位姿变化估计与外点剔除
	>作者：李维鹏
	>联系方式：248636779@163.com
	
	>技术要点：
	>1.概率权值最小二乘求解位姿变化
	>2.基于标准差的外点剔除

****************************************************************/
#include "stdafx.h"
#include "poseEstimateKickOutpoint.h"

/*	概率权值最小二乘求解位姿变化
	采用牛顿法求解最小二乘

	输入：
	原始点坐标序列		vector<cv::Point2f>& origin_p		
	匹配点坐标序列		vector<cv::Point2f>& match_p
	匹配权重			vector<float>& weight_p

	输出：
	位姿变化			cv::Mat& pose_cg
	*/
bool weightBaedPoseEstimate(vector<cv::Point2f>& origin_p,vector<cv::Point2f>& match_p,vector<float>& weight_p,cv::Mat& pose_cg)
{
	//1.初始化计算参数
	int point_n=min(origin_p.size(),match_p.size());	//匹配特征对数

	if(point_n<3)
	{
		cout<<"warning: 匹配特征过少，无法计算位姿变化"<<endl;
		return false;
	}

	//最小二乘求解参数准备
	if(pose_cg.rows!=5)
	{
		pose_cg=cv::Mat::zeros(5,1,CV_32FC1);		//待求解参数矩阵初始化(wx,wy,wz,dx,dy)
	}

	cv::Mat fun_input(point_n,3,CV_32FC1);				//函数输入参数矩阵
	cv::Mat fun_out(point_n,2,CV_32FC1);				//函数输出参数矩阵

	float *fun_input_ptr,*fun_out_ptr;
	int data_id=0;	//方程数据组数

	for(int i=0;i<point_n;i++)
	{
		fun_input_ptr=fun_input.ptr<float>(i);
		fun_out_ptr=fun_out.ptr<float>(i);

		float wp=sqrt(weight_p[i]);

		fun_input_ptr[0]=origin_p[i].x;
		fun_input_ptr[1]=origin_p[i].y;
		fun_input_ptr[2]=wp;

		fun_out_ptr[0]=(match_p[i].x-origin_p[i].x)*wp;
		fun_out_ptr[1]=(match_p[i].y-origin_p[i].y)*wp;
	}

	//2.定义计算函数，NewTon法迭代计算
	//计算退出条件
	float ex_cond=0;
	for(int i=0;i<fun_input.rows;i++)
	{
		ex_cond+=weight_p[i];
	}
	ex_cond*=0.0001;

	//计算位姿变化
	vector<void (*)(cv::Mat&,cv::Mat&,float&)> pfuns(2);
	pfuns[0]=&matchPoseChangeX;
	pfuns[1]=&matchPoseChangeY;

	aLeastSquareNewtonSolver(pfuns,0.0001,ex_cond,pose_cg,fun_input,fun_out);

	//cout<<pose_cg<<endl;

	return true;
}

/*	概率权值最小二乘求解图像旋转平移变化
	采用解析法求解最小二乘

	输入：
	原始点坐标序列		vector<cv::Point2f>& origin_p		
	匹配点坐标序列		vector<cv::Point2f>& match_p
	匹配权重			vector<float>& weight_p

	输出：
	图像变化			cv::Mat& pose_cg
	*/
bool weightBaedImageCgEstimate(vector<cv::Point2f>& origin_p,vector<cv::Point2f>& match_p,vector<float>& weight_p,cv::Mat& pose_cg)
{
	//1.初始化计算参数
	int point_n=min(origin_p.size(),match_p.size());	//匹配特征对数

	if(point_n<3)
	{
		cout<<"warning: 匹配特征过少，无法计算位姿变化"<<endl;
		return false;
	}

	//最小二乘求解参数准备
	if(pose_cg.rows!=4)
	{
		pose_cg=cv::Mat::zeros(4,1,CV_32FC1);		//待求解参数矩阵初始化(q,w,dx,dy)
	}


	//2.根据最小二乘偏导，求解线性方程组
	cv::Mat para_matrix(4,4,CV_32FC1);
	cv::Mat para_b(4,1,CV_32FC1);

	//计算系数矩阵para_matrix各个参数
	//a.计算第0行系数
	float sum_spp=0,sum_spwp=0,sum_spx=0,sum_spy=0;

	for(int i=0;i<point_n;i++)
	{
		sum_spp+=weight_p[i]*(origin_p[i].x*origin_p[i].x+origin_p[i].y*origin_p[i].y);
		sum_spx+=weight_p[i]*origin_p[i].x;
		sum_spy+=weight_p[i]*origin_p[i].y;
	}

	para_matrix.at<float>(0,0)=sum_spp;
	para_matrix.at<float>(0,1)=sum_spwp;
	para_matrix.at<float>(0,2)=sum_spx;
	para_matrix.at<float>(0,3)=sum_spy;

	//b.计算第1行系数(后两个)
	float sum_spwx=-sum_spy,sum_spwy=sum_spx;

	para_matrix.at<float>(1,0)=sum_spwp;
	para_matrix.at<float>(1,1)=sum_spp;
	para_matrix.at<float>(1,2)=sum_spwx;
	para_matrix.at<float>(1,3)=sum_spwy;

	//c.计算第2行系数
	float sum_s=0;

	for(int i=0;i<point_n;i++)
	{
		sum_s+=weight_p[i];
	}

	para_matrix.at<float>(2,0)=sum_spx;
	para_matrix.at<float>(2,1)=-sum_spy;
	para_matrix.at<float>(2,2)=sum_s;
	para_matrix.at<float>(2,3)=0;

	//d.计算第3行系数
	para_matrix.at<float>(3,0)=sum_spy;
	para_matrix.at<float>(3,1)=sum_spx;
	para_matrix.at<float>(3,2)=0;
	para_matrix.at<float>(3,3)=sum_s;

	//计算向量para_b各个参数
	float sum_spp2=0,sum_spwp2=0,sum_sp2x=0,sum_sp2y=0;

	for(int i=0;i<point_n;i++)
	{
		sum_spp2+=weight_p[i]*(origin_p[i].x*match_p[i].x+origin_p[i].y*match_p[i].y);
		sum_spwp2+=weight_p[i]*(origin_p[i].x*match_p[i].y-origin_p[i].y*match_p[i].x);
		sum_sp2x+=weight_p[i]*match_p[i].x;
		sum_sp2y+=weight_p[i]*match_p[i].y;
	}
	para_b.at<float>(0,0)=sum_spp2;
	para_b.at<float>(1,0)=sum_spwp2;
	para_b.at<float>(2,0)=sum_sp2x;
	para_b.at<float>(3,0)=sum_sp2y;

	//3.求解方程
	cv::solve(para_matrix,para_b,pose_cg);

	//特殊情况下，输出尺寸变化为0，需要校正为1
	if(pose_cg.at<float>(0)<0.7||pose_cg.at<float>(0)>1.4)
	{
		pose_cg=cv::Mat::zeros(pose_cg.rows,pose_cg.cols,CV_32FC1);
		pose_cg.at<float>(0)=1;
	}

	return true;
}


/*	基于标准差的外点剔除

	输入：
	位姿变化			cv::Mat& pose_cg
	标准差倍数阈值		float std_th
	原始点坐标序列		vector<cv::Point2f>& origin_p		
	匹配点坐标序列		vector<cv::Point2f>& match_p

	输出：
	各点距离			vector<float>& d_points
	内点标记			vector<bool>& inline_p
	*/
bool kickOutpoint(cv::Mat& pose_cg,float std_th,vector<cv::Point2f>& origin_p,vector<cv::Point2f>& match_p,vector<float>& d_points,vector<bool>& inline_p)
{
	//1.初始化计算参数
	int point_n=min(origin_p.size(),match_p.size());	//匹配特征对数

	if(max(pose_cg.rows,pose_cg.cols)<4)
	{
		cout<<"error: in greateProPoseEstimate -> 输入位姿变化有误,参数个数不足"<<endl;
		return false;
	}

	if(point_n<=1)
	{
		cout<<"error: in greateProPoseEstimate -> 匹配点对过少,无法剔除外点"<<endl;
		return false;
	}


	float* pose_cg_ptr=pose_cg.ptr<float>(0);

	//2.统计位移误差
	vector<cv::Point2f> delta_pcg(point_n);		//每对匹配点位置变化误差
	cv::Point2f aver_pcg;
	aver_pcg.x=0;
	aver_pcg.y=0;

	float delta_x,delta_y; 

	for(int i=0;i<point_n;i++)
	{
		cv::Point2f& orig_p=origin_p[i];
		cv::Point2f& new_p=match_p[i];

		delta_x=(orig_p.x*pose_cg_ptr[0]-orig_p.y*pose_cg_ptr[1]+pose_cg_ptr[2])-new_p.x;
		delta_y=(orig_p.x*pose_cg_ptr[1]+orig_p.y*pose_cg_ptr[0]+pose_cg_ptr[3])-new_p.y;

		delta_pcg[i].x=delta_x;
		delta_pcg[i].y=delta_y;

		aver_pcg=aver_pcg+delta_pcg[i];
	}
	if(point_n>0)
	{
		aver_pcg.x=aver_pcg.x/point_n;
		aver_pcg.y=aver_pcg.y/point_n;
	}

	//3.计算估计位移误差的标准差
	d_points=vector<float>(point_n);
	vector<float>dx_points(point_n);
	vector<float>dy_points(point_n);

	float std_dx=0;
	float std_dy=0;

	for(int i=0;i<point_n;i++)
	{
		float error_pcg_x=delta_pcg[i].x-aver_pcg.x;
		float error_pcg_y=delta_pcg[i].y-aver_pcg.y;
		float error_pcg_x2=error_pcg_x*error_pcg_x;
		float error_pcg_y2=error_pcg_y*error_pcg_y;
		float square_d=error_pcg_x2+error_pcg_y2;

		d_points[i]=sqrt(square_d);
		dx_points[i]=abs(error_pcg_x);
		dy_points[i]=abs(error_pcg_y);

		std_dx+=error_pcg_x2;
		std_dy+=error_pcg_y2;
	}
	if(point_n>0)
	{
		std_dx=sqrt(std_dx/point_n);
		std_dy=sqrt(std_dy/point_n);
	}

	//4.剔除阈值外的点
	inline_p=vector<bool>(point_n,true);

	float threshold_x=std_th*std_dx;
	float threshold_y=std_th*std_dy;
	
	for(int i=0;i<point_n;i++)
	{
		if(dx_points[i]>threshold_x||dy_points[i]>threshold_y)		//外点判断条件
		{
			inline_p[i]=false;
		}
	}

	return true;
}


/*	点的坐标归一化
	已知像素坐标，由相机基本参数得到归一化的点坐标

	输入：
	像素坐标序列		vector<cv::Point2f>& pix_p
	相机参数			Camera_Para& cam_para

	输出：
	归一化坐标序列		vector<cv::Point2f>& cam_p
*/
bool pixCoordtoCameraCoord(vector<cv::Point2f>& pix_p,Camera_Para& cam_para,vector<cv::Point2f>& cam_p)
{
	//0.数据准备
	int point_n=pix_p.size();	//点的个数

	float cx=cam_para.cx;
	float cy=cam_para.cy;

	float fx=cam_para.fx;
	float fy=cam_para.fy;

	if(cx<=0||cy<=0||fx<=0||fy<=0)
	{
		cout<<"error: in pixCoordtoCameraCoord -> 输入相机参数必须大于0"<<endl;
		return false;
	}

	//1.坐标转化
	cam_p.resize(point_n);

	for(int i=0;i<point_n;i++)
	{
		cam_p[i].x=(pix_p[i].x-cx)/fx;
		cam_p[i].y=(pix_p[i].y-cy)/fy;
	}

	return true;
}


/*	经过旋转平移后的特征点位置变化
	输入：
	函数输入		cv::Mat& fun_in
	函数参数		cv::Mat& undt_para

	输出：
	函数输出		float& fun_out
*/
void matchPoseChangeX(cv::Mat& fun_in,cv::Mat& undt_para,float& fun_out)
{
	float* fun_in_ptr=fun_in.ptr<float>(0);
	float* undt_para_ptr=undt_para.ptr<float>(0);

	fun_out=(undt_para_ptr[1]-fun_in_ptr[1]*undt_para_ptr[2])+(fun_in_ptr[0]*undt_para_ptr[1]-fun_in_ptr[1]*undt_para_ptr[0])*undt_para_ptr[3];
	fun_out=fun_out*fun_in_ptr[2];
}

void matchPoseChangeY(cv::Mat& fun_in,cv::Mat& undt_para,float& fun_out)
{
	float* fun_in_ptr=fun_in.ptr<float>(0);
	float* undt_para_ptr=undt_para.ptr<float>(0);

	fun_out=(-undt_para_ptr[0]+fun_in_ptr[0]*undt_para_ptr[2])+(fun_in_ptr[0]*undt_para_ptr[1]-fun_in_ptr[1]*undt_para_ptr[0])*undt_para_ptr[4];
	fun_out=fun_out*fun_in_ptr[2];
}
