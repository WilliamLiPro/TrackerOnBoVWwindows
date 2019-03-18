/***************************************************************
	>类名：基于SURF特征跟踪的目标跟踪程序 (for windows版)
	>作者：李维鹏
	>联系方式：248636779@163.com
	>解决视觉里程计相邻帧的SUEF特征跟踪问题
	>技术要点：
	>1.SURF特征跟踪
	>2.基于SURF跟踪统计的目标跟踪

	>注意：该算法成功跟踪的基础是目标特征占据子图特征的大部分
****************************************************************/
#include "stdafx.h"
#include "TrackerOnBoVW.h"

/********************** public函数 *************************/

TrackerOnBoVW::TrackerOnBoVW(int sub_im_rows,int sub_im_cols,string feature_type,float track_th,int grid_size,bool compute_surf_angle)
{
	//1.特征跟踪器初始化
	feature_tracker_=BovwFeatureTracker::BovwFeatureTracker(sub_im_rows,sub_im_cols,feature_type);
	feature_tracker_.setTrackThre(track_th);
	feature_tracker_.setGridSize(grid_size);
	feature_tracker_.setComputeFeatureAngle(compute_surf_angle);

	//2.初始化图像尺寸
	im_rows_=0;		//输入图像尺寸
	im_cols_=0;

	sub_im_rows_=sub_im_rows;	//跟踪子图尺寸
	sub_im_cols_=sub_im_cols;

	//相机基本参数
	cam_cx_=sub_im_cols_/2;
	cam_cy_=sub_im_rows_/2;		//相机的图像中心
	fx_=max(sub_im_cols_,sub_im_rows_);
	fy_=fx_;					//相机xy焦距

	//初始化目标位置
	aim_position_.cx=0;
	aim_position_.cy=0;
	aim_position_.size_x=0;
	aim_position_.size_y=0;

	d_aim_position_.cx=0;		//更新目标位置变化差值
	d_aim_position_.cy=0;
	d_aim_position_.size_x=0;
	d_aim_position_.size_y=0;

	aim_weight_=1;

	//初始化子图坐标
	sub_im_x_=0;
	sub_im_y_=0;

	nsub_im_x_=0;
	nsub_im_y_=0;

	//初始化特征跟踪误差的几何分布模型参数
	geo_xx_=0.01;geo_xy_=0;geo_yy_=0.01;	//二次参数
	geo_b_=0.5;					//偏移

	//绘制目标跟踪结果
	draw_track_result_=true;
}

//设置是否绘制跟踪结果
void TrackerOnBoVW::setDrawTrackResult(bool draw)
{
	draw_track_result_=draw;
}

//设置相机基本参数
bool TrackerOnBoVW::setCameraPara(float fx,float fy,float cam_cx,float cam_cy)
{

	bool re=feature_tracker_.setCameraPara(fx,fy,cam_cx,cam_cy);
	if(re==false)
	{
		return false;
	}

	cam_cx_=cam_cx;
	cam_cy_=cam_cy;
	fx_=fx;
	fy_=fy;

	return true;
}

//载入视觉词典
bool TrackerOnBoVW::loadBoVW(string path)
{
	bool re=feature_tracker_.loadBoVW(path);

	return re;
}


//获得新的图像
bool TrackerOnBoVW::getNewFrame(cv::Mat& input_frame)
{
	//输入图像的尺寸
	int im_rows=input_frame.rows;
	int im_cols=input_frame.cols;

	if(im_rows<=1||im_cols<=1)
	{
		cout<<"error:->in TrackerOnBoVWSURF::getNewFrame 输入图像尺寸有误 "<<endl;
		return false;
	}

	
	if(im_rows_==0||im_cols==0)	//1.无第一张图片
	{
		im_rows_=im_rows;
		im_cols_=im_cols;

		image_in_=input_frame.clone();
	}
	else	//2.以前存在图片
	{
		if(im_rows_==im_rows&&im_cols_==im_cols)
		{
			image_in_=input_frame.clone();
		}
		else
		{
			cv::resize(input_frame,image_in_,cv::Size(im_cols_,im_rows_));
		}
	}

	return true;
}

//获得目标位置
bool TrackerOnBoVW::getAimPostion(Aim_Position& aim_p)
{
	if(aim_p.cx<0||aim_p.cy<0||aim_p.size_x<=1||aim_p.size_y<=1)
	{
		cout<<"error:->in TrackerOnBoVWSURF::getAimPostion 目标位置或尺寸有误 "<<endl;
		return false;
	}

	//计算子图位置
	nsub_im_x_=aim_p.cx-sub_im_cols_/2;
	nsub_im_y_=aim_p.cy-sub_im_rows_/2;

	if(nsub_im_x_<0)
	{
		nsub_im_x_=0;
	}
	else if(nsub_im_x_>im_cols_-sub_im_cols_)
	{
		nsub_im_x_=im_cols_-sub_im_cols_;
	}

	if(nsub_im_y_<0)
	{
		nsub_im_y_=0;
	}
	else if(nsub_im_y_>im_rows_-sub_im_rows_)
	{
		nsub_im_y_=im_rows_-sub_im_rows_;
	}

	new_aim_position_.cx=aim_p.cx-nsub_im_x_;
	new_aim_position_.cy=aim_p.cy-nsub_im_y_;

	new_aim_position_.size_x=min(aim_p.size_x,sub_im_cols_);
	new_aim_position_.size_y=min(aim_p.size_y,sub_im_rows_);

	d_aim_position_.cx=0;		//更新目标位置变化差值
	d_aim_position_.cy=0;
	d_aim_position_.size_x=0;
	d_aim_position_.size_y=0;

	aim_weight_=1;

	//更新参考图像
	cv::Mat sub_frame=image_in_(cv::Rect(nsub_im_x_,nsub_im_y_,sub_im_cols_,sub_im_rows_));
	bool gt=feature_tracker_.getNewFrame(sub_frame);
	if(gt==false)
	{
		return gt;
	}

	vector<int> matched_old_pid;
	vector<int> matched_new_pid;
	vector<float> score;
	cv::Mat pose_cg;
	feature_tracker_.trackFeaturebetweenFrames(matched_old_pid,matched_new_pid,score,pose_cg);

	bool re=updateReferFrame();
	if(re==false)
	{
		return re;
	}

	//更新参考帧每一个特征对目标的权重
	vector<cv::KeyPoint>& frame_old_point=feature_tracker_.key_point_old_;
	int old_pn=frame_old_point.size();
	aim_ref_pw_.resize(old_pn,0);

	int aim_region[4]={aim_p.cx-aim_p.size_x/2,aim_p.cx+aim_p.size_x/2,aim_p.cy-aim_p.size_y/2,aim_p.cy+aim_p.size_y/2};
	re=false;

	int aim_pn=0;
	for (int i = 0; i < old_pn; i++)
	{
		judgeAimRegionOldPoint(frame_old_point[i].pt,aim_region,re);

		if(re==true)
		{
			aim_ref_pw_[i]=1;
			aim_pn++;
		}
	}

	for (int i = 0; i < old_pn; i++)	//归一化
	{
		if(aim_ref_pw_[i]==1)
		{
			aim_ref_pw_[i]=aim_ref_pw_[i]/aim_pn;
		}

	}
	

	return true;
}

//更新目标位置
bool TrackerOnBoVW::updateAimPosition(Aim_Position& aim_p)
{
	//1.获取特征跟踪图像
	cv::Mat sub_frame=image_in_(cv::Rect(nsub_im_x_,nsub_im_y_,sub_im_cols_,sub_im_rows_));
	feature_tracker_.getNewFrame(sub_frame);

	//2.执行特征跟踪
	bool re=false;
	vector<int> matched_old_pid;
	vector<int> matched_new_pid;
	vector<float> score;
	cv::Mat pose_cg;

	re=feature_tracker_.trackFeaturebetweenFrames(matched_old_pid,matched_new_pid,score,pose_cg);
	if(re=false)
	{
		return false;
	}

	int match_p_n=matched_old_pid.size();
	if(match_p_n==0)	//第一帧图像
	{
		feature_tracker_.updateBaseFrame();
		return true;
	}

	//获得匹配特征点坐标
	vector<cv::Point2f> matched_old_point(match_p_n);
	vector<cv::Point2f> matched_new_point(match_p_n);

	for (int i = 0; i < match_p_n; i++)
	{
		matched_old_point[i]=feature_tracker_.key_point_old_[matched_old_pid[i]].pt;
		matched_new_point[i]=feature_tracker_.key_point_new_[matched_new_pid[i]].pt;
	}

	//3.筛选目标区域特征点
	vector<cv::Point2f> aim_old_p;
	vector<cv::Point2f> aim_new_p;
	vector<int> aim_pid;

	findAimRegionFeature(matched_old_point,matched_new_point,aim_old_p,aim_new_p,aim_pid);

	//4.前景特征权重分配
	vector<float> front_pw;
	re=frontFeatureWeight(aim_old_p,aim_new_p,front_pw,pose_cg);
	if(re=false)
	{
		return false;
	}

	int aim_pn=aim_old_p.size();
	vector<float> comp_pw(aim_pn);
	float total_w=0.00001;

	for(int i=0;i<aim_pn;i++)
	{
		int cur_mid=aim_pid[i];	//匹配点中目标区域id
		front_pw[i]=aim_ref_pw_[matched_old_pid[cur_mid]]*front_pw[i];
		comp_pw[i]=front_pw[i]*score[cur_mid];//综合权重=先验权重*几何权重*匹配权重
		total_w+=front_pw[i];
	}

	//更新当前图像前景权重
	int new_frame_pn=feature_tracker_.key_point_new_.size();
	aim_new_pw_.resize(new_frame_pn,0);

	for (int i = 0; i < aim_pn; i++)
	{
		int cur_mid=aim_pid[i];	//匹配点中目标区域id
		aim_new_pw_[matched_new_pid[cur_mid]]=front_pw[i]/total_w;	//先验权重归一化
	}

	//5.求解目标位移，旋转缩放观测值
	Aim_Position aimp_observed;
	re=computeObservedAimPosition(aim_old_p,aim_new_p,comp_pw,aimp_observed);
	if(re=false)
	{
		return false;
	}

	//6.更新目标位置
	if(aim_pn<3)
	{
		cout<<"warning: 目标上的特征点太少  无法求解精确的旋转平移 \n将根据图像总体变化输出目标的大致变化"<<endl;

		float* pose_cg_p=pose_cg.ptr<float>(0);
		new_aim_position_.cx=aim_position_.cx*pose_cg_p[0]-aim_position_.cy*pose_cg_p[1]+pose_cg_p[2];
		new_aim_position_.cy=aim_position_.cy*pose_cg_p[0]+aim_position_.cx*pose_cg_p[1]+pose_cg_p[3];
		new_aim_position_.size_x=aim_position_.size_x*pose_cg_p[0];
		new_aim_position_.size_y=aim_position_.size_y*pose_cg_p[0];
	}
	else
	{
		float ob_w=0;	//几何权值之和
		for(int i=0;i<aim_pn;i++)
		{
			ob_w+=front_pw[i];
		}

		re=aimPositionFilter(aimp_observed,ob_w);
		if (re==false)
		{
			return false;
		}
	}

	//计算全图坐标系下的位置
	//校正变化
	if(!(new_aim_position_.cx>0&&new_aim_position_.cx<320&&new_aim_position_.cy>0&&new_aim_position_.cy<240))
	{
		new_aim_position_=aim_position_;
	}

	aim_p=new_aim_position_;
	aim_p.cx+=nsub_im_x_;
	aim_p.cy+=nsub_im_y_;

	cout<<"目标框变化\n"<<new_aim_position_.cx<<"  "<<new_aim_position_.cy<<
		" "<<new_aim_position_.size_x<<" "<<new_aim_position_.size_y<<endl;	//test

	//更新子图位置
	nsub_im_x_=aim_p.cx-160;
	nsub_im_y_=aim_p.cy-120;

	if(nsub_im_x_<0)
	{
		nsub_im_x_=0;
	}
	else if (nsub_im_x_>im_cols_-320)
	{
		nsub_im_x_=im_cols_-320;
	}

	if(nsub_im_y_<0)
	{
		nsub_im_y_=0;
	}
	else if (nsub_im_y_>im_rows_-240)
	{
		nsub_im_y_=im_rows_-240;
	}

	/*//校正目标在子图中的位置
	new_aim_position_.cx=aim_p.cx-nsub_im_x_;
	new_aim_position_.cy=aim_p.cy-nsub_im_y_;

	if(new_aim_position_.cx<1)
	{
		new_aim_position_.cx=1;
	}
	else if(new_aim_position_.cx>319)
	{
		new_aim_position_.cx=319;
	}

	if(new_aim_position_.cy<1)
	{
		new_aim_position_.cy=1;
	}
	else if(new_aim_position_.cy>239)
	{
		new_aim_position_.cy=239;
	}*/

	//7.绘制跟踪结果
	if (draw_track_result_)
	{
		drawTrack(aim_new_p);
	}

	//8.根据特征跟踪状况考虑参考帧更新
	if (feature_tracker_.tracked_rate_<0.3&&aim_pn>4)	//特征跟踪率小于0.3且保证目标尚未被完全遮挡
		//(否则目标特征将被替换为遮挡物)
	{
		re=updateReferFrame();
		if(re=false)
		{
			return false;
		}
	}

	return true;
}

/********************** private函数 *************************/
//筛选目标区域特征点
void TrackerOnBoVW::findAimRegionFeature(vector<cv::Point2f>& matched_old_point,vector<cv::Point2f>& matched_new_point,vector<cv::Point2f>& aim_old_p,vector<cv::Point2f>& aim_new_p,vector<int>& aim_pid)
{
	//1.挑选目标范围内的特征点
	int point_n=matched_old_point.size();

	int aim_region[4];	//目标在参考图中的坐标范围minx maxx miny maxy
	aim_region[0]=max(0,aim_position_.cx-aim_position_.size_x/2);
	aim_region[1]=min(320,aim_position_.cx-aim_position_.size_x/2+aim_position_.size_x);
	aim_region[2]=max(0,aim_position_.cy-aim_position_.size_y/2);
	aim_region[3]=min(240,aim_position_.cy-aim_position_.size_y/2+aim_position_.size_y);

	aim_old_p.clear();
	aim_new_p.clear();
	aim_pid.clear();
	aim_old_p.reserve(point_n);
	aim_new_p.reserve(point_n);
	aim_pid.reserve(point_n);

	for(int i=0;i<point_n;i++)
	{
		cv::Point2f& old_p=matched_old_point[i];
		cv::Point2f& new_p=matched_new_point[i];
		bool re=false;

		judgeAimRegionOldPoint(old_p,aim_region,re);
			
		if(re==true)
		{
			aim_old_p.push_back(old_p);
			aim_new_p.push_back(new_p);
			aim_pid.push_back(i);
		}
	}
}

void TrackerOnBoVW::judgeAimRegionOldPoint(cv::Point2f& point,int aim_region[4],bool& re)
{
	if(point.x>=aim_region[0]&&point.x<=aim_region[1]&&point.y>=aim_region[2]&&point.y<=aim_region[3])
	{
		re=true;
	}
	else
		re=false;
}


//前景特征权重分配
bool TrackerOnBoVW::frontFeatureWeight(vector<cv::Point2f>& aim_old_p,vector<cv::Point2f>& aim_new_p,vector<float>& front_pw,cv::Mat& pose_cg)
{
	//1.计算匹配点的误差
	int aim_pn=aim_old_p.size();

	//位置变化误差
	vector<float> delta_x(aim_pn);
	vector<float> delta_y(aim_pn);

	float* pose_cg_ptr=pose_cg.ptr<float>(0);		//4变量图像变化

	for(int i=0;i<aim_pn;i++)
	{
		cv::Point2f& old_p=aim_old_p[i];
		cv::Point2f& new_p=aim_new_p[i];

		delta_x[i]=new_p.x-(old_p.x*pose_cg_ptr[0]-old_p.y*pose_cg_ptr[1]+pose_cg_ptr[2]);
		delta_y[i]=new_p.y-(old_p.y*pose_cg_ptr[0]+old_p.x*pose_cg_ptr[1]+pose_cg_ptr[3]);
	}

	//2.计算目标误差均值
	float delta_avx=0,delta_avy=0;	//误差加权均值

	for(int i=0;i<aim_pn;i++)
	{
		delta_avx+=delta_x[i];
		delta_avy+=delta_y[i];
	}

	delta_avx/=aim_pn;
	delta_avy/=aim_pn;

	//3.采用 2 sigma原则剔除背景点
	float s_x=0,s_y=0;
	vector<float> ddx(aim_pn),ddy(aim_pn);
	for(int i=0;i<aim_pn;i++)
	{
		ddx[i]=delta_x[i]-delta_avx;
		ddy[i]=delta_y[i]-delta_avy;

		s_x+=ddx[i]*ddx[i];
		s_y+=ddy[i]*ddy[i];
	}

	s_x=sqrt(s_x/aim_pn);
	s_y=sqrt(s_y/aim_pn);

	float th_x=2*s_x+1E-4;
	float th_y=2*s_y+1E-4;

	//4.重新计算前景匹配误差均值
	delta_avx=0;
	delta_avy=0;
	int in_point_n=0;//内点个数

	for(int i=0;i<aim_pn;i++)
	{
		if (abs(ddx[i])<th_x&&abs(ddy[i])<th_y)
		{
			delta_avx+=delta_x[i];
			delta_avy+=delta_y[i];
			in_point_n++;
		}
	}

	delta_avx/=in_point_n;
	delta_avy/=in_point_n;

	//5.计算各个匹配点相对前景均值误差
	for(int i=0;i<aim_pn;i++)
	{
		ddx[i]=delta_x[i]-delta_avx;
		ddy[i]=delta_y[i]-delta_avy;
	}

	//6.计算各点几何权重
	front_pw.resize(aim_pn);
	for(int i=0;i<aim_pn;i++)
	{
		front_pw[i]=4.0/(sqrt(ddx[i]*ddx[i]+ddy[i]*ddy[i]+1));	//可忍受误差的标准差在2个像素之内
	}

	return true;
}

//求解目标位移，旋转缩放观测值
bool TrackerOnBoVW::computeObservedAimPosition(vector<cv::Point2f>& aim_old_p,vector<cv::Point2f>& aim_new_p,vector<float>& comp_pw,Aim_Position& aimp_observed)
{
	//1.计算原图特征相对目标中心位移
	int aim_pn=aim_old_p.size();
	vector<cv::Point2f> old_cp(aim_pn);

	for(int i=0;i<aim_pn;i++)
	{
		old_cp[i].x=aim_old_p[i].x-aim_position_.cx;
		old_cp[i].y=aim_old_p[i].y-aim_position_.cy;
	}


	//2.估计位姿变化
	cv::Mat aim_trans_pos;
	bool pos_re=weightBaedImageCgEstimate(old_cp,aim_new_p,comp_pw,aim_trans_pos);
	if(pos_re==false)	//特征点太少
	{
		aimp_observed=new_aim_position_;
		return true;
	}

	//2.计算平移旋转缩放观测值
	float* aim_trans_ptr=aim_trans_pos.ptr<float>(0);
	aimp_observed.cx=aim_trans_ptr[2];
	aimp_observed.cy=aim_trans_ptr[3];

	aimp_observed.size_x=aim_position_.size_x*aim_trans_ptr[0];
	aimp_observed.size_y=aim_position_.size_y*aim_trans_ptr[0];

	if(aimp_observed.size_x<15)	//可观测的最小尺寸
	{
		aimp_observed.size_x=15;
	}

	if(aimp_observed.size_y<15)
	{
		aimp_observed.size_y=15;
	}


	return true;
}

//目标位置滤波更新
bool TrackerOnBoVW::aimPositionFilter(Aim_Position& aim_observed,float& total_w)
{
	//1.计算当前目标位置相对上一幅图像位置变化
	Aim_Position d_aim_p;
	d_aim_p.cx=aim_observed.cx-new_aim_position_.cx;
	d_aim_p.cy=aim_observed.cy-new_aim_position_.cy;
	d_aim_p.size_x=aim_observed.size_x-new_aim_position_.size_x;
	d_aim_p.size_y=aim_observed.size_y-new_aim_position_.size_y;

	if(abs(d_aim_p.cx)>30||abs(d_aim_p.cy)>30||abs(d_aim_p.size_x)>30||abs(d_aim_p.size_y)>30)
	{
		cout<<"warning: 目标框变化的观测值过大，目标框将不会更新"<<endl;
		return true;
	}

	//2.根据权值更新位置变化
	aim_weight_*=0.5;	//时间衰减因子0.5
	d_aim_position_.cx=(d_aim_p.cx*total_w+d_aim_position_.cx*aim_weight_)/(total_w+aim_weight_);
	d_aim_position_.cy=(d_aim_p.cy*total_w+d_aim_position_.cy*aim_weight_)/(total_w+aim_weight_);
	d_aim_position_.size_x=(d_aim_p.size_x*total_w+d_aim_position_.size_x*aim_weight_)/(total_w+aim_weight_);
	d_aim_position_.size_y=(d_aim_p.size_y*total_w+d_aim_position_.size_y*aim_weight_)/(total_w+aim_weight_);

	new_aim_position_.cx+=d_aim_position_.cx;
	new_aim_position_.cy+=d_aim_position_.cy;
	new_aim_position_.size_x+=d_aim_position_.size_x;
	new_aim_position_.size_y+=d_aim_position_.size_y;

	//3.更新权值变化
	aim_weight_=(aim_weight_*aim_weight_+total_w*total_w)/(aim_weight_+total_w);

	return true;
}

//更新参考图像
bool TrackerOnBoVW::updateReferFrame()
{
	//1.更新特征跟踪器参考帧
	bool re=feature_tracker_.updateBaseFrame();
	if(re==false)
	{
		return false;
	}

	//2.更新目标的子图坐标以及参考帧目标位置
	sub_im_x_=nsub_im_x_;
	sub_im_y_=nsub_im_y_;

	aim_position_=new_aim_position_;

	//3.更新特征先验权重
	aim_ref_pw_=aim_new_pw_;

	return true;
}

//绘制跟踪结果
void TrackerOnBoVW::drawTrack(vector<cv::Point2f>& aim_new_point)
{
	//1.绘制特征点
	int point_n=aim_new_point.size();
	vector<cv::KeyPoint> draw_point(point_n);

	for(int i=0;i<point_n;i++)
	{
		draw_point[i].response=500;
		draw_point[i].size=5;
		draw_point[i].octave=1;
		draw_point[i].pt.x=aim_new_point[i].x+nsub_im_x_;
		draw_point[i].pt.y=aim_new_point[i].y+nsub_im_y_;
	}

	cv::drawKeypoints(image_in_,draw_point,drawer_track_,CV_RGB(80,160,80),0);

	//2.绘制特跟踪框
	cv::rectangle(drawer_track_,cv::Rect(nsub_im_x_+new_aim_position_.cx-new_aim_position_.size_x/2,nsub_im_y_+new_aim_position_.cy-new_aim_position_.size_y/2,new_aim_position_.size_x,new_aim_position_.size_y),CV_RGB(255,255,255),3);

	cv::imshow("tracking result",drawer_track_);
	cv::waitKey(1);
}
