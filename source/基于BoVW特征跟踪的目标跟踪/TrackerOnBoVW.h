/***************************************************************
	>类名：基于特征跟踪的目标跟踪程序 (for windows版)
	>作者：李维鹏
	>联系方式：248636779@163.com
	>解决视觉里程计相邻帧的SUEF特征跟踪问题
	>技术要点：
	>1.SURF特征跟踪
	>2.基于SURF跟踪统计的目标跟踪

	>注意：该算法成功跟踪的基础是目标特征占据子图特征的大部分
****************************************************************/

#pragma once

#define _WINDOWS	//声明在WINDOWS下运行

// C++标准库
#include <iostream>
#include <fstream>
#include <vector>

// OpenCV
#include <opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

//自编库
#include "BovwFeatureTracker.h"			//基于视觉词库的特征跟踪类

using namespace std;

struct Aim_Position
{
	float cx,cy;			//中心位置
	float size_x,size_y;	//宽高尺寸
};

class TrackerOnBoVW
{
public:
	/* 类初始化*/
	TrackerOnBoVW(int sub_im_rows=240,int sub_im_cols=320,string feature_type="SURF",float track_th=0.004,int grid_size=10,bool compute_surf_angle=false);

	//设置是否绘制跟踪结果
	void setDrawTrackResult(bool draw);

	//设置相机基本参数
	bool setCameraPara(float fx,float fy,float cam_cx=160,float cam_cy=120);

	//载入视觉词典
	bool loadBoVW(string path="BoVW/SURF BoVW.dat");

	//获得新的图像
	bool getNewFrame(cv::Mat& input_frame);

	//获得目标位置
	bool getAimPostion(Aim_Position& aim_p);

	//更新目标位置
	bool updateAimPosition(Aim_Position& aim_p);

	//跟踪图
	cv::Mat drawer_track_;
private:
	//筛选目标区域特征点
	void findAimRegionFeature(vector<cv::Point2f>& matched_old_point,vector<cv::Point2f>& matched_new_point,vector<cv::Point2f>& aim_old_p,vector<cv::Point2f>& aim_new_p,vector<int>& aim_pid);
	void judgeAimRegionOldPoint(cv::Point2f& point,int aim_region[4],bool& re);

	//前景特征权重分配
	bool frontFeatureWeight(vector<cv::Point2f>& aim_old_p,vector<cv::Point2f>& aim_new_p,vector<float>& front_pw,cv::Mat& pose_cg);

	//求解目标位移，旋转缩放观测值
	bool computeObservedAimPosition(vector<cv::Point2f>& aim_old_p,vector<cv::Point2f>& aim_new_p,vector<float>& comp_pw,Aim_Position& aimp_observed);

	//目标位置滤波更新
	bool aimPositionFilter(Aim_Position& aim_observed,float& total_w);

	//更新参考图像
	bool updateReferFrame();

	//绘制跟踪结果
	void drawTrack(vector<cv::Point2f>& aim_new_point);

	//****************** 内部变量 *****************

	//特征跟踪类
	BovwFeatureTracker feature_tracker_;

	//目标在子图中的位置
	Aim_Position aim_position_;			//参考图中目标位置
	Aim_Position new_aim_position_;		//新图中目标位置
	Aim_Position d_aim_position_;		//相邻图像目标位置之差
	float aim_weight_;					//目标总权重

	//相机基本参数
	float cam_cx_,cam_cy_;		//相机的图像中心
	float fx_,fy_;				//相机焦距

	//目标所在子图坐标
	int sub_im_x_,sub_im_y_;			//参考图子图范围
	int nsub_im_x_,nsub_im_y_;			//新图子图范围
	int sub_im_rows_,sub_im_cols_;		//子图尺寸

	//特征跟踪误差的几何分布模型参数
	float geo_xx_,geo_xy_,geo_yy_;	//二次方差
	float geo_b_;					//偏移

	//参考图像中目标区域包含的特征权重
	vector<float> aim_ref_pw_;	//特征权重
	vector<float> aim_new_pw_;	//目标区域特征id

	//输入图像
	cv::Mat image_in_;	//输入图像

	int im_rows_,im_cols_;	//输入图像尺寸

	//其他参数
	bool draw_track_result_;	//绘制目标跟踪结果

};
