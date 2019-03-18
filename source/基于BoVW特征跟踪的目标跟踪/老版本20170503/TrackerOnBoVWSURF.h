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
#include "BovwSurfTracker.h"			//基于视觉词库的特征跟踪类

using namespace std;

struct Aim_Position
{
	float cx,cy;			//中心位置
	float size_x,size_y;	//宽高尺寸
};

class TrackerOnBoVWSURF
{
public:
	/* 类初始化*/
	TrackerOnBoVWSURF(double hessian_thre=500,float track_th=0.004,int grid_size=10,bool compute_surf_angle=false);

	//设置是否绘制跟踪结果
	void setDrawTrackResult(bool draw);

	//设置相机基本参数
	bool setCameraPara(float fx,float fy,float cam_cx=160,float cam_cy=120);

	//载入视觉词典
	bool loadBoVW(string path="BoVW of SURF/BoVW.dat");

	//获得新的图像
	bool getNewFrame(cv::Mat& input_frame);

	//获得目标位置
	bool getAimPostion(Aim_Position& aim_p);

	//更新目标位置
	bool updateAimPosition(Aim_Position& aim_p);

	//跟踪图
	cv::Mat drawer_track_;
private:
	//根据目标几何模型,计算给定原图特征的几何权重
	void computeGeometricWeight(float& d_x,float& d_y,float& geo_weight);

	//根据特征跟踪误差的分布情况更新目标几何模型参数
	bool updateGeometricErrorPara(vector<cv::Point2f>& matched_old_point,vector<cv::Point2f>& matched_new_point,vector<int>& aim_old_p_id,vector<cv::Point2f>& aim_old_point,vector<cv::Point2f>& aim_new_point,cv::Mat& pose_cg);

	//根据特征的综合权重更新目标位置
	bool computeAimPosition(vector<cv::Point2f>& aim_old_point,vector<cv::Point2f>& aim_new_point,vector<float>& comp_weight);

	//更新参考图像
	bool updateReferFrame();

	//绘制跟踪结果
	void drawTrack(vector<cv::Point2f>& aim_new_point);

	//****************** 内部变量 *****************

	//特征跟踪类
	BovwSurfTracker surf_tracker_;

	//目标在子图中的位置
	Aim_Position aim_position_;			//参考图中目标位置
	Aim_Position new_aim_position_;		//新图中目标位置

	//相机基本参数
	float cam_cx_,cam_cy_;		//相机的图像中心
	float fx_,fy_;				//相机焦距

	//目标所在子图坐标
	float sub_im_x_,sub_im_y_;			//参考图子图范围
	float nsub_im_x_,nsub_im_y_;			//新图子图范围

	//特征跟踪误差的几何分布模型参数
	float geo_xx_,geo_xy_,geo_yy_;	//二次方差
	float geo_b_;					//偏移

	//输入图像
	cv::Mat image_in_;	//输入图像

	int im_rows_,im_cols_;	//输入图像尺寸

	//其他参数
	bool draw_track_result_;	//绘制目标跟踪结果

};
