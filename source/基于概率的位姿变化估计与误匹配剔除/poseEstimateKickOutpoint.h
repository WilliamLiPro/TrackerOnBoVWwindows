/***************************************************************
	>程序：基于概率的位姿变化估计与外点剔除
	>作者：李维鹏
	>联系方式：248636779@163.com
	
	>技术要点：
	>1.概率权值最小二乘求解位姿变化
	>2.基于标准差的外点剔除
	>3.点的坐标归一化

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
#include "leastSquaresAndNewton.h"		//快速Newton求解最小二乘类

using namespace std;

struct Camera_Para
{
	float cx,cy;
	float fx,fy;
};

/*	概率权值最小二乘求解位姿变化
	采用牛顿法求解最小二乘

	输入：
	原始点坐标序列		vector<cv::Point2f>& origin_p		
	匹配点坐标序列		vector<cv::Point2f>& match_p
	匹配权重			vector<float>& weight_p

	输出：
	位姿变化			cv::Mat& pose_cg
	*/
bool weightBaedPoseEstimate(vector<cv::Point2f>& origin_p,vector<cv::Point2f>& match_p,vector<float>& weight_p,cv::Mat& pose_cg);

/*	概率权值最小二乘求解图像旋转平移变化
	采用解析法求解最小二乘

	输入：
	原始点坐标序列		vector<cv::Point2f>& origin_p		
	匹配点坐标序列		vector<cv::Point2f>& match_p
	匹配权重			vector<float>& weight_p

	输出：
	图像变化			cv::Mat& pose_cg
	*/
bool weightBaedImageCgEstimate(vector<cv::Point2f>& origin_p,vector<cv::Point2f>& match_p,vector<float>& weight_p,cv::Mat& pose_cg);

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
bool kickOutpoint(cv::Mat& pose_cg,float std_th,vector<cv::Point2f>& origin_p,vector<cv::Point2f>& match_p,vector<float>& d_points,vector<bool>& inline_p);

/*	点的坐标归一化
	已知像素坐标，由相机基本参数得到归一化的点坐标

	输入：
	像素坐标序列		vector<cv::Point2f>& pix_p
	相机参数			Cmera_Para& cam_para

	输出：
	归一化坐标序列		vector<cv::Point2f>& cam_p
*/
bool pixCoordtoCameraCoord(vector<cv::Point2f>& pix_p,Camera_Para& cam_para,vector<cv::Point2f>& cam_p);

//经过旋转平移后的特征点位置变化
void matchPoseChangeX(cv::Mat& fun_in,cv::Mat& undt_para,float& fun_out);
void matchPoseChangeY(cv::Mat& fun_in,cv::Mat& undt_para,float& fun_out);