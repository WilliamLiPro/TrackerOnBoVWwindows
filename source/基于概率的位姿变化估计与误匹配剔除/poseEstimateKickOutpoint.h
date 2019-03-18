/***************************************************************
	>���򣺻��ڸ��ʵ�λ�˱仯����������޳�
	>���ߣ���ά��
	>��ϵ��ʽ��248636779@163.com
	
	>����Ҫ�㣺
	>1.����Ȩֵ��С�������λ�˱仯
	>2.���ڱ�׼�������޳�
	>3.��������һ��

****************************************************************/
#pragma once

#define _WINDOWS	//������WINDOWS������

// C++��׼��
#include <iostream>
#include <fstream>
#include <vector>

// OpenCV
#include <opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

//�Ա��
#include "leastSquaresAndNewton.h"		//����Newton�����С������

using namespace std;

struct Camera_Para
{
	float cx,cy;
	float fx,fy;
};

/*	����Ȩֵ��С�������λ�˱仯
	����ţ�ٷ������С����

	���룺
	ԭʼ����������		vector<cv::Point2f>& origin_p		
	ƥ�����������		vector<cv::Point2f>& match_p
	ƥ��Ȩ��			vector<float>& weight_p

	�����
	λ�˱仯			cv::Mat& pose_cg
	*/
bool weightBaedPoseEstimate(vector<cv::Point2f>& origin_p,vector<cv::Point2f>& match_p,vector<float>& weight_p,cv::Mat& pose_cg);

/*	����Ȩֵ��С�������ͼ����תƽ�Ʊ仯
	���ý����������С����

	���룺
	ԭʼ����������		vector<cv::Point2f>& origin_p		
	ƥ�����������		vector<cv::Point2f>& match_p
	ƥ��Ȩ��			vector<float>& weight_p

	�����
	ͼ��仯			cv::Mat& pose_cg
	*/
bool weightBaedImageCgEstimate(vector<cv::Point2f>& origin_p,vector<cv::Point2f>& match_p,vector<float>& weight_p,cv::Mat& pose_cg);

/*	���ڱ�׼�������޳�

	���룺
	λ�˱仯			cv::Mat& pose_cg
	��׼�����ֵ		float std_th
	ԭʼ����������		vector<cv::Point2f>& origin_p		
	ƥ�����������		vector<cv::Point2f>& match_p

	�����
	�������			vector<float>& d_points
	�ڵ���			vector<bool>& inline_p
	*/
bool kickOutpoint(cv::Mat& pose_cg,float std_th,vector<cv::Point2f>& origin_p,vector<cv::Point2f>& match_p,vector<float>& d_points,vector<bool>& inline_p);

/*	��������һ��
	��֪�������꣬��������������õ���һ���ĵ�����

	���룺
	������������		vector<cv::Point2f>& pix_p
	�������			Cmera_Para& cam_para

	�����
	��һ����������		vector<cv::Point2f>& cam_p
*/
bool pixCoordtoCameraCoord(vector<cv::Point2f>& pix_p,Camera_Para& cam_para,vector<cv::Point2f>& cam_p);

//������תƽ�ƺ��������λ�ñ仯
void matchPoseChangeX(cv::Mat& fun_in,cv::Mat& undt_para,float& fun_out);
void matchPoseChangeY(cv::Mat& fun_in,cv::Mat& undt_para,float& fun_out);