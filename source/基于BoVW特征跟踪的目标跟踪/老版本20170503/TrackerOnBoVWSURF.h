/***************************************************************
	>����������SURF�������ٵ�Ŀ����ٳ��� (for windows��)
	>���ߣ���ά��
	>��ϵ��ʽ��248636779@163.com
	>����Ӿ���̼�����֡��SUEF������������
	>����Ҫ�㣺
	>1.SURF��������
	>2.����SURF����ͳ�Ƶ�Ŀ�����

	>ע�⣺���㷨�ɹ����ٵĻ�����Ŀ������ռ����ͼ�����Ĵ󲿷�
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
#include "BovwSurfTracker.h"			//�����Ӿ��ʿ������������

using namespace std;

struct Aim_Position
{
	float cx,cy;			//����λ��
	float size_x,size_y;	//��߳ߴ�
};

class TrackerOnBoVWSURF
{
public:
	/* ���ʼ��*/
	TrackerOnBoVWSURF(double hessian_thre=500,float track_th=0.004,int grid_size=10,bool compute_surf_angle=false);

	//�����Ƿ���Ƹ��ٽ��
	void setDrawTrackResult(bool draw);

	//���������������
	bool setCameraPara(float fx,float fy,float cam_cx=160,float cam_cy=120);

	//�����Ӿ��ʵ�
	bool loadBoVW(string path="BoVW of SURF/BoVW.dat");

	//����µ�ͼ��
	bool getNewFrame(cv::Mat& input_frame);

	//���Ŀ��λ��
	bool getAimPostion(Aim_Position& aim_p);

	//����Ŀ��λ��
	bool updateAimPosition(Aim_Position& aim_p);

	//����ͼ
	cv::Mat drawer_track_;
private:
	//����Ŀ�꼸��ģ��,�������ԭͼ�����ļ���Ȩ��
	void computeGeometricWeight(float& d_x,float& d_y,float& geo_weight);

	//���������������ķֲ��������Ŀ�꼸��ģ�Ͳ���
	bool updateGeometricErrorPara(vector<cv::Point2f>& matched_old_point,vector<cv::Point2f>& matched_new_point,vector<int>& aim_old_p_id,vector<cv::Point2f>& aim_old_point,vector<cv::Point2f>& aim_new_point,cv::Mat& pose_cg);

	//�����������ۺ�Ȩ�ظ���Ŀ��λ��
	bool computeAimPosition(vector<cv::Point2f>& aim_old_point,vector<cv::Point2f>& aim_new_point,vector<float>& comp_weight);

	//���²ο�ͼ��
	bool updateReferFrame();

	//���Ƹ��ٽ��
	void drawTrack(vector<cv::Point2f>& aim_new_point);

	//****************** �ڲ����� *****************

	//����������
	BovwSurfTracker surf_tracker_;

	//Ŀ������ͼ�е�λ��
	Aim_Position aim_position_;			//�ο�ͼ��Ŀ��λ��
	Aim_Position new_aim_position_;		//��ͼ��Ŀ��λ��

	//�����������
	float cam_cx_,cam_cy_;		//�����ͼ������
	float fx_,fy_;				//�������

	//Ŀ��������ͼ����
	float sub_im_x_,sub_im_y_;			//�ο�ͼ��ͼ��Χ
	float nsub_im_x_,nsub_im_y_;			//��ͼ��ͼ��Χ

	//�����������ļ��ηֲ�ģ�Ͳ���
	float geo_xx_,geo_xy_,geo_yy_;	//���η���
	float geo_b_;					//ƫ��

	//����ͼ��
	cv::Mat image_in_;	//����ͼ��

	int im_rows_,im_cols_;	//����ͼ��ߴ�

	//��������
	bool draw_track_result_;	//����Ŀ����ٽ��

};
