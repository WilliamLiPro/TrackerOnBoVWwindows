/***************************************************************
	>�����������������ٵ�Ŀ����ٳ��� (for windows��)
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
#include "BovwFeatureTracker.h"			//�����Ӿ��ʿ������������

using namespace std;

struct Aim_Position
{
	float cx,cy;			//����λ��
	float size_x,size_y;	//��߳ߴ�
};

class TrackerOnBoVW
{
public:
	/* ���ʼ��*/
	TrackerOnBoVW(int sub_im_rows=240,int sub_im_cols=320,string feature_type="SURF",float track_th=0.004,int grid_size=10,bool compute_surf_angle=false);

	//�����Ƿ���Ƹ��ٽ��
	void setDrawTrackResult(bool draw);

	//���������������
	bool setCameraPara(float fx,float fy,float cam_cx=160,float cam_cy=120);

	//�����Ӿ��ʵ�
	bool loadBoVW(string path="BoVW/SURF BoVW.dat");

	//����µ�ͼ��
	bool getNewFrame(cv::Mat& input_frame);

	//���Ŀ��λ��
	bool getAimPostion(Aim_Position& aim_p);

	//����Ŀ��λ��
	bool updateAimPosition(Aim_Position& aim_p);

	//����ͼ
	cv::Mat drawer_track_;
private:
	//ɸѡĿ������������
	void findAimRegionFeature(vector<cv::Point2f>& matched_old_point,vector<cv::Point2f>& matched_new_point,vector<cv::Point2f>& aim_old_p,vector<cv::Point2f>& aim_new_p,vector<int>& aim_pid);
	void judgeAimRegionOldPoint(cv::Point2f& point,int aim_region[4],bool& re);

	//ǰ������Ȩ�ط���
	bool frontFeatureWeight(vector<cv::Point2f>& aim_old_p,vector<cv::Point2f>& aim_new_p,vector<float>& front_pw,cv::Mat& pose_cg);

	//���Ŀ��λ�ƣ���ת���Ź۲�ֵ
	bool computeObservedAimPosition(vector<cv::Point2f>& aim_old_p,vector<cv::Point2f>& aim_new_p,vector<float>& comp_pw,Aim_Position& aimp_observed);

	//Ŀ��λ���˲�����
	bool aimPositionFilter(Aim_Position& aim_observed,float& total_w);

	//���²ο�ͼ��
	bool updateReferFrame();

	//���Ƹ��ٽ��
	void drawTrack(vector<cv::Point2f>& aim_new_point);

	//****************** �ڲ����� *****************

	//����������
	BovwFeatureTracker feature_tracker_;

	//Ŀ������ͼ�е�λ��
	Aim_Position aim_position_;			//�ο�ͼ��Ŀ��λ��
	Aim_Position new_aim_position_;		//��ͼ��Ŀ��λ��
	Aim_Position d_aim_position_;		//����ͼ��Ŀ��λ��֮��
	float aim_weight_;					//Ŀ����Ȩ��

	//�����������
	float cam_cx_,cam_cy_;		//�����ͼ������
	float fx_,fy_;				//�������

	//Ŀ��������ͼ����
	int sub_im_x_,sub_im_y_;			//�ο�ͼ��ͼ��Χ
	int nsub_im_x_,nsub_im_y_;			//��ͼ��ͼ��Χ
	int sub_im_rows_,sub_im_cols_;		//��ͼ�ߴ�

	//�����������ļ��ηֲ�ģ�Ͳ���
	float geo_xx_,geo_xy_,geo_yy_;	//���η���
	float geo_b_;					//ƫ��

	//�ο�ͼ����Ŀ���������������Ȩ��
	vector<float> aim_ref_pw_;	//����Ȩ��
	vector<float> aim_new_pw_;	//Ŀ����������id

	//����ͼ��
	cv::Mat image_in_;	//����ͼ��

	int im_rows_,im_cols_;	//����ͼ��ߴ�

	//��������
	bool draw_track_result_;	//����Ŀ����ٽ��

};
