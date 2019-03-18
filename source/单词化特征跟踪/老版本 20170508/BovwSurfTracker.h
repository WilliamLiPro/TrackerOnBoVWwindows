/***************************************************************
	>����������֡SURF������������ (for windows��)
	>���ߣ���ά��
	>��ϵ��ʽ��248636779@163.com
	>����Ӿ���̼�����֡��SUEF������������
	>����Ҫ�㣺
	>1.SURF�������
	>2.SURF����ɸѡ�����񻯾ֲ�ƥ��
	>3.���ڸ��ʵ�λ�˱仯��������ƥ���޳�

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
#include "TrainBoVWofSURF.h"			//�Ӿ��ʿ���
#include "poseEstimateKickOutpoint.h"		//����Newton�����С������

using namespace std;

class BovwSurfTracker
{
public:
	/* ���ʼ��*/
	BovwSurfTracker(double hessian_thre=500);

	//����SURF�������ͣ��Ƿ����������������������������ٶȽ�����ۿۣ�
	bool setComputeSURFangle(bool compute_surf_angle=false);

	//����SURF����������ֵ
	bool setSURFtrackThre(float track_th=0.02);

	//���ø�����������ߴ�
	bool setGridSize(int grid_size=10);

	//�����Ƿ���Ƹ��ٽ��
	void setDrawTrackResult(bool draw);

	//���������������
	bool setCameraPara(float fx,float fy,float cam_cx=160,float cam_cy=120);

	//�����Ӿ��ʵ�
	bool loadBoVW(string path="BoVW of SURF/BoVW.dat");

	//���¸��ٻ�׼ͼ
	bool updateBaseFrame();

	//����µ�ͼ��
	bool getNewFrame(cv::Mat& input_frame);

	//SURF��������
	bool trackingSURFbetweenFrames(vector<cv::Point2f>& matched_old_point,vector<cv::Point2f>& matched_new_point,vector<float>& score,cv::Mat& pose_cg);

	//�������ٽ��ͼ
	cv::Mat draw_tracking_result_;

	//���ٱ���
	float tracked_rate_;

private:

	//��������뵥�ʻ�
	bool featureDetectAndWord();

	//������������������
	void createRegionIndex();

	//�����ڽ������ѡƥ�����
	bool findPreMatched(vector<vector<int>>& pre_matcher,vector<vector<float>>& match_score);

	//���ݺ�ѡƥ����������תƽ�Ʋ��õ��Ϻõ�ƥ����
	bool computePoseChangeAndGoodMatch(vector<vector<int>>& pre_matcher,vector<vector<float>>& match_score,cv::Mat& trans_pos,vector<int>& match_p,vector<float>& match_sc);

	//���¼���λ�˱仯�������޳����
	bool clickOutPoint(vector<int>& match_p,vector<float>& match_sc,float std_th,cv::Mat& trans_pos);

	//��������ڲο�ͼ��ĸ��ٽ��
	void updateReferenceResult();

	//���Ƹ���ͼ
	void drawPointTrack();

	//****************** �ڲ����� *****************

	//���������ƥ��
	cv::SurfFeatureDetector surf_feature_detector_;				//surf���������
	cv::SurfDescriptorExtractor surf_descriptor_extractor_;		//surf������������������

	float track_th_;		//surf������ֵ
	int grid_size_;			//����ߴ�
	bool draw_result_;		//���ƽ��

	cv::Mat new_frame_;		//�µ�ͼƬ

	//�����������
	float cam_cx_,cam_cy_;		//�����ͼ������
	float fx_,fy_;				//�������

	cv::Mat trans_pose_;		//λ�˱仯

	//ͼƬ�е�SURF��������
	TrainBoVWofSURF bovw_surf_;						//�Ӿ��ʵ�

	int tree_levels_,tree_branches_;							//�ʵ�������֧��
	vector<cv::KeyPoint> surf_point_old_,surf_point_new_;
	cv::Mat surf_word_old_,surf_word_new_;

	//surf������������
	int row_grids_,col_grids_;		//�������
	vector<vector<int>> old_grid_word_id_,new_grid_word_id_;	//���������������id
	vector<int> old_word_belone_,new_word_belone_;				//������������

	//SURF�������ٽ��
	vector<int> frame_match_id_;
	vector<float> frame_match_score_;
};