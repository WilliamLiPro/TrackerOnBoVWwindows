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
#include "TrainBoVWofFeature.h"			//�Ӿ��ʿ���
#include "poseEstimateKickOutpoint.h"	//����Newton�����С������

using namespace std;

class BovwFeatureTracker
{
public:
	/* ���ʼ��*/
	BovwFeatureTracker(int frame_rows=240,int frame_cols=320,string feature_type="SURF");

	~BovwFeatureTracker();

	//�����������ͣ��Ƿ����������������������������ٶȽ�����ۿۣ�
	bool setComputeFeatureAngle(bool compute_angle=false);

	//��������������ֵ
	bool setTrackThre(float track_th=0.008);

	//���ø�����������ߴ�
	bool setGridSize(int grid_size=10);

	//�����Ƿ���Ƹ��ٽ��
	void setDrawTrackResult(bool draw);

	//���������������
	bool setCameraPara(float fx,float fy,float cam_cx=160,float cam_cy=120);

	//�����Ӿ��ʵ�
	bool loadBoVW(string path="BoVW/SURF BoVW.dat");

	//���¸��ٻ�׼ͼ
	bool updateBaseFrame();

	//����µ�ͼ��
	bool getNewFrame(cv::Mat& input_frame);

	//SURF��������
	bool trackFeaturebetweenFrames(vector<int>& matched_old_pointid,vector<int>& matched_new_pointid,vector<float>& score,cv::Mat& pose_cg);

	//�������ٽ��ͼ
	cv::Mat draw_tracking_result_;

	//���ٱ���
	float tracked_rate_;

	//������ȡ���
	vector<cv::KeyPoint> key_point_old_,key_point_new_;
	cv::Mat key_word_old_,key_word_new_;

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

	//��������ڲο�ͼ��ĸ��ٽ��,δ�������ٵ������������
	void updateReferenceResult(cv::Mat& trans_pose);

	//���Ƹ���ͼ
	void drawPointTrack();

	//****************** �ڲ����� *****************

	//���������ƥ��
	cv::Ptr<cv::FeatureDetector> feature_detector_;				//surf���������
	cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_;		//surf������������������
	string feature_type_;

	float track_th_;		//surf������ֵ
	int grid_size_;			//����ߴ�
	bool draw_result_;		//���ƽ��

	cv::Mat new_frame_;				//�µ�ͼƬ
	int frame_rows_,frame_cols_;	//ͼ��ߴ�

	//�����������
	float cam_cx_,cam_cy_;		//�����ͼ������
	float fx_,fy_;				//�������

	cv::Mat trans_pose_;		//λ�˱仯

	//�Ӿ��ʴ�
	TrainBoVWofFeature bovw_;						//�Ӿ��ʵ�

	int tree_levels_,tree_branches_;							//�ʵ�������֧��

	//surf������������
	int row_grids_,col_grids_;		//�������
	vector<vector<int>> old_grid_word_id_,new_grid_word_id_;	//���������������id
	vector<int> old_word_belone_,new_word_belone_;				//������������

	//SURF�������ٽ��
	vector<int> frame_match_id_;
	vector<float> frame_match_score_;
};