/*************************************************
	>BoVW of Different Kind of Features

	>programmed by ��ά��
	>2016.5.13

	>1.����opencv��������ɼ�
	>2.����k-means�㷨���������࣬������ͬ��������BoVW
	>3.����ŷ�Ͼ������������

*************************************************/
#pragma once

#define _WINDOWS	//������WINDOWS������
//windows�����
#include <windows.h>

// C++��׼��
#include <iostream>
#include <fstream>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

using namespace std;

//BoVW�Ľṹ
struct K_D_tree //m��n����
{
	vector<vector<float>> kd_tree_data;//k-d���洢������
	int tree_levels;//���Ĳ���
	int branches;//ÿ���֧��
};

class TrainBoVWofFeature
{
public:
	/* ���ʼ��*/
	TrainBoVWofFeature(string feature_type="SURF",string descriptor_type="SURF");

	//�����ɼ���BoVW����
	bool getSamplesOfFeature(int data_type);//��������

	bool setBoVWlevelsAndBranches(int tree_levels,int branches);//����k-d���Ĳ����Լ���֧��

	bool buildBagofVisvalWord();//BoVW���㺯��

	bool loadBagofVisvalWord(string& filename);//�����Ӿ��ʵ�BoVW

	bool getDataFromBagofVisvalWord(vector<int>& data_index,vector<float>& data); //����Ӿ��ʵ䰴��data_index��������������

	bool getIndexofFeature(vector<float>& input_feature_data,vector<int>& feature_index);//����������������
	bool getIndexofFeature(cv::Mat& input_feature_data,cv::Mat& feature_index);//����������������

	void getSizeofBoVW(int& tree_levels,int& branches);//����Ӿ��ʵ��ģ

	K_D_tree getBagofVisvalWord(); //����Ӿ��ʵ���

private:
	//����ɼ�ͼ��
	bool catchFrame(cv::Mat& input_frame);

	//�����������Ӿ��ʵ�BoVW
	//�Ӿ��ʿ�Ϊ4��
	bool detectFeature();//������֡����
	bool myK_Means(vector<vector<float>>& samples,int classify_num,vector<vector<float>>& k_meas_result,vector<int>& samples_belones_result);//ͨ��k-means����BoVW����ľ�������

	//ͼ������ز���
	cv::Mat frame_now_;//�����ͼ��
	string feature_type_,descriptor_type_;				//��������
	cv::FeatureDetector* feature_detector_;//���������
	cv::DescriptorExtractor* feature_extractor_;//����������
	cv::Mat features_position_keyframe_,features_position_now_;//ǰһ�ؼ�֡�뵱ǰͼƬ����������
	cv::Mat features_descriptor_keyframe_,features_descriptor_now_;//ǰһ�ؼ�֡�뵱ǰͼƬ��������������
	vector<cv::DMatch> match_result_;//��֡ͼ��֮����������ٽ��
	vector<float> rgb_hist_keyframe_,rgb_hist_now_;//ǰһ�ؼ�֡�뵱ǰͼƬ��RGBͳ�ƽ��

	//BoVW��ز���
	vector<vector<float>> samples_of_feature_;//����������
	K_D_tree bag_of_visual_word_;//�����ʿ�,����tree_levels��branches��֧��
};