/*************************************************
	>BoVW of Different Kind of Features

	>programmed by 李维鹏
	>2016.5.13

	>1.基于opencv库的特征采集
	>2.采用k-means算法的特征聚类，构建不同类特征的BoVW
	>3.基于欧氏距离的特征索引

*************************************************/
#pragma once

#define _WINDOWS	//声明在WINDOWS下运行
//windows界面库
#include <windows.h>

// C++标准库
#include <iostream>
#include <fstream>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

using namespace std;

//BoVW的结构
struct K_D_tree //m层n叉树
{
	vector<vector<float>> kd_tree_data;//k-d树存储的数据
	int tree_levels;//数的层数
	int branches;//每层分支树
};

class TrainBoVWofFeature
{
public:
	/* 类初始化*/
	TrainBoVWofFeature(string feature_type="SURF",string descriptor_type="SURF");

	//特征采集与BoVW计算
	bool getSamplesOfFeature(int data_type);//特征采样

	bool setBoVWlevelsAndBranches(int tree_levels,int branches);//设置k-d树的层数以及分支数

	bool buildBagofVisvalWord();//BoVW计算函数

	bool loadBagofVisvalWord(string& filename);//载入视觉词典BoVW

	bool getDataFromBagofVisvalWord(vector<int>& data_index,vector<float>& data); //输出视觉词典按照data_index索引的特征数据

	bool getIndexofFeature(vector<float>& input_feature_data,vector<int>& feature_index);//计算特征向量索引
	bool getIndexofFeature(cv::Mat& input_feature_data,cv::Mat& feature_index);//计算特征向量索引

	void getSizeofBoVW(int& tree_levels,int& branches);//获得视觉词典规模

	K_D_tree getBagofVisvalWord(); //输出视觉词典树

private:
	//输入采集图像
	bool catchFrame(cv::Mat& input_frame);

	//采样及构造视觉词典BoVW
	//视觉词库为4层
	bool detectFeature();//特征单帧采样
	bool myK_Means(vector<vector<float>>& samples,int classify_num,vector<vector<float>>& k_meas_result,vector<int>& samples_belones_result);//通过k-means计算BoVW各层的聚类中心

	//图像处理相关参数
	cv::Mat frame_now_;//输入的图像
	string feature_type_,descriptor_type_;				//特征类型
	cv::FeatureDetector* feature_detector_;//特征检测器
	cv::DescriptorExtractor* feature_extractor_;//特征描述器
	cv::Mat features_position_keyframe_,features_position_now_;//前一关键帧与当前图片的特征坐标
	cv::Mat features_descriptor_keyframe_,features_descriptor_now_;//前一关键帧与当前图片的特征描述向量
	vector<cv::DMatch> match_result_;//两帧图像之间的特征跟踪结果
	vector<float> rgb_hist_keyframe_,rgb_hist_now_;//前一关键帧与当前图片的RGB统计结果

	//BoVW相关参数
	vector<vector<float>> samples_of_feature_;//特征样本集
	K_D_tree bag_of_visual_word_;//特征词库,采用tree_levels层branches分支树
};