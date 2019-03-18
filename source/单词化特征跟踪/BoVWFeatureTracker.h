/***************************************************************
	>类名：相邻帧SURF特征检测与跟踪 (for windows版)
	>作者：李维鹏
	>联系方式：248636779@163.com
	>解决视觉里程计相邻帧的SUEF特征跟踪问题
	>技术要点：
	>1.SURF特征检测
	>2.SURF特征筛选与网格化局部匹配
	>3.基于概率的位姿变化估计与误匹配剔除

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
#include "TrainBoVWofFeature.h"			//视觉词库类
#include "poseEstimateKickOutpoint.h"	//快速Newton求解最小二乘类

using namespace std;

class BovwFeatureTracker
{
public:
	/* 类初始化*/
	BovwFeatureTracker(int frame_rows=240,int frame_cols=320,string feature_type="SURF");

	~BovwFeatureTracker();

	//设置特征类型（是否包含主方向，如果包含主方向，运算速度将大打折扣）
	bool setComputeFeatureAngle(bool compute_angle=false);

	//设置特征跟踪阈值
	bool setTrackThre(float track_th=0.008);

	//设置跟踪所用网格尺寸
	bool setGridSize(int grid_size=10);

	//设置是否绘制跟踪结果
	void setDrawTrackResult(bool draw);

	//设置相机基本参数
	bool setCameraPara(float fx,float fy,float cam_cx=160,float cam_cy=120);

	//载入视觉词典
	bool loadBoVW(string path="BoVW/SURF BoVW.dat");

	//更新跟踪基准图
	bool updateBaseFrame();

	//获得新的图像
	bool getNewFrame(cv::Mat& input_frame);

	//SURF特征跟踪
	bool trackFeaturebetweenFrames(vector<int>& matched_old_pointid,vector<int>& matched_new_pointid,vector<float>& score,cv::Mat& pose_cg);

	//特征跟踪结果图
	cv::Mat draw_tracking_result_;

	//跟踪比例
	float tracked_rate_;

	//特征提取结果
	vector<cv::KeyPoint> key_point_old_,key_point_new_;
	cv::Mat key_word_old_,key_word_new_;

private:

	//特征检测与单词化
	bool featureDetectAndWord();

	//生成新特征区域索引
	void createRegionIndex();

	//计算邻近区域候选匹配对象
	bool findPreMatched(vector<vector<int>>& pre_matcher,vector<vector<float>>& match_score);

	//根据候选匹配计算相机旋转平移并得到较好的匹配点对
	bool computePoseChangeAndGoodMatch(vector<vector<int>>& pre_matcher,vector<vector<float>>& match_score,cv::Mat& trans_pos,vector<int>& match_p,vector<float>& match_sc);

	//重新计算位姿变化，二次剔除外点
	bool clickOutPoint(vector<int>& match_p,vector<float>& match_sc,float std_th,cv::Mat& trans_pos);

	//更新相对于参考图像的跟踪结果,未发生跟踪的数据邻域关联
	void updateReferenceResult(cv::Mat& trans_pose);

	//绘制跟踪图
	void drawPointTrack();

	//****************** 内部变量 *****************

	//特征检测与匹配
	cv::Ptr<cv::FeatureDetector> feature_detector_;				//surf特征检测器
	cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_;		//surf特征描述向量生成器
	string feature_type_;

	float track_th_;		//surf跟踪阈值
	int grid_size_;			//网格尺寸
	bool draw_result_;		//绘制结果

	cv::Mat new_frame_;				//新的图片
	int frame_rows_,frame_cols_;	//图像尺寸

	//相机基本参数
	float cam_cx_,cam_cy_;		//相机的图像中心
	float fx_,fy_;				//相机焦距

	cv::Mat trans_pose_;		//位姿变化

	//视觉词袋
	TrainBoVWofFeature bovw_;						//视觉词典

	int tree_levels_,tree_branches_;							//词典层数与分支数

	//surf特征所属网格
	int row_grids_,col_grids_;		//网格个数
	vector<vector<int>> old_grid_word_id_,new_grid_word_id_;	//各个网格包含特征id
	vector<int> old_word_belone_,new_word_belone_;				//特征所属网格

	//SURF特征跟踪结果
	vector<int> frame_match_id_;
	vector<float> frame_match_score_;
};