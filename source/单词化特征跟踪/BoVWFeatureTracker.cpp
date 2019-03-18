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
#include "stdafx.h"
#include "BovwFeatureTracker.h"

/********************** public函数 *************************/

//类初始化
BovwFeatureTracker::BovwFeatureTracker(int frame_rows,int frame_cols,string feature_type)
{
	// welcome
	cout<<"************* All right reserved by 李维鹏 ***************"<<endl;
	cout<<"欢迎使用：BovwFeatureTracker 类"<<endl;
	cout<<"本程序基于BoVW词库跟踪任意类型特征"<<endl;

	cout<<"默认参数：\n 输入图像尺寸 320*240 \n 跟踪网格尺寸 10 \n 不绘制跟踪结果"<<endl
		<<"如需改变参数，请调用参数修改函数"<<endl;

	//图像尺寸
	frame_rows_=frame_rows;
	frame_cols_=frame_cols;

	//相机基本参数
	cam_cx_=frame_cols/2;
	cam_cy_=frame_rows/2;		//相机的图像中心
	fx_=max(frame_rows_,frame_cols);
	fy_=fx_;					//相机xy焦距

	//数据初始化：特征检测器与描述向量生成器
	feature_type_=feature_type;
	cv::initModule_nonfree();
	feature_detector_=cv::FeatureDetector::create(feature_type);
	descriptor_extractor_=cv::DescriptorExtractor::create(feature_type);

	if(feature_type=="SURF")
	{
		try
		{
			feature_detector_->setBool("upright",true);	//默认true特征不计算方向
		}
		catch(cv::Exception ex)
		{

		}
	}

	track_th_=0.008;		//特征跟踪阈值
	grid_size_=10;			//网格尺寸
	draw_result_=false;		//绘制结果

	row_grids_=(frame_rows_+grid_size_-1)/grid_size_;	//网格个数
	col_grids_=(frame_cols_+grid_size_-1)/grid_size_;	//网格个数
	old_grid_word_id_.resize(row_grids_*col_grids_);
	new_grid_word_id_.resize(row_grids_*col_grids_);

	//词典参数
	tree_levels_=0;
	tree_branches_=0;

	//其他
	draw_tracking_result_=cv::Mat(frame_rows_,frame_cols_,CV_8UC3);
	tracked_rate_=0;
}

BovwFeatureTracker::~BovwFeatureTracker()
{
	//delete feature_detector_;
	//delete descriptor_extractor_;
}

//设置特征类型（是否包含主方向，如果包含主方向，运算速度将大打折扣）
bool BovwFeatureTracker::setComputeFeatureAngle(bool compute_angle)
{
	if(feature_type_=="SURF")
	{
		/*cv::initModule_nonfree();
		if(compute_angle==true)
		{
			try
			{
				feature_detector_->setBool("upright",false);//false代表计算方向
			}
			catch(cv::Exception except)
			{

			}
		}
		else
		{
			try
			{
				feature_detector_->setBool("upright",true);
			}
			catch(cv::Exception except)
			{

			}
		}*/
	}

	return true;
}

//设置特征跟踪阈值
bool BovwFeatureTracker::setTrackThre(float track_th)
{
	if(track_th<0||track_th>=1)
	{
		cout<<"error: 输入的特征跟踪阈值不在有效范围内 0~1"<<endl;
		return false;
	}

	track_th_=track_th;
	return true;
}

//设置跟踪所用网格尺寸
bool BovwFeatureTracker::setGridSize(int grid_size)
{
	//判断给定数目是否有效
	if(grid_size<8||grid_size>50)
	{
		cout<<"error: 输入网格尺寸不在有效范围内 8~50"<<endl;
		return false;
	}

	grid_size_=grid_size;

	row_grids_=(frame_rows_+grid_size_-1)/grid_size_;	//网格个数
	col_grids_=(frame_cols_+grid_size_-1)/grid_size_;	//网格个数
	old_grid_word_id_.resize(row_grids_*col_grids_);
	new_grid_word_id_.resize(row_grids_*col_grids_);

	return true;
}

//设置是否绘制跟踪结果
void BovwFeatureTracker::setDrawTrackResult(bool draw)
{
	draw_result_=draw;
}

//设置相机基本参数
bool BovwFeatureTracker::setCameraPara(float fx,float fy,float cam_cx,float cam_cy)
{
	if(fx<=0||fy<=0)
	{
		cout<<"error:输入焦距有误，焦距必须大于0"<<endl;
		return false;
	}

	if(cam_cx<=0||cam_cy<=0||cam_cx>=frame_cols_||cam_cy>=frame_rows_)
	{
		cout<<"error:输入相机中心坐标有误，相机中心坐标必须在图像范围内"<<endl;
		return false;
	}

	fx_=fx;
	fy_=fy;

	cam_cx_=cam_cx;
	cam_cy_=cam_cy;
}

//载入视觉词典
bool BovwFeatureTracker::loadBoVW(string path)
{
	//视觉词典初始化
	if(path.size()==0)
	{
		path="BoVW/";
		path=path+feature_type_+" BoVW.dat";
	}

	bovw_.loadBagofVisvalWord(path);
	bovw_.getSizeofBoVW(tree_levels_,tree_branches_);

	return true;
}

//更新跟踪基准图
bool BovwFeatureTracker::updateBaseFrame()
{
	if(new_frame_.rows==0)//上一帧的图片无效
	{
		cout<<"基准图无法更新，输入的图像无效"<<endl;
		return false;
	}

	//更新位姿变化
	trans_pose_=cv::Mat::zeros(4,1,CV_32FC1);		//位姿置0
	trans_pose_.at<float>(0)=1;

	//更新基准图特征
	key_point_old_=key_point_new_;
	key_word_old_=key_word_new_.clone();

	//更新基准图特征所属网格
	old_grid_word_id_=new_grid_word_id_;
	old_word_belone_=new_word_belone_;

	//更新跟踪结果
	int new_point_n=key_point_new_.size();
	frame_match_id_.resize(new_point_n);
	frame_match_score_.resize(new_point_n);

	for(int i=0;i<new_point_n;i++)
	{
		frame_match_id_[i]=i;
		frame_match_score_[i]=0;
	}

	return true;
}

//获得新的图像
bool BovwFeatureTracker::getNewFrame(cv::Mat& input_frame)
{
	//检测图像是否有效
	if(input_frame.rows<frame_rows_||input_frame.cols<frame_cols_)
	{
		cout<<"error: 图像尺寸太小，输入的图像无效"<<endl;
		return false;
	}

	//为提高跟踪鲁棒性，高斯滤波
	cv::Mat input_frame2;
	cv::GaussianBlur(input_frame,input_frame2,cv::Size(5,5),2);

	//存储新的图像
	if(input_frame2.rows==frame_rows_&&input_frame2.cols==frame_cols_)
	{
		new_frame_=input_frame2.clone();
	}
	else
	{
		cv::resize(input_frame2,new_frame_,cv::Size(frame_cols_,frame_rows_));
	}

	return true;
}

//SURF特征跟踪
bool BovwFeatureTracker::trackFeaturebetweenFrames(vector<int>& matched_old_pointid,vector<int>& matched_new_pointid,vector<float>& score,cv::Mat& pose_cg)
{
	//0.检测BoVW是否有效
	if(tree_levels_==0||tree_branches_==0)
	{
		bool lr=loadBoVW();
		if(lr==false)
		{
			return false;
		}
	}

	//1.特征检测与单词化
	bool fd=featureDetectAndWord();
	if(fd==false)
	{
		return false;
	}

	//2.生成新特征区域索引
	createRegionIndex();

	//3.计算邻近区域候选匹配对象
	vector<vector<int>> pre_matcher;
	vector<vector<float>> match_score;

	bool fp=findPreMatched(pre_matcher,match_score);
	if(fp==false)
	{
		return false;
	}

	//4.根据候选匹配计算相机旋转平移并得到较好的匹配点对
	cv::Mat trans_pose=trans_pose_.clone();
	vector<int> match_p;
	vector<float> match_sc;

	bool cp=computePoseChangeAndGoodMatch(pre_matcher,match_score,trans_pose,match_p,match_sc);
	if(cp==false)
	{
		return false;
	}

	//5.重新计算位姿变化，二次剔除外点
	float std_th=3;
	bool co=clickOutPoint(match_p,match_sc,std_th,trans_pose);
	if(co==false)
	{
		return false;
	}
	cout<<trans_pose<<endl;//test
	/*if(trans_pose.at<float>(0)<0.8)//test
	{
		cv::waitKey();
	}*/


	trans_pose_=trans_pose.clone();
	pose_cg=trans_pose_.clone();

	//6.更新相对于参考图像的跟踪结果
	updateReferenceResult(trans_pose);

	//7.绘制跟踪图
	if(draw_result_)
	{
		drawPointTrack();
	}

	//8.输出结果
	int old_pn=key_point_old_.size();

	matched_old_pointid.clear();
	matched_new_pointid.clear();
	score.clear();

	matched_old_pointid.reserve(old_pn);
	matched_new_pointid.reserve(old_pn);
	score.reserve(old_pn);

	for(int i=0;i<old_pn;i++)
	{
		int new_id=frame_match_id_[i];
		if(new_id>=0)
		{
			matched_old_pointid.push_back(i);
			matched_new_pointid.push_back(new_id);

			score.push_back(frame_match_score_[i]);
		}
	}

	tracked_rate_=float(score.size())/old_pn;
	//cout<<"成功跟踪特征比例 "<<tracked_rate_<<endl;//test

	return true;
}



/********************** private函数 *************************/

//特征检测与单词化
bool BovwFeatureTracker::featureDetectAndWord()
{
	if(new_frame_.rows!=frame_rows_||new_frame_.cols!=frame_cols_)
	{
		cout<<"error:检测图像无效！"<<endl;
		return false;
	}

	//1.特征检测
	feature_detector_->detect(new_frame_,key_point_new_);
	if(key_point_new_.size()<12)
	{
		cout<<"error:特征个数太少！"<<endl;
		return false;
	}

	//2.计算特征向量
	cv::Mat descriptor_new;
	descriptor_extractor_->compute(new_frame_,key_point_new_,descriptor_new);

	//3.特征单词化
	int feature_n=descriptor_new.rows;
	int feature_size=descriptor_new.cols;

	key_word_new_=cv::Mat(feature_n,tree_levels_,CV_8UC1);
	
	cv::Mat out_feature_index;

	for(int i=0;i<feature_n;i++)
	{
		bovw_.getIndexofFeature(descriptor_new.row(i),key_word_new_.row(i));
	}

	return true;
}

//生成新特征区域索引
void BovwFeatureTracker::createRegionIndex()
{
	//1.统计各个特征所属网格
	int feature_n=key_point_new_.size();
	new_word_belone_.resize(feature_n);

	int grid_x,grid_y;

	for(int i=0;i<feature_n;i++)
	{
		grid_x=key_point_new_[i].pt.x/grid_size_;
		grid_y=key_point_new_[i].pt.y/grid_size_;

		new_word_belone_[i]=grid_y*col_grids_+grid_x;	//计算对应网格序号
	}

	//2.统计各个网格包含的特征
	int grid_n=new_grid_word_id_.size();

	vector<int> grid_feature_n(grid_n,0);
	for(int i=0;i<feature_n;i++)
	{
		grid_feature_n[new_word_belone_[i]]++;
	}

	for(int i=0;i<grid_n;i++)
	{
		new_grid_word_id_[i].clear();
		new_grid_word_id_[i].reserve(grid_feature_n[i]);
	}

	for(int i=0;i<feature_n;i++)
	{
		new_grid_word_id_[new_word_belone_[i]].push_back(i);
	}
}

//计算邻近区域候选匹配对象
bool BovwFeatureTracker::findPreMatched(vector<vector<int>>& pre_matcher,vector<vector<float>>& match_score)
{
	//0.基本参数准备
	int old_key_n=key_point_old_.size();
	int new_key_n=key_point_new_.size();

	pre_matcher.resize(old_key_n);
	match_score.resize(old_key_n);

	vector<float> level_w(tree_levels_);	//bovw每个级别权重
	level_w[0]=1.0;

	for(int i=1;i<tree_levels_;i++)
	{
		level_w[i]=level_w[i-1]/tree_branches_;
	}
	float best_d=level_w[tree_levels_-1]/tree_branches_;

	//1.计算基准图中每个特征在新图中的邻近区域候选匹配对象
	int old_g_x,old_g_y;
	int st_row,st_col,end_row,end_col;
	int new_word_id;
	float match_d;	//匹配距离

	uchar* old_word_ptr,*new_word_ptr;

	if (old_word_belone_.size()!=old_key_n)
	{
		return false;
	}

	for(int i=0;i<old_key_n;i++)
	{
		old_word_ptr=key_word_old_.ptr<uchar>(i);

		//计算网格坐标
		pre_matcher[i].clear();
		pre_matcher[i].reserve(50);

		match_score[i].clear();
		match_score[i].reserve(50);

		old_g_y=old_word_belone_[i]/col_grids_;
		old_g_x=old_word_belone_[i]-old_g_y*col_grids_;

		//在新图中搜索邻近网格
		st_row=max(0,old_g_y-1);
		st_col=max(0,old_g_x-1);
		end_row=min(row_grids_-1,old_g_y+1);
		end_col=min(col_grids_-1,old_g_x+1);

		for(int j=st_row;j<=end_row;j++)
		{
			for(int k=st_col;k<=end_col;k++)
			{
				int grid_id=j*col_grids_+k;
				vector<int>& cur_grid=new_grid_word_id_[grid_id];

				int ng_key_n=cur_grid.size();				//新网格包含的特征个数

				for(int l=0;l<ng_key_n;l++)
				{
					new_word_id=cur_grid[l];
					new_word_ptr=key_word_new_.ptr<uchar>(new_word_id);

					//计算单词的匹配距离
					match_d=best_d;
					for(int b=0;b<tree_levels_;b++)
					{
						if(old_word_ptr[b]!=new_word_ptr[b])
						{
							match_d=level_w[b];
							break;
						}
					}

					//根据阈值筛选预匹配单词
					if(match_d<track_th_)
					{
						pre_matcher[i].push_back(new_word_id);
						match_score[i].push_back(1.0f/match_d);
					}
				}
			}
		}
	}

	return true;
}

//根据候选匹配计算相机旋转平移并得到较好的匹配点对
bool BovwFeatureTracker::computePoseChangeAndGoodMatch(vector<vector<int>>& pre_matcher,vector<vector<float>>& match_score,cv::Mat& trans_pos,vector<int>& match_p,vector<float>& match_sc)
{
	//1.计算匹配点对的归一化坐标
	int old_point_n=key_point_old_.size();
	int pre_mch_n=0;		//预匹配个数

	for(int i=0;i<old_point_n;i++)
	{
		pre_mch_n+=pre_matcher[i].size();
	}

	vector<cv::Point2f> old_p(pre_mch_n),new_p(pre_mch_n);
	vector<float> weight_p(pre_mch_n);
	int match_id=0;

	for(int i=0;i<old_point_n;i++)
	{
		int cr_match_n=pre_matcher[i].size();

		for(int j=0;j<cr_match_n;j++)
		{
			int new_id=pre_matcher[i][j];

			old_p[match_id]=key_point_old_[i].pt;
			new_p[match_id]=key_point_new_[new_id].pt;
			weight_p[match_id]=match_score[i][j];

			match_id++;
		}
	}

	//相机基本参数
	Camera_Para cam_para;
	cam_para.cx=cam_cx_;
	cam_para.cy=cam_cy_;
	cam_para.fx=fx_;
	cam_para.fy=fy_;

	/*//计算归一化坐标
	vector<cv::Point2f> cam_old_p,cam_new_p;		//归一化结果

	bool cg_re=pixCoordtoCameraCoord(old_p,cam_para,cam_old_p);
	if(cg_re==false)
	{
		return false;
	}

	cg_re=pixCoordtoCameraCoord(new_p,cam_para,cam_new_p);
	if(cg_re==false)
	{
		return false;
	}*/

	//2.估计位姿变化
	bool pos_re=weightBaedImageCgEstimate(old_p,new_p,weight_p,trans_pos);
	if(pos_re==false)
	{
		return false;
	}

	//3.剔除外点
	float std_th=3;
	vector<float> d_points;		//点的空间误差
	vector<bool> inline_p;
	bool kick_re=kickOutpoint(trans_pos,std_th,old_p,new_p,d_points,inline_p);

	//4.从内点中找到最适合的匹配点(采用极大抑制分配)
	match_p.resize(old_point_n,-1);
	match_sc.resize(old_point_n);

	vector<int> match_inv(key_point_new_.size(),-1);		//匹配的逆向索引

	for(int iter=0;iter<9;iter++)
	{
		match_id=0;

		for(int i=0;i<old_point_n;i++)
		{
			int cr_match_n=pre_matcher[i].size();

			if(match_p[i]>=0)
			{
				match_id+=cr_match_n;
				continue;
			}

			float max_w=0;		//寻找最大匹配权重
			float max_dst=0;
			float max_id=-1;

			for(int j=0;j<cr_match_n;j++)
			{
				int new_id=pre_matcher[i][j];

				if(inline_p[match_id])		//内点
				{
					float c_w=weight_p[match_id]/(d_points[match_id]+0.0001);	//综合权重

					if(c_w>max_w)
					{
						int new_id_mtold=match_inv[new_id];	//new point已经匹配的old点
						if(new_id_mtold>=0)			//判断该new point是否已经被占用
						{
							if(c_w>match_sc[new_id_mtold])
							{
								match_p[new_id_mtold]=-1;
								match_sc[new_id_mtold]=0;
								match_inv[new_id]=-1;

								max_w=c_w;
								max_dst=weight_p[match_id];
								max_id=new_id;
							}
						}
						else
						{
							max_w=c_w;
							max_dst=weight_p[match_id];
							max_id=new_id;
						}
					}
				}

				match_id++;
			}

			match_p[i]=max_id;
			match_sc[i]=max_dst;		//鉴于估计位子变化包含匹配误差，此处存贮特征距离，而非综合权重

			if(max_id>=0)
			{
				match_inv[max_id]=i;
			}
		}
	}

	return true;
}


//重新计算位姿变化，二次剔除外点,得到匹配结果
bool BovwFeatureTracker::clickOutPoint(vector<int>& match_p,vector<float>& match_sc,float std_th,cv::Mat& trans_pos)
{
	//1.计算匹配点对的归一化坐标
	//统计匹配的点对个数
	int point_n=match_p.size();		//原图点的个数
	int matched_pn=0;				//匹配的点对个数

	for(int i=0;i<point_n;i++)
	{
		if(match_p[i]>=0)
		{
			matched_pn++;
		}
	}

	//生成匹配点向量
	vector<cv::Point2f> old_p(matched_pn),new_p(matched_pn);
	vector<float> weight_p(matched_pn);

	int match_id=0;
	for(int i=0;i<point_n;i++)
	{
		int new_id=match_p[i];

		if(new_id>=0)
		{
			old_p[match_id]=key_point_old_[i].pt;
			new_p[match_id]=key_point_new_[new_id].pt;
			weight_p[match_id]=match_sc[i];

			match_id++;
		}
	}

	//相机基本参数
	Camera_Para cam_para;
	cam_para.cx=cam_cx_;
	cam_para.cy=cam_cy_;
	cam_para.fx=fx_;
	cam_para.fy=fy_;

	/*//计算归一化坐标
	vector<cv::Point2f> cam_old_p,cam_new_p;		//归一化结果

	bool cg_re=pixCoordtoCameraCoord(old_p,cam_para,cam_old_p);
	if(cg_re==false)
	{
		return false;
	}

	cg_re=pixCoordtoCameraCoord(new_p,cam_para,cam_new_p);
	if(cg_re==false)
	{
		return false;
	}*/

	//2.估计位姿变化
	bool pos_re=weightBaedImageCgEstimate(old_p,new_p,weight_p,trans_pos);
	if(pos_re==false)
	{
		return false;
	}

	//3.剔除外点，得到匹配结果
	vector<float> d_points;		//移动误差
	vector<bool> inline_p;
	bool kick_re=kickOutpoint(trans_pos,std_th,old_p,new_p,d_points,inline_p);

	frame_match_id_=cv::vector<int>(point_n,-1);
	frame_match_score_=cv::vector<float>(point_n,0);

	match_id=0;
	for(int i=0;i<point_n;i++)
	{
		if(match_p[i]>=0)
		{
			if(inline_p[match_id])
			{
				frame_match_id_[i]=match_p[i];
				frame_match_score_[i]=match_sc[i];
			}

			match_id++;
		}
	}

	return true;
}


//更新相对于参考图像的跟踪结果
void BovwFeatureTracker::updateReferenceResult(cv::Mat& trans_pose)
{
	//1.根据原图与新图匹配结果，更新原图各特征对应新图片中的区域
	vector<int> old_word_belone_new=old_word_belone_;	//存储各个 old point 所属新区域
	int old_point_n=frame_match_id_.size();
	int region_n=old_grid_word_id_.size();

	for(int i=0;i<old_point_n;i++)
	{
		int new_p_id=frame_match_id_[i];

		if(new_p_id>=0)
		{
			old_word_belone_new[i]=new_word_belone_[new_p_id];
		}
	}

	//2.未跟踪特征邻域关联
	float* trans_pose_ptr=trans_pose.ptr<float>(0);	//位姿变化

	int old_g_x,old_g_y;
	int st_row,st_col,end_row,end_col;
	int new_word_id;

	for(int i=0;i<old_point_n;i++)
	{
		int new_p_id=frame_match_id_[i];

		if(new_p_id>=0)	//跳过发生匹配的特征
		{
			continue;
		}

		//搜索3*3邻域特征,投票表决
		old_g_y=old_word_belone_[i]/col_grids_;				//旧特征所属网格
		old_g_x=old_word_belone_[i]-old_g_y*col_grids_;

		//在新图中搜索邻近网格
		st_row=max(0,old_g_y-1);
		st_col=max(0,old_g_x-1);
		end_row=min(row_grids_-1,old_g_y+1);
		end_col=min(col_grids_-1,old_g_x+1);

		int near_track_n=0;				//邻域跟踪特征个数
		int tracked_gx=0,tracked_gy=0;	//插值跟踪网格id

		for(int j=st_row;j<=end_row;j++)
		{
			for(int k=st_col;k<=end_col;k++)
			{
				int grid_id=j*col_grids_+k;
				vector<int>& cur_grid=old_grid_word_id_[grid_id];

				int og_key_n=cur_grid.size();				//原网格包含的特征个数

				for (int l=0;l<og_key_n;l++)
				{
					//判断邻域特征是否包含跟踪
					int near_ft_id=cur_grid[l];				//邻域特征序号
					new_p_id=frame_match_id_[near_ft_id];	//匹配的新特征

					if(new_p_id>=0)
					{
						tracked_gx+=k;
						tracked_gy+=j;

						near_track_n++;
					}
				}
			}
		}

		if(near_track_n>0)	//根据邻域特征所属网格确定当前特征的所属网格
		{
			tracked_gx=tracked_gx/near_track_n;
			tracked_gy=tracked_gy/near_track_n;
		}
		else				//无邻域跟踪结果,直接根据位姿变化计算特征坐标变化估计值
		{
			cv::Point2f cur_p=key_point_old_[i].pt;
			float old_p_trans_x=cur_p.x*trans_pose_ptr[0]-cur_p.y*trans_pose_ptr[1]+trans_pose_ptr[2];	//坐标变化
			float old_p_trans_y=cur_p.y*trans_pose_ptr[0]+cur_p.x*trans_pose_ptr[1]+trans_pose_ptr[3];

			tracked_gx=old_p_trans_x/grid_size_;	//所属网格
			tracked_gy=old_p_trans_y/grid_size_;
		}

		if(tracked_gx<col_grids_&&tracked_gx>=0&&tracked_gy<row_grids_&&tracked_gy>=0)	//网格id必须在有效范围内
		{
			int grid_id=tracked_gy*col_grids_+tracked_gx;
			old_word_belone_new[i]=grid_id;
		}
	}

	//更新特征所属网格
	old_word_belone_=old_word_belone_new;

	//3.重新统计各个区域中的原图特征
	vector<int> grid_word_n(region_n,0);	//各个区域的单词个数

	for(int i=0;i<old_point_n;i++)
	{
		grid_word_n[old_word_belone_[i]]++;
	}

	for(int i=0;i<region_n;i++)
	{
		old_grid_word_id_[i].clear();
		old_grid_word_id_[i].reserve(grid_word_n[i]);
	}

	for(int i=0;i<old_point_n;i++)
	{
		old_grid_word_id_[old_word_belone_[i]].push_back(i);
	}

}


//绘制跟踪图
void BovwFeatureTracker::drawPointTrack()
{
	//1.绘制特征
	cv::drawKeypoints(new_frame_,key_point_new_,draw_tracking_result_,CV_RGB(80,160,80),0);

	//2.绘制特征运动轨迹
	int point_n=frame_match_id_.size();

	for(int i=0;i<point_n;i++)
	{
		int match_id=frame_match_id_[i];
		if(match_id>=0)
		{
			cv::line(draw_tracking_result_,key_point_old_[i].pt,key_point_new_[match_id].pt,CV_RGB(100,20,250));
		}
	}

	cv::imshow("point track",draw_tracking_result_);
	cv::waitKey(1);
}


/*
//test
void main()
{
	//BovwSurfTracker类函数测试

	BovwFeatureTracker the_traker;
	the_traker.setComputeFeatureAngle(false);
	the_traker.setGridSize(10);
	the_traker.setTrackThre(0.006);
	the_traker.setDrawTrackResult(true);
	the_traker.loadBoVW();

	cout<<"摄像头初始化..."<<endl;
	cv::VideoCapture capture=cv::VideoCapture(1);//存储摄像头拍摄的数据指针
	if(!capture.isOpened()) // 打开摄像头
	{
		cout<<"无法打开摄像头"<<endl;
		return;
	}

	int frame_n=0;
	cv::Mat frame_in;
	while(1)
	{
		frame_n++;

		capture >> frame_in;//获取图像
		bool get_re=the_traker.getNewFrame(frame_in);

		if(get_re==false)//图像采样不正确
		{
			continue;
		}

		float time_t=(float)cv::getTickCount();
		vector<cv::Point2f> matched_old_point,matched_new_point;
		vector<float> score;
		cv::Mat pose_cg;
		the_traker.trackFeaturebetweenFrames(matched_old_point,matched_new_point,score,pose_cg);
		time_t = ((float)cv::getTickCount()-time_t)/cv::getTickFrequency();
		cout<<"trackingSURFbetweenFrames 用时:"<<time_t<<"s"<<endl;

		cout<<"特征跟踪率 "<<the_traker.tracked_rate_<<endl;

		if(frame_n%20==0)
		{
			bool up_re=the_traker.updateBaseFrame();
			if(up_re==false)
			{
				continue;
			}
		}

		//cout<<"当前总的SURF特征数目为"<<samples_of_surf_.size()+lines_of_file<<endl;
		cv::imshow("BovwSurfTracker-test",frame_in);
		if(cv::waitKey(10)>0)//退出
		{
			break;
		}
	}
}
*/