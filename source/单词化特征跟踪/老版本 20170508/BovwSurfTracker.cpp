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

#include "BovwSurfTracker.h"

/********************** public���� *************************/

//���ʼ��
BovwSurfTracker::BovwSurfTracker(double hessian_thre)
{
	// welcome
	cout<<"************* All right reserved by ��ά�� ***************"<<endl;
	cout<<"��ӭʹ�ã�BovwSurfTracker ��"<<endl;
	cout<<"���������BoVW�ʿ����SURF����"<<endl;

	cout<<"Ĭ�ϲ�����\n surf������ֵ 500 \n ��������ߴ� 10 \n �����Ƹ��ٽ��"<<endl
		<<"����ı����������ò����޸ĺ���"<<endl;

	//�����������
	cam_cx_=160;
	cam_cy_=120;		//�����ͼ������
	fx_=330;
	fy_=330;			//�������

	//���ݳ�ʼ�����������������������������
	surf_feature_detector_=cv::SurfFeatureDetector(hessian_thre,4,2,false,false);

	track_th_=0.008;		//surf����������ֵ
	grid_size_=10;			//����ߴ�
	draw_result_=false;		//���ƽ��

	row_grids_=(240+grid_size_-1)/grid_size_;	//�������
	col_grids_=(320+grid_size_-1)/grid_size_;	//�������
	old_grid_word_id_.resize(row_grids_*col_grids_);
	new_grid_word_id_.resize(row_grids_*col_grids_);

	//�ʵ����
	tree_levels_=0;
	tree_branches_=0;

	//����
	draw_tracking_result_=cv::Mat(240,640,CV_8UC3);
	tracked_rate_=0;
}

//����SURF�������ͣ��Ƿ����������������������������ٶȽ�����ۿۣ�
bool BovwSurfTracker::setComputeSURFangle(bool compute_surf_angle)
{
	if(compute_surf_angle==true)
		surf_feature_detector_.upright=0;//0������㷽��
	else
		surf_feature_detector_.upright=1;

	return true;
}

//����SURF����������ֵ
bool BovwSurfTracker::setSURFtrackThre(float track_th)
{
	if(track_th<0||track_th>=1)
	{
		cout<<"error: ���������������ֵ������Ч��Χ�� 0~1"<<endl;
		return false;
	}

	track_th_=track_th;
	return true;
}

//���ø�����������ߴ�
bool BovwSurfTracker::setGridSize(int grid_size)
{
	//�жϸ�����Ŀ�Ƿ���Ч
	if(grid_size<8||grid_size>50)
	{
		cout<<"error: ��������ߴ粻����Ч��Χ�� 8~50"<<endl;
		return false;
	}

	grid_size_=grid_size;

	row_grids_=(240+grid_size_-1)/grid_size_;	//�������
	col_grids_=(320+grid_size_-1)/grid_size_;	//�������
	old_grid_word_id_.resize(row_grids_*col_grids_);
	new_grid_word_id_.resize(row_grids_*col_grids_);

	return true;
}

//�����Ƿ���Ƹ��ٽ��
void BovwSurfTracker::setDrawTrackResult(bool draw)
{
	draw_result_=draw;
}

//���������������
bool BovwSurfTracker::setCameraPara(float fx,float fy,float cam_cx,float cam_cy)
{
	if(fx<=0||fy<=0)
	{
		cout<<"error:���뽹�����󣬽���������0"<<endl;
		return false;
	}

	if(cam_cx<=0||cam_cy<=0||cam_cx>=320||cam_cy>=240)
	{
		cout<<"error:�����������������������������������ͼ��Χ��"<<endl;
		return false;
	}

	fx_=fx;
	fy_=fy;

	cam_cx_=cam_cx;
	cam_cy_=cam_cy;
}

//�����Ӿ��ʵ�
bool BovwSurfTracker::loadBoVW(string path)
{
	//�Ӿ��ʵ��ʼ��
	if(path.size()==0)
	{
		path="BoVW of SURF/BoVW.dat";
	}

	bovw_surf_.loadBagofVisvalWord(path);
	bovw_surf_.getSizeofBoVW(tree_levels_,tree_branches_);

	return true;
}

//���¸��ٻ�׼ͼ
bool BovwSurfTracker::updateBaseFrame()
{
	if(new_frame_.rows==0)//��һ֡��ͼƬ��Ч
	{
		cout<<"��׼ͼ�޷����£������ͼ����Ч"<<endl;
		return false;
	}

	//����λ�˱仯
	trans_pose_=cv::Mat::zeros(4,1,CV_32FC1);		//λ����0
	trans_pose_.at<float>(0)=1;

	//���»�׼ͼSURF����
	surf_point_old_=surf_point_new_;
	surf_word_old_=surf_word_new_.clone();

	//���»�׼ͼSURF��������
	old_grid_word_id_=new_grid_word_id_;
	old_word_belone_=new_word_belone_;

	//���¸��ٽ��
	int new_point_n=surf_point_new_.size();
	frame_match_id_.resize(new_point_n);
	frame_match_score_.resize(new_point_n);

	for(int i=0;i<new_point_n;i++)
	{
		frame_match_id_[i]=i;
		frame_match_score_[i]=0;
	}

	return true;
}

//����µ�ͼ��
bool BovwSurfTracker::getNewFrame(cv::Mat& input_frame)
{
	//���ͼ���Ƿ���Ч
	if(input_frame.rows<240||input_frame.cols<320)
	{
		cout<<"error: ͼ��ߴ�̫С�������ͼ����Ч"<<endl;
		return false;
	}

	//�洢�µ�ͼ��
	if(input_frame.rows==240&&input_frame.cols==320)
	{
		new_frame_=input_frame;
	}
	else
	{
		cv::resize(input_frame,new_frame_,cv::Size(320,240));
	}

	return true;
}

//SURF��������
bool BovwSurfTracker::trackingSURFbetweenFrames(vector<cv::Point2f>& matched_old_point,vector<cv::Point2f>& matched_new_point,vector<float>& score,cv::Mat& pose_cg)
{
	//0.���BoVW�Ƿ���Ч
	if(tree_levels_==0||tree_branches_==0)
	{
		bool lr=loadBoVW();
		if(lr==false)
		{
			return false;
		}
	}

	//1.��������뵥�ʻ�
	bool fd=featureDetectAndWord();
	if(fd==false)
	{
		return false;
	}

	//2.������������������
	createRegionIndex();

	//3.�����ڽ������ѡƥ�����
	vector<vector<int>> pre_matcher;
	vector<vector<float>> match_score;

	bool fp=findPreMatched(pre_matcher,match_score);
	if(fp==false)
	{
		return false;
	}

	//4.���ݺ�ѡƥ����������תƽ�Ʋ��õ��Ϻõ�ƥ����
	cv::Mat trans_pose=trans_pose_.clone();
	vector<int> match_p;
	vector<float> match_sc;

	bool cp=computePoseChangeAndGoodMatch(pre_matcher,match_score,trans_pose,match_p,match_sc);
	if(cp==false)
	{
		return false;
	}

	//5.���¼���λ�˱仯�������޳����
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

	//6.��������ڲο�ͼ��ĸ��ٽ��
	updateReferenceResult();

	//7.���Ƹ���ͼ
	if(draw_result_)
	{
		drawPointTrack();
	}

	//8.������
	int old_pn=surf_point_old_.size();

	matched_old_point.clear();
	matched_new_point.clear();
	score.clear();

	matched_old_point.reserve(old_pn);
	matched_new_point.reserve(old_pn);
	score.reserve(old_pn);

	for(int i=0;i<old_pn;i++)
	{
		int new_id=frame_match_id_[i];
		if(new_id>=0)
		{
			matched_old_point.push_back(surf_point_old_[i].pt);
			matched_new_point.push_back(surf_point_new_[new_id].pt);

			score.push_back(frame_match_score_[i]);
		}
	}

	tracked_rate_=float(score.size())/old_pn;
	//cout<<"�ɹ������������� "<<tracked_rate_<<endl;//test

	return true;
}



/********************** private���� *************************/

//��������뵥�ʻ�
bool BovwSurfTracker::featureDetectAndWord()
{
	if(new_frame_.rows!=240||new_frame_.cols!=320)
	{
		cout<<"error:���ͼ����Ч��"<<endl;
		return false;
	}

	//1.�������
	surf_feature_detector_.detect(new_frame_,surf_point_new_);
	if(surf_point_new_.size()<12)
	{
		cout<<"error:��������̫�٣�"<<endl;
		return false;
	}

	//2.������������
	cv::Mat surf_descriptor_new;
	surf_descriptor_extractor_.compute(new_frame_,surf_point_new_,surf_descriptor_new);

	//3.�������ʻ�
	int surf_n=surf_descriptor_new.rows;
	int feature_size=surf_descriptor_new.cols;

	surf_word_new_=cv::Mat(surf_n,tree_levels_,CV_8UC1);
	
	cv::Mat out_feature_index;

	for(int i=0;i<surf_n;i++)
	{
		bovw_surf_.getIndexofSURF(surf_descriptor_new.row(i),surf_word_new_.row(i));
	}

	return true;
}

//������������������
void BovwSurfTracker::createRegionIndex()
{
	//1.ͳ�Ƹ���������������
	int surf_n=surf_point_new_.size();
	new_word_belone_.resize(surf_n);

	int grid_x,grid_y;

	for(int i=0;i<surf_n;i++)
	{
		grid_x=surf_point_new_[i].pt.x/grid_size_;
		grid_y=surf_point_new_[i].pt.y/grid_size_;

		new_word_belone_[i]=grid_y*col_grids_+grid_x;	//�����Ӧ�������
	}

	//2.ͳ�Ƹ����������������
	int grid_n=new_grid_word_id_.size();

	vector<int> grid_surf_n(grid_n,0);
	for(int i=0;i<surf_n;i++)
	{
		grid_surf_n[new_word_belone_[i]]++;
	}

	for(int i=0;i<grid_n;i++)
	{
		new_grid_word_id_[i].clear();
		new_grid_word_id_[i].reserve(grid_surf_n[i]);
	}

	for(int i=0;i<surf_n;i++)
	{
		new_grid_word_id_[new_word_belone_[i]].push_back(i);
	}
}

//�����ڽ������ѡƥ�����
bool BovwSurfTracker::findPreMatched(vector<vector<int>>& pre_matcher,vector<vector<float>>& match_score)
{
	//0.��������׼��
	int old_surf_n=surf_point_old_.size();
	int new_surf_n=surf_point_new_.size();

	pre_matcher.resize(old_surf_n);
	match_score.resize(old_surf_n);

	vector<float> level_w(tree_levels_);	//bovwÿ������Ȩ��
	level_w[0]=1.0;

	for(int i=1;i<tree_levels_;i++)
	{
		level_w[i]=level_w[i-1]/tree_branches_;
	}
	float best_d=level_w[tree_levels_-1]/tree_branches_;

	//1.�����׼ͼ��ÿ����������ͼ�е��ڽ������ѡƥ�����
	int old_g_x,old_g_y;
	int st_row,st_col,end_row,end_col;
	int new_word_id;
	float match_d;	//ƥ�����

	uchar* old_word_ptr,*new_word_ptr;

	for(int i=0;i<old_surf_n;i++)
	{
		old_word_ptr=surf_word_old_.ptr<uchar>(i);

		//������������
		pre_matcher[i].clear();
		pre_matcher[i].reserve(50);

		match_score[i].clear();
		match_score[i].reserve(50);

		old_g_y=old_word_belone_[i]/col_grids_;
		old_g_x=old_word_belone_[i]-old_g_y*col_grids_;

		//����ͼ�������ڽ�����
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

				int ng_surf_n=cur_grid.size();				//�������������������

				for(int l=0;l<ng_surf_n;l++)
				{
					new_word_id=cur_grid[l];
					new_word_ptr=surf_word_new_.ptr<uchar>(new_word_id);

					//���㵥�ʵ�ƥ�����
					match_d=best_d;
					for(int b=0;b<tree_levels_;b++)
					{
						if(old_word_ptr[b]!=new_word_ptr[b])
						{
							match_d=level_w[b];
							break;
						}
					}

					//������ֵɸѡԤƥ�䵥��
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

//���ݺ�ѡƥ����������תƽ�Ʋ��õ��Ϻõ�ƥ����
bool BovwSurfTracker::computePoseChangeAndGoodMatch(vector<vector<int>>& pre_matcher,vector<vector<float>>& match_score,cv::Mat& trans_pos,vector<int>& match_p,vector<float>& match_sc)
{
	//1.����ƥ���ԵĹ�һ������
	int old_point_n=surf_point_old_.size();
	int pre_mch_n=0;		//Ԥƥ�����

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

			old_p[match_id]=surf_point_old_[i].pt;
			new_p[match_id]=surf_point_new_[new_id].pt;
			weight_p[match_id]=match_score[i][j];

			match_id++;
		}
	}

	//�����������
	Camera_Para cam_para;
	cam_para.cx=cam_cx_;
	cam_para.cy=cam_cy_;
	cam_para.fx=fx_;
	cam_para.fy=fy_;

	/*//�����һ������
	vector<cv::Point2f> cam_old_p,cam_new_p;		//��һ�����

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

	//2.����λ�˱仯
	bool pos_re=weightBaedImageCgEstimate(old_p,new_p,weight_p,trans_pos);
	if(pos_re==false)
	{
		return false;
	}

	//3.�޳����
	float std_th=3;
	vector<float> d_points;		//��Ŀռ����
	vector<bool> inline_p;
	bool kick_re=kickOutpoint(trans_pos,std_th,old_p,new_p,d_points,inline_p);

	//4.���ڵ����ҵ����ʺϵ�ƥ���(���ü������Ʒ���)
	match_p.resize(old_point_n,-1);
	match_sc.resize(old_point_n);

	vector<int> match_inv(surf_point_new_.size(),-1);		//ƥ�����������

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

			float max_w=0;		//Ѱ�����ƥ��Ȩ��
			float max_dst=0;
			float max_id=-1;

			for(int j=0;j<cr_match_n;j++)
			{
				int new_id=pre_matcher[i][j];

				if(inline_p[match_id])		//�ڵ�
				{
					float c_w=weight_p[match_id]/(d_points[match_id]+0.0001);	//�ۺ�Ȩ��

					if(c_w>max_w)
					{
						int new_id_mtold=match_inv[new_id];	//new point�Ѿ�ƥ���old��
						if(new_id_mtold>=0)			//�жϸ�new point�Ƿ��Ѿ���ռ��
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
			match_sc[i]=max_dst;		//���ڹ���λ�ӱ仯����ƥ�����˴������������룬�����ۺ�Ȩ��

			if(max_id>=0)
			{
				match_inv[max_id]=i;
			}
		}
	}

	return true;
}


//���¼���λ�˱仯�������޳����,�õ�ƥ����
bool BovwSurfTracker::clickOutPoint(vector<int>& match_p,vector<float>& match_sc,float std_th,cv::Mat& trans_pos)
{
	//1.����ƥ���ԵĹ�һ������
	//ͳ��ƥ��ĵ�Ը���
	int point_n=match_p.size();		//ԭͼ��ĸ���
	int matched_pn=0;				//ƥ��ĵ�Ը���

	for(int i=0;i<point_n;i++)
	{
		if(match_p[i]>=0)
		{
			matched_pn++;
		}
	}

	//����ƥ�������
	vector<cv::Point2f> old_p(matched_pn),new_p(matched_pn);
	vector<float> weight_p(matched_pn);

	int match_id=0;
	for(int i=0;i<point_n;i++)
	{
		int new_id=match_p[i];

		if(new_id>=0)
		{
			old_p[match_id]=surf_point_old_[i].pt;
			new_p[match_id]=surf_point_new_[new_id].pt;
			weight_p[match_id]=match_sc[i];

			match_id++;
		}
	}

	//�����������
	Camera_Para cam_para;
	cam_para.cx=cam_cx_;
	cam_para.cy=cam_cy_;
	cam_para.fx=fx_;
	cam_para.fy=fy_;

	/*//�����һ������
	vector<cv::Point2f> cam_old_p,cam_new_p;		//��һ�����

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

	//2.����λ�˱仯
	bool pos_re=weightBaedImageCgEstimate(old_p,new_p,weight_p,trans_pos);
	if(pos_re==false)
	{
		return false;
	}

	//3.�޳���㣬�õ�ƥ����
	vector<float> d_points;		//�ƶ����
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


//��������ڲο�ͼ��ĸ��ٽ��
void BovwSurfTracker::updateReferenceResult()
{
	//1.����ԭͼ����ͼƥ����������ԭͼ��������Ӧ��ͼƬ�е�����
	vector<int> old_word_belone_new=old_word_belone_;	//�洢���� old point ����������
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

	old_word_belone_=old_word_belone_new;

	//4.����ͳ�Ƹ��������е�ԭͼ����
	vector<int> grid_word_n(region_n,0);	//��������ĵ��ʸ���

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


//���Ƹ���ͼ
void BovwSurfTracker::drawPointTrack()
{
	//1.��������
	cv::drawKeypoints(new_frame_,surf_point_new_,draw_tracking_result_,CV_RGB(80,160,80),0);

	//2.���������˶��켣
	int point_n=frame_match_id_.size();

	for(int i=0;i<point_n;i++)
	{
		int match_id=frame_match_id_[i];
		if(match_id>=0)
		{
			cv::line(draw_tracking_result_,surf_point_old_[i].pt,surf_point_new_[match_id].pt,CV_RGB(100,20,250));
		}
	}

	cv::imshow("point track",draw_tracking_result_);
	cv::waitKey(1);
}



/*//test
void main()
{
	leastSquaresTest();

	//BovwSurfTracker�ຯ������

	BovwSurfTracker surf_traker(500);
	surf_traker.setComputeSURFangle(false);
	surf_traker.setGridSize(10);
	surf_traker.setSURFtrackThre(0.006);
	surf_traker.setDrawTrackResult(true);
	surf_traker.loadBoVW();

	cout<<"����ͷ��ʼ��..."<<endl;
	cv::VideoCapture capture=cv::VideoCapture(1);//�洢����ͷ���������ָ��
	if(!capture.isOpened()) // ������ͷ
	{
		cout<<"�޷�������ͷ"<<endl;
		return;
	}

	int frame_n=0;
	cv::Mat frame_in;
	while(1)
	{
		frame_n++;

		capture >> frame_in;//��ȡͼ��
		bool get_re=surf_traker.getNewFrame(frame_in);

		if(get_re==false)//ͼ���������ȷ
		{
			continue;
		}

		float time_t=(float)cv::getTickCount();
		vector<cv::Point2f> matched_old_point,matched_new_point;
		vector<float> score;
		cv::Mat pose_cg;
		surf_traker.trackingSURFbetweenFrames(matched_old_point,matched_new_point,score,pose_cg);
		time_t = ((float)cv::getTickCount()-time_t)/cv::getTickFrequency();
		cout<<"trackingSURFbetweenFrames ��ʱ:"<<time_t<<"s"<<endl;

		cout<<"���������� "<<surf_traker.tracked_rate_<<endl;

		if(frame_n%20==0)
		{
			bool up_re=surf_traker.updateBaseFrame();
			if(up_re==false)
			{
				continue;
			}
		}

		//cout<<"��ǰ�ܵ�SURF������ĿΪ"<<samples_of_surf_.size()+lines_of_file<<endl;
		cv::imshow("BovwSurfTracker-test",frame_in);
		if(cv::waitKey(10)>0)//�˳�
		{
			break;
		}
	}
}*/
