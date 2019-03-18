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

#include "TrackerOnBoVWSURF.h"

/********************** public���� *************************/

TrackerOnBoVWSURF::TrackerOnBoVWSURF(double hessian_thre,float track_th,int grid_size,bool compute_surf_angle)
{
	//1.������������ʼ��
	surf_tracker_=BovwSurfTracker::BovwSurfTracker(hessian_thre);
	surf_tracker_.setSURFtrackThre(track_th);
	surf_tracker_.setGridSize(grid_size);
	surf_tracker_.setComputeSURFangle(compute_surf_angle);

	//2.��ʼ��ͼ��ߴ�
	im_rows_=0;
	im_cols_=0;

	//�����������
	cam_cx_=160;
	cam_cy_=120;		//�����ͼ������
	fx_=330;
	fy_=330;			//�������

	//��ʼ��Ŀ��λ��
	aim_position_.cx=0;
	aim_position_.cy=0;
	aim_position_.size_x=0;
	aim_position_.size_y=0;

	d_aim_position_.cx=0;		//����Ŀ��λ�ñ仯��ֵ
	d_aim_position_.cy=0;
	d_aim_position_.size_x=0;
	d_aim_position_.size_y=0;

	aim_weight_=1;

	//��ʼ����ͼ����
	sub_im_x_=0;
	sub_im_y_=0;

	nsub_im_x_=0;
	nsub_im_y_=0;

	//��ʼ�������������ļ��ηֲ�ģ�Ͳ���
	geo_xx_=0.01;geo_xy_=0;geo_yy_=0.01;	//���β���
	geo_b_=0.5;					//ƫ��

	//����Ŀ����ٽ��
	draw_track_result_=true;
}

//�����Ƿ���Ƹ��ٽ��
void TrackerOnBoVWSURF::setDrawTrackResult(bool draw)
{
	draw_track_result_=draw;
}

//���������������
bool TrackerOnBoVWSURF::setCameraPara(float fx,float fy,float cam_cx,float cam_cy)
{

	bool re=surf_tracker_.setCameraPara(fx,fy,cam_cx,cam_cy);
	if(re==false)
	{
		return false;
	}

	cam_cx_=cam_cx;
	cam_cy_=cam_cy;
	fx_=fx;
	fy_=fy;

	return true;
}

//�����Ӿ��ʵ�
bool TrackerOnBoVWSURF::loadBoVW(string path)
{
	bool re=surf_tracker_.loadBoVW(path);

	return re;
}


//����µ�ͼ��
bool TrackerOnBoVWSURF::getNewFrame(cv::Mat& input_frame)
{
	//����ͼ��ĳߴ�
	int im_rows=input_frame.rows;
	int im_cols=input_frame.cols;

	if(im_rows<=1||im_cols<=1)
	{
		cout<<"error:->in TrackerOnBoVWSURF::getNewFrame ����ͼ��ߴ����� "<<endl;
		return false;
	}

	
	if(im_rows_==0||im_cols==0)	//1.�޵�һ��ͼƬ
	{
		im_rows_=im_rows;
		im_cols_=im_cols;

		image_in_=input_frame.clone();
	}
	else	//2.��ǰ����ͼƬ
	{
		if(im_rows_==im_rows&&im_cols_==im_cols)
		{
			image_in_=input_frame.clone();
		}
		else
		{
			cv::resize(input_frame,image_in_,cv::Size(im_cols_,im_rows_));
		}
	}

	return true;
}

//���Ŀ��λ��
bool TrackerOnBoVWSURF::getAimPostion(Aim_Position& aim_p)
{
	if(aim_p.cx<0||aim_p.cy<0||aim_p.size_x<=1||aim_p.size_y<=1)
	{
		cout<<"error:->in TrackerOnBoVWSURF::getAimPostion Ŀ��λ�û�ߴ����� "<<endl;
		return false;
	}

	//������ͼλ��
	nsub_im_x_=aim_p.cx-160;
	nsub_im_y_=aim_p.cy-120;

	if(nsub_im_x_<0)
	{
		nsub_im_x_=0;
	}
	else if(nsub_im_x_>im_cols_-320)
	{
		nsub_im_x_=im_cols_-320;
	}

	if(nsub_im_y_<0)
	{
		nsub_im_y_=0;
	}
	else if(nsub_im_y_>im_rows_-240)
	{
		nsub_im_y_=im_rows_-240;
	}

	new_aim_position_.cx=aim_p.cx-nsub_im_x_;
	new_aim_position_.cy=aim_p.cy-nsub_im_y_;

	new_aim_position_.size_x=min(aim_p.size_x,320);
	new_aim_position_.size_y=min(aim_p.size_y,240);

	d_aim_position_.cx=0;		//����Ŀ��λ�ñ仯��ֵ
	d_aim_position_.cy=0;
	d_aim_position_.size_x=0;
	d_aim_position_.size_y=0;

	aim_weight_=1;

	//���²ο�ͼ��
	cv::Mat sub_frame=image_in_(cv::Rect(nsub_im_x_,nsub_im_y_,320,240));
	bool gt=surf_tracker_.getNewFrame(sub_frame);
	if(gt==false)
	{
		return gt;
	}

	vector<cv::Point2f> matched_old_point;
	vector<cv::Point2f> matched_new_point;
	vector<float> score;
	cv::Mat pose_cg;
	surf_tracker_.trackingSURFbetweenFrames(matched_old_point,matched_new_point,score,pose_cg);

	bool re=updateReferFrame();
	if(re==false)
	{
		return re;
	}

	return true;
}

//����Ŀ��λ��
bool TrackerOnBoVWSURF::updateAimPosition(Aim_Position& aim_p)
{
	//1.��ȡ��������ͼ��
	cv::Mat sub_frame=image_in_(cv::Rect(nsub_im_x_,nsub_im_y_,320,240));
	surf_tracker_.getNewFrame(sub_frame);

	//2.ִ����������
	bool re=false;
	vector<cv::Point2f> matched_old_point;
	vector<cv::Point2f> matched_new_point;
	vector<float> score;
	cv::Mat pose_cg;

	re=surf_tracker_.trackingSURFbetweenFrames(matched_old_point,matched_new_point,score,pose_cg);
	if(re=false)
	{
		return false;
	}

	if(matched_old_point.size()==0)	//��һ֡ͼ��
	{
		surf_tracker_.updateBaseFrame();
		return true;
	}

	//������������״�����ǲο�֡����
	if (surf_tracker_.tracked_rate_<0.2)	//����������С��0.2
	{
		re=updateReferFrame();
		if(re=false)
		{
			return false;
		}
	}

	//3.ɸѡĿ������������
	vector<cv::Point2f> aim_old_p;
	vector<cv::Point2f> aim_new_p;
	vector<int> aim_pid;

	findAimRegionFeature(matched_old_point,matched_new_point,aim_old_p,aim_new_p,aim_pid);

	//4.ǰ������Ȩ�ط���
	vector<float> front_pw;
	re=frontFeatureWeight(aim_old_p,aim_new_p,front_pw,pose_cg);
	if(re=false)
	{
		return false;
	}

	int aim_pn=aim_old_p.size();
	vector<float> comp_pw(aim_pn);

	for(int i=0;i<aim_pn;i++)
	{
		comp_pw[i]=front_pw[i]*score[aim_pid[i]];
	}

	//5.���Ŀ��λ�ƣ���ת���Ź۲�ֵ
	Aim_Position aimp_observed;
	re=computeObservedAimPosition(aim_old_p,aim_new_p,comp_pw,aimp_observed);
	if(re=false)
	{
		return false;
	}

	//6.����Ŀ��λ��
	if(aim_pn<3)
	{
		cout<<"warning: Ŀ���ϵ�������̫��  �޷���⾫ȷ����תƽ�� \n������ͼ������仯���Ŀ��Ĵ��±仯"<<endl;

		float* pose_cg_p=pose_cg.ptr<float>(0);
		new_aim_position_.cx=aim_position_.cx*pose_cg_p[0]-aim_position_.cy*pose_cg_p[1]+pose_cg_p[2];
		new_aim_position_.cy=aim_position_.cy*pose_cg_p[0]+aim_position_.cx*pose_cg_p[1]+pose_cg_p[3];
		new_aim_position_.size_x=aim_position_.size_x*pose_cg_p[0];
		new_aim_position_.size_y=aim_position_.size_y*pose_cg_p[0];
	}
	else
	{
		float ob_w=0;	//����Ȩֵ֮��
		for(int i=0;i<aim_pn;i++)
		{
			ob_w+=front_pw[i];
		}

		re=aimPositionFilter(aimp_observed,ob_w);
		if (re==false)
		{
			return false;
		}
	}

	//����ȫͼ����ϵ�µ�λ��
	//У���仯
	if(!(new_aim_position_.cx>0&&new_aim_position_.cx<320&&new_aim_position_.cy>0&&new_aim_position_.cy<240))
	{
		new_aim_position_=aim_position_;
	}

	aim_p=new_aim_position_;
	aim_p.cx+=nsub_im_x_;
	aim_p.cy+=nsub_im_y_;

	cout<<"Ŀ���仯\n"<<new_aim_position_.cx<<"  "<<new_aim_position_.cy<<
		" "<<new_aim_position_.size_x<<" "<<new_aim_position_.size_y<<endl;	//test

	//������ͼλ��
	nsub_im_x_=aim_p.cx-160;
	nsub_im_y_=aim_p.cy-120;

	if(nsub_im_x_<0)
	{
		nsub_im_x_=0;
	}
	else if (nsub_im_x_>im_cols_-320)
	{
		nsub_im_x_=im_cols_-320;
	}

	if(nsub_im_y_<0)
	{
		nsub_im_y_=0;
	}
	else if (nsub_im_y_>im_rows_-240)
	{
		nsub_im_y_=im_rows_-240;
	}

	//У��Ŀ������ͼ�е�λ��
	new_aim_position_.cx=aim_p.cx-nsub_im_x_;
	new_aim_position_.cy=aim_p.cy-nsub_im_y_;

	if(new_aim_position_.cx<1)
	{
		new_aim_position_.cx=1;
	}
	else if(new_aim_position_.cx>319)
	{
		new_aim_position_.cx=319;
	}

	if(new_aim_position_.cy<1)
	{
		new_aim_position_.cy=1;
	}
	else if(new_aim_position_.cy>239)
	{
		new_aim_position_.cy=239;
	}

	//7.���Ƹ��ٽ��
	if (draw_track_result_)
	{
		drawTrack(aim_new_p);
	}

	return true;
}

/********************** private���� *************************/
//ɸѡĿ������������
void TrackerOnBoVWSURF::findAimRegionFeature(vector<cv::Point2f>& matched_old_point,vector<cv::Point2f>& matched_new_point,vector<cv::Point2f>& aim_old_p,vector<cv::Point2f>& aim_new_p,vector<int>& aim_pid)
{
	//1.��ѡĿ�귶Χ�ڵ�������
	int point_n=matched_old_point.size();

	int aim_sx,aim_sy,aim_ex,aim_ey;	//Ŀ���ڲο�ͼ�е����귶Χ
	aim_sx=max(0,aim_position_.cx-aim_position_.size_x/2);
	aim_sy=max(0,aim_position_.cy-aim_position_.size_y/2);
	aim_ex=min(320,aim_position_.cx-aim_position_.size_x/2+aim_position_.size_x);
	aim_ey=min(240,aim_position_.cy-aim_position_.size_y/2+aim_position_.size_y);

	aim_old_p.clear();
	aim_new_p.clear();
	aim_pid.clear();
	aim_old_p.reserve(point_n);
	aim_new_p.reserve(point_n);
	aim_pid.reserve(point_n);

	for(int i=0;i<point_n;i++)
	{
		cv::Point2f& old_p=matched_old_point[i];
		cv::Point2f& new_p=matched_new_point[i];

		if(old_p.x>=aim_sx&&old_p.x<=aim_ex&&old_p.y>=aim_sy&&old_p.y<=aim_ey)
		{
			aim_old_p.push_back(old_p);
			aim_new_p.push_back(new_p);
			aim_pid.push_back(i);
		}
	}
}

//ǰ������Ȩ�ط���
bool TrackerOnBoVWSURF::frontFeatureWeight(vector<cv::Point2f>& aim_old_p,vector<cv::Point2f>& aim_new_p,vector<float>& front_pw,cv::Mat& pose_cg)
{
	//1.����ƥ�������
	int aim_pn=aim_old_p.size();

	//λ�ñ仯���
	vector<float> delta_x(aim_pn);
	vector<float> delta_y(aim_pn);

	float* pose_cg_ptr=pose_cg.ptr<float>(0);		//4����ͼ��仯

	for(int i=0;i<aim_pn;i++)
	{
		cv::Point2f& old_p=aim_old_p[i];
		cv::Point2f& new_p=aim_new_p[i];

		delta_x[i]=new_p.x-(old_p.x*pose_cg_ptr[0]-old_p.y*pose_cg_ptr[1]+pose_cg_ptr[2]);
		delta_y[i]=new_p.y-(old_p.y*pose_cg_ptr[0]+old_p.x*pose_cg_ptr[1]+pose_cg_ptr[3]);
	}

	//2.����Ŀ������ֵ
	float delta_avx=0,delta_avy=0;	//����Ȩ��ֵ

	for(int i=0;i<aim_pn;i++)
	{
		delta_avx+=delta_x[i];
		delta_avy+=delta_y[i];
	}

	delta_avx/=aim_pn;
	delta_avy/=aim_pn;

	//3.���� 3 sigmaԭ���޳�������
	float s_x=0,s_y=0;
	vector<float> ddx(aim_pn),ddy(aim_pn);
	for(int i=0;i<aim_pn;i++)
	{
		ddx[i]=delta_x[i]-delta_avx;
		ddy[i]=delta_y[i]-delta_avy;

		s_x+=ddx[i]*ddx[i];
		s_y+=ddy[i]*ddy[i];
	}

	s_x=sqrt(s_x/aim_pn);
	s_y=sqrt(s_y/aim_pn);

	float th_x=3*s_x+1E-4;
	float th_y=3*s_y+1E-4;

	//4.���¼���ǰ��ƥ������ֵ
	delta_avx=0;
	delta_avy=0;
	int in_point_n=0;//�ڵ����

	for(int i=0;i<aim_pn;i++)
	{
		if (abs(ddx[i])<th_x&&abs(ddy[i])<th_y)
		{
			delta_avx+=delta_x[i];
			delta_avy+=delta_y[i];
			in_point_n++;
		}
	}

	delta_avx/=in_point_n;
	delta_avy/=in_point_n;

	//5.�������ƥ������ǰ����ֵ���
	for(int i=0;i<aim_pn;i++)
	{
		ddx[i]=delta_x[i]-delta_avx;
		ddy[i]=delta_y[i]-delta_avy;
	}

	//6.������㼸��Ȩ��
	front_pw.resize(aim_pn);
	for(int i=0;i<aim_pn;i++)
	{
		front_pw[i]=1.0/(sqrt(ddx[i]*ddx[i]+ddy[i]*ddy[i]+1));	//���������ı�׼����1������֮��
	}

	return true;
}

//���Ŀ��λ�ƣ���ת���Ź۲�ֵ
bool TrackerOnBoVWSURF::computeObservedAimPosition(vector<cv::Point2f>& aim_old_p,vector<cv::Point2f>& aim_new_p,vector<float>& comp_pw,Aim_Position& aimp_observed)
{
	//1.����ԭͼ�������Ŀ������λ��
	int aim_pn=aim_old_p.size();
	vector<cv::Point2f> old_cp(aim_pn);

	for(int i=0;i<aim_pn;i++)
	{
		old_cp[i].x=aim_old_p[i].x-aim_position_.cx;
		old_cp[i].y=aim_old_p[i].y-aim_position_.cy;
	}


	//2.����λ�˱仯
	cv::Mat aim_trans_pos;
	bool pos_re=weightBaedImageCgEstimate(old_cp,aim_new_p,comp_pw,aim_trans_pos);
	if(pos_re==false)	//������̫��
	{
		aimp_observed=new_aim_position_;
		return true;
	}

	//2.����ƽ����ת���Ź۲�ֵ
	float* aim_trans_ptr=aim_trans_pos.ptr<float>(0);
	aimp_observed.cx=aim_trans_ptr[2];
	aimp_observed.cy=aim_trans_ptr[3];

	aimp_observed.size_x=aim_position_.size_x*aim_trans_ptr[0];
	aimp_observed.size_y=aim_position_.size_y*aim_trans_ptr[0];

	if(aimp_observed.size_x<15)	//�ɹ۲����С�ߴ�
	{
		aimp_observed.size_x=15;
	}

	if(aimp_observed.size_y<15)
	{
		aimp_observed.size_y=15;
	}


	return true;
}

//Ŀ��λ���˲�����
bool TrackerOnBoVWSURF::aimPositionFilter(Aim_Position& aim_observed,float& total_w)
{
	//1.���㵱ǰĿ��λ�������һ��ͼ��λ�ñ仯
	Aim_Position d_aim_p;
	d_aim_p.cx=aim_observed.cx-new_aim_position_.cx;
	d_aim_p.cy=aim_observed.cy-new_aim_position_.cy;
	d_aim_p.size_x=aim_observed.size_x-new_aim_position_.size_x;
	d_aim_p.size_y=aim_observed.size_y-new_aim_position_.size_y;

	if(abs(d_aim_p.cx)>30||abs(d_aim_p.cy)>30||abs(d_aim_p.size_x)>30||abs(d_aim_p.size_y)>30)
	{
		cout<<"warning: Ŀ���仯�Ĺ۲�ֵ����Ŀ��򽫲������"<<endl;
		return true;
	}

	//2.����Ȩֵ����λ�ñ仯
	aim_weight_*=0.5;	//ʱ��˥������0.5
	d_aim_position_.cx=(d_aim_p.cx*total_w+d_aim_position_.cx*aim_weight_)/(total_w+aim_weight_);
	d_aim_position_.cy=(d_aim_p.cy*total_w+d_aim_position_.cy*aim_weight_)/(total_w+aim_weight_);
	d_aim_position_.size_x=(d_aim_p.size_x*total_w+d_aim_position_.size_x*aim_weight_)/(total_w+aim_weight_);
	d_aim_position_.size_y=(d_aim_p.size_y*total_w+d_aim_position_.size_y*aim_weight_)/(total_w+aim_weight_);

	new_aim_position_.cx+=d_aim_position_.cx;
	new_aim_position_.cy+=d_aim_position_.cy;
	new_aim_position_.size_x+=d_aim_position_.size_x;
	new_aim_position_.size_y+=d_aim_position_.size_y;

	//3.����Ȩֵ�仯
	aim_weight_=(aim_weight_*aim_weight_+total_w*total_w)/(aim_weight_+total_w);

	return true;
}

//���²ο�ͼ��
bool TrackerOnBoVWSURF::updateReferFrame()
{
	//1.���������������ο�֡
	bool re=surf_tracker_.updateBaseFrame();
	if(re==false)
	{
		return false;
	}

	//2.����Ŀ�����ͼ�����Լ��ο�֡Ŀ��λ��
	sub_im_x_=nsub_im_x_;
	sub_im_y_=nsub_im_y_;

	aim_position_=new_aim_position_;

	return true;
}

//���Ƹ��ٽ��
void TrackerOnBoVWSURF::drawTrack(vector<cv::Point2f>& aim_new_point)
{
	//1.����������
	int point_n=aim_new_point.size();
	vector<cv::KeyPoint> draw_point(point_n);

	for(int i=0;i<point_n;i++)
	{
		draw_point[i].response=500;
		draw_point[i].size=5;
		draw_point[i].octave=1;
		draw_point[i].pt.x=aim_new_point[i].x+nsub_im_x_;
		draw_point[i].pt.y=aim_new_point[i].y+nsub_im_y_;
	}

	cv::drawKeypoints(image_in_,draw_point,drawer_track_,CV_RGB(80,160,80),0);

	//2.�����ظ��ٿ�
	cv::rectangle(drawer_track_,cv::Rect(nsub_im_x_+new_aim_position_.cx-new_aim_position_.size_x/2,nsub_im_y_+new_aim_position_.cy-new_aim_position_.size_y/2,new_aim_position_.size_x,new_aim_position_.size_y),CV_RGB(255,255,255),3);

	cv::imshow("tracking result",drawer_track_);
	cv::waitKey(1);
}
