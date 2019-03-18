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

	//3.���������������ķֲ��������Ŀ�꼸��ģ�Ͳ���
	vector<cv::Point2f> aim_old_point;
	vector<cv::Point2f> aim_new_point;
	vector<int> aim_old_p_id;

	re=updateGeometricErrorPara(matched_old_point,matched_new_point,aim_old_p_id,aim_old_point,aim_new_point,pose_cg);
	if(re=false)
	{
		return false;
	}

	//4.����Ŀ�������ļ��ε÷����ۺϵ÷�
	int point_n=aim_old_point.size();
	vector<float> comp_weight(point_n);		//�ۺϵ÷�
	float dx,dy,geo_w;

	for(int i=0;i<point_n;i++)
	{
		dx=aim_old_point[i].x-aim_position_.cx;
		dy=aim_old_point[i].y-aim_position_.cy;

		computeGeometricWeight(dx,dy,geo_w);

		comp_weight[i]=geo_w*score[aim_old_p_id[i]];
	}

	//5.����Ŀ��λ��
	if(point_n<2)
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
		re=computeAimPosition(aim_old_point,aim_new_point,comp_weight);
		if(re=false)
		{
			return false;
		}
	}

	//����ȫͼ����ϵ�µ�λ��
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

	new_aim_position_.size_x=aim_p.size_x;
	if(aim_p.cx+aim_p.size_x/2>=im_cols_)
	{
		new_aim_position_.size_x=2*(im_cols_-aim_p.cx);
	}
	else if (aim_p.cx-aim_p.size_x/2<0)
	{
		new_aim_position_.size_x=2*aim_p.cx;
	}

	new_aim_position_.size_y=aim_p.size_y;
	if(aim_p.cy+aim_p.size_y/2>=im_rows_)
	{
		new_aim_position_.size_y=2*(im_rows_-aim_p.cy);
	}
	else if (aim_p.cy-aim_p.size_y/2<0)
	{
		new_aim_position_.size_y=2*aim_p.cy;
	}

	//6.���Ƹ��ٽ��
	if (draw_track_result_)
	{
		drawTrack(aim_new_point);
	}

	//7.������������״�����ǲο�֡����
	if (surf_tracker_.tracked_rate_<0.2)	//����������С��0.2
	{
		re=updateReferFrame();
		if(re=false)
		{
			return false;
		}
	}

	return true;
}

/********************** private���� *************************/

//����Ŀ�꼸��ģ��,�������ԭͼ�����ļ���Ȩ��
void TrackerOnBoVWSURF::computeGeometricWeight(float& d_x,float& d_y,float& geo_weight)
{
	geo_weight=geo_b_/(d_x*d_x*geo_xx_+d_x*d_y*geo_xy_+d_y*d_y*geo_yy_+geo_b_);
}

//���������������ķֲ��������Ŀ�꼸��ģ�Ͳ���
bool TrackerOnBoVWSURF::updateGeometricErrorPara(vector<cv::Point2f>& matched_old_point,vector<cv::Point2f>& matched_new_point,vector<int>& aim_old_p_id,vector<cv::Point2f>& aim_old_point,vector<cv::Point2f>& aim_new_point,cv::Mat& pose_cg)
{
	//1.��ѡĿ�귶Χ�ڵ�������
	int point_n=matched_old_point.size();

	int aim_sx,aim_sy,aim_ex,aim_ey;	//Ŀ���ڲο�ͼ�е����귶Χ
	aim_sx=max(0,aim_position_.cx-aim_position_.size_x/2);
	aim_sy=max(0,aim_position_.cy-aim_position_.size_y/2);
	aim_ex=min(320,aim_position_.cx-aim_position_.size_x/2+aim_position_.size_x);
	aim_ey=min(240,aim_position_.cy-aim_position_.size_y/2+aim_position_.size_y);

	aim_old_point.reserve(point_n);
	aim_new_point.reserve(point_n);
	aim_old_p_id.reserve(point_n);

	int aim_pn=0;
	for(int i=0;i<point_n;i++)
	{
		cv::Point2f& old_p=matched_old_point[i];
		cv::Point2f& new_p=matched_new_point[i];

		if(old_p.x>=aim_sx&&old_p.x<=aim_ex&&old_p.y>=aim_sy&&old_p.y<=aim_ey)
		{
			aim_old_point.push_back(old_p);
			aim_new_point.push_back(new_p);
			aim_old_p_id.push_back(aim_pn);
			aim_pn++;
		}
	}

	if(aim_pn<3)	//Ŀ����������٣��޷����������ֲ�
	{
		cout<<"warning: Ŀ���ϵ���������٣��޷����������ֲ�"<<endl;

		return true;
	}

	//2.����Ŀ���и����������λ�ñ仯��������Ŀ�����ļ��ξ���
	//���Ŀ�����ļ��ξ���
	vector<float> cd_x(aim_pn);
	vector<float> cd_y(aim_pn);

	for(int i=0;i<aim_pn;i++)
	{
		cv::Point2f& old_p=aim_old_point[i];

		cd_x[i]=old_p.x-aim_position_.cx;
		cd_y[i]=old_p.y-aim_position_.cy;
	}

	//λ�ñ仯���
	vector<float> delta_x(aim_pn);
	vector<float> delta_y(aim_pn);

	float* pose_cg_ptr=pose_cg.ptr<float>(0);		//4����ͼ��仯

	for(int i=0;i<aim_pn;i++)
	{
		cv::Point2f& old_p=aim_old_point[i];
		cv::Point2f& new_p=aim_new_point[i];

		/*old_p.x-=cam_cx_;		//��һ����������
		old_p.y-=cam_cy_;
		new_p.x-=cam_cx_;
		new_p.y-=cam_cy_;*/

		delta_x[i]=new_p.x-(old_p.x*pose_cg_ptr[0]-old_p.y*pose_cg_ptr[1]+pose_cg_ptr[2]);
		delta_y[i]=new_p.y-(old_p.y*pose_cg_ptr[0]+old_p.x*pose_cg_ptr[1]+pose_cg_ptr[3]);
	}

	//3.�������������ļ���Ȩֵ������Ȩ��ֵ
	vector<float> geo_w(aim_pn);	//����Ȩֵ

	for(int i=0;i<aim_pn;i++)
	{
		computeGeometricWeight(cd_x[i],cd_y[i],geo_w[i]);
	}

	float delta_avx=0,delta_avy=0;	//����Ȩ��ֵ
	float sum_w=0;

	for(int i=0;i<aim_pn;i++)
	{
		delta_avx+=geo_w[i]*delta_x[i];
		delta_avy+=geo_w[i]*delta_y[i];
		sum_w+=geo_w[i];
	}

	delta_avx/=sum_w;
	delta_avy/=sum_w;

	//4.���㷽��������
	//����������
	vector<float> delta_d(aim_pn);
	float delta_dx,delta_dy;

	for(int i=0;i<aim_pn;i++)
	{
		delta_dx=delta_x[i]-delta_avx;
		delta_dy=delta_y[i]-delta_avy;
		delta_d[i]=sqrt(delta_dx*delta_dx+delta_dy*delta_dy);
	}

	//��������ֵ
	float cd_ax=0,cd_ay=0;

	for(int i=0;i<aim_pn;i++)
	{
		cd_ax+=cd_x[i];
		cd_ay+=cd_y[i];
	}

	cd_ax/=aim_pn;
	cd_ay/=aim_pn;

	//5.��ⷽ��
	//������߾���
	cv::Mat func_l(4,4,CV_32FC1);
	float* func_l_p=func_l.ptr<float>(0);

	//�����ұ߾���
	cv::Mat func_r=cv::Mat::zeros(4,1,CV_32FC1);
	float* func_r_p=func_r.ptr<float>(0);

	float sum_cdx4=0,sum_cdx3cdy=0;			//��������
	float sum_cdx2cdy2=0,sum_cdx1cdy3=0;
	float sum_cdy4=0;
	float sum_cdx2=0,sum_cdy2=0,sum_cdxcdy=0;

	float cdx2,cdy2,cdxcdy,
		cdx4,cdx3cdy,cdx2cdy2,cdx1cdy3,cdy4;

	for(int i=0;i<aim_pn;i++)
	{
		cdx2=cd_x[i]*cd_x[i];
		cdy2=cd_y[i]*cd_y[i];
		cdxcdy=cd_x[i]*cd_y[i];

		cdx4=cdx2*cdx2;
		cdx3cdy=cdx2*cdxcdy;
		cdx2cdy2=cdxcdy*cdxcdy;
		cdx1cdy3=cdxcdy*cdy2;
		cdy4=cdy2*cdy2;

		sum_cdx2+=cdx2;
		sum_cdy2+=cdy2;
		sum_cdxcdy+=cdxcdy;
		sum_cdx4+=cdx4;
		sum_cdx3cdy+=cdx3cdy;
		sum_cdx2cdy2+=cdx2cdy2;
		sum_cdx1cdy3+=cdx1cdy3;
		sum_cdy4+=cdy4;

		func_r_p[0]+=cdx2*delta_d[i];
		func_r_p[1]+=cdxcdy*delta_d[i];
		func_r_p[2]+=cdy2*delta_d[i];
		func_r_p[3]+=delta_d[i];
	}

	func_l_p[0]=sum_cdx4;
	func_l_p[1]=sum_cdx3cdy;
	func_l_p[2]=sum_cdx2cdy2;
	func_l_p[3]=sum_cdx2;

	func_l_p[4]=sum_cdx3cdy;
	func_l_p[5]=sum_cdx2cdy2;
	func_l_p[6]=sum_cdx1cdy3;
	func_l_p[7]=sum_cdxcdy;

	func_l_p[8]=sum_cdx2cdy2;
	func_l_p[9]=sum_cdx1cdy3;
	func_l_p[10]=sum_cdy4;
	func_l_p[11]=sum_cdy2;

	func_l_p[12]=sum_cdx2;
	func_l_p[13]=sum_cdxcdy;
	func_l_p[14]=sum_cdy2;
	func_l_p[15]=aim_pn;

	//���㷽�����
	cv::Mat func_sv(4,1,CV_32FC1);
	cv::solve(func_l,func_r,func_sv);

	//6.������
	float* func_sv_p=func_sv.ptr<float>(0);

	geo_xx_=max(func_sv_p[0],0);
	geo_xy_=func_sv_p[1];
	geo_yy_=max(func_sv_p[2],0);
	if (4*geo_xx_*geo_yy_<geo_xy_*geo_xy_)	//�����������Ϊ��
	{
		geo_xy_=sqrt(4*geo_xx_*geo_yy_)*(geo_xy_/abs(geo_xy_));
	}

	geo_b_=func_sv_p[3]+1E-6;

	return true;
}

//�����������ۺ�Ȩ�ظ���Ŀ��λ��
bool TrackerOnBoVWSURF::computeAimPosition(vector<cv::Point2f>& aim_old_point,vector<cv::Point2f>& aim_new_point,vector<float>& comp_weight)
{
	//1.���Ŀ�����תƽ��
	int point_n=aim_old_point.size();

	cv::Mat aim_dt(2,1,CV_32FC1);		//ƽ��
	cv::Mat aim_rro(2,2,CV_32FC1);		//��ת����

	cv::Mat fun_l(2,2,CV_32FC1);		//�������
	cv::Mat fun_r(2,2,CV_32FC1);		//�����ұ�

	float* fun_l_p=fun_l.ptr<float>(0);
	float* fun_r_p=fun_r.ptr<float>(0);

	for(int i=0;i<4;i++)
	{
		fun_l_p[i]=0;
		fun_r_p[i]=0;
	}

	float sum_w=0;		//Ȩֵ֮��
	float sum_wpx=0,sum_wpy=0,sum_wpnx=0,sum_wpny=0;	//���������ļ�Ȩ��

	float wx,wy;

	for(int i=0;i<point_n;i++)
	{
		wx=aim_old_point[i].x*comp_weight[i];
		wy=aim_old_point[i].y*comp_weight[i];
		sum_w+=comp_weight[i];

		sum_wpx+=wx;
		sum_wpy+=wy;

		sum_wpnx+=aim_new_point[i].x*comp_weight[i];
		sum_wpny+=aim_new_point[i].y*comp_weight[i];

		fun_l_p[0]+=wx*aim_old_point[i].x;
		fun_l_p[1]+=wx*aim_old_point[i].y;
		fun_l_p[2]+=wx*aim_old_point[i].y;
		fun_l_p[3]+=wy*aim_old_point[i].y;

		fun_r_p[0]+=wx*aim_new_point[i].x;
		fun_r_p[1]+=wy*aim_new_point[i].x;
		fun_r_p[2]+=wx*aim_new_point[i].y;
		fun_r_p[3]+=wy*aim_new_point[i].y;
	}

	fun_l_p[0]-=sum_wpx*sum_wpx/sum_w;
	fun_l_p[1]-=sum_wpx*sum_wpy/sum_w;
	fun_l_p[2]-=sum_wpy*sum_wpx/sum_w;
	fun_l_p[3]-=sum_wpy*sum_wpy/sum_w;

	fun_r_p[0]-=sum_wpnx*sum_wpx/sum_w;
	fun_r_p[1]-=sum_wpnx*sum_wpy/sum_w;
	fun_r_p[2]-=sum_wpny*sum_wpx/sum_w;
	fun_r_p[3]-=sum_wpny*sum_wpy/sum_w;

	//�����ת����
	cv::solve(fun_l,fun_r,aim_rro);

	//����ƽ��
	float* aim_dt_p=aim_dt.ptr<float>(0);
	float* aim_rro_p=aim_rro.ptr<float>(0);

	aim_dt_p[0]=(sum_wpnx-aim_rro_p[0]*sum_wpx-aim_rro_p[1]*sum_wpy)/sum_w;
	aim_dt_p[1]=(sum_wpny-aim_rro_p[2]*sum_wpx-aim_rro_p[3]*sum_wpy)/sum_w;

	//2.����Ŀ��������Ŀ������Χ
	new_aim_position_.cx=aim_rro_p[0]*aim_position_.cx+aim_rro_p[1]*aim_position_.cy+aim_dt_p[0];
	new_aim_position_.cy=aim_rro_p[2]*aim_position_.cx+aim_rro_p[3]*aim_position_.cy+aim_dt_p[1];

	new_aim_position_.size_x=aim_position_.size_x*aim_rro_p[0]+aim_position_.size_y*aim_rro_p[1];
	new_aim_position_.size_y=aim_position_.size_y*aim_rro_p[3]+aim_position_.size_x*aim_rro_p[2];

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
