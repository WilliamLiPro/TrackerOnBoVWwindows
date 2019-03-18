/***************************************************************
	>���򣺻��ڸ��ʵ�λ�˱仯����������޳�
	>���ߣ���ά��
	>��ϵ��ʽ��248636779@163.com
	
	>����Ҫ�㣺
	>1.����Ȩֵ��С�������λ�˱仯
	>2.���ڱ�׼�������޳�

****************************************************************/
#include "stdafx.h"
#include "poseEstimateKickOutpoint.h"

/*	����Ȩֵ��С�������λ�˱仯
	����ţ�ٷ������С����

	���룺
	ԭʼ����������		vector<cv::Point2f>& origin_p		
	ƥ�����������		vector<cv::Point2f>& match_p
	ƥ��Ȩ��			vector<float>& weight_p

	�����
	λ�˱仯			cv::Mat& pose_cg
	*/
bool weightBaedPoseEstimate(vector<cv::Point2f>& origin_p,vector<cv::Point2f>& match_p,vector<float>& weight_p,cv::Mat& pose_cg)
{
	//1.��ʼ���������
	int point_n=min(origin_p.size(),match_p.size());	//ƥ����������

	if(point_n<3)
	{
		cout<<"warning: ƥ���������٣��޷�����λ�˱仯"<<endl;
		return false;
	}

	//��С����������׼��
	if(pose_cg.rows!=5)
	{
		pose_cg=cv::Mat::zeros(5,1,CV_32FC1);		//�������������ʼ��(wx,wy,wz,dx,dy)
	}

	cv::Mat fun_input(point_n,3,CV_32FC1);				//���������������
	cv::Mat fun_out(point_n,2,CV_32FC1);				//���������������

	float *fun_input_ptr,*fun_out_ptr;
	int data_id=0;	//������������

	for(int i=0;i<point_n;i++)
	{
		fun_input_ptr=fun_input.ptr<float>(i);
		fun_out_ptr=fun_out.ptr<float>(i);

		float wp=sqrt(weight_p[i]);

		fun_input_ptr[0]=origin_p[i].x;
		fun_input_ptr[1]=origin_p[i].y;
		fun_input_ptr[2]=wp;

		fun_out_ptr[0]=(match_p[i].x-origin_p[i].x)*wp;
		fun_out_ptr[1]=(match_p[i].y-origin_p[i].y)*wp;
	}

	//2.������㺯����NewTon����������
	//�����˳�����
	float ex_cond=0;
	for(int i=0;i<fun_input.rows;i++)
	{
		ex_cond+=weight_p[i];
	}
	ex_cond*=0.0001;

	//����λ�˱仯
	vector<void (*)(cv::Mat&,cv::Mat&,float&)> pfuns(2);
	pfuns[0]=&matchPoseChangeX;
	pfuns[1]=&matchPoseChangeY;

	aLeastSquareNewtonSolver(pfuns,0.0001,ex_cond,pose_cg,fun_input,fun_out);

	//cout<<pose_cg<<endl;

	return true;
}

/*	����Ȩֵ��С�������ͼ����תƽ�Ʊ仯
	���ý����������С����

	���룺
	ԭʼ����������		vector<cv::Point2f>& origin_p		
	ƥ�����������		vector<cv::Point2f>& match_p
	ƥ��Ȩ��			vector<float>& weight_p

	�����
	ͼ��仯			cv::Mat& pose_cg
	*/
bool weightBaedImageCgEstimate(vector<cv::Point2f>& origin_p,vector<cv::Point2f>& match_p,vector<float>& weight_p,cv::Mat& pose_cg)
{
	//1.��ʼ���������
	int point_n=min(origin_p.size(),match_p.size());	//ƥ����������

	if(point_n<3)
	{
		cout<<"warning: ƥ���������٣��޷�����λ�˱仯"<<endl;
		return false;
	}

	//��С����������׼��
	if(pose_cg.rows!=4)
	{
		pose_cg=cv::Mat::zeros(4,1,CV_32FC1);		//�������������ʼ��(q,w,dx,dy)
	}


	//2.������С����ƫ����������Է�����
	cv::Mat para_matrix(4,4,CV_32FC1);
	cv::Mat para_b(4,1,CV_32FC1);

	//����ϵ������para_matrix��������
	//a.�����0��ϵ��
	float sum_spp=0,sum_spwp=0,sum_spx=0,sum_spy=0;

	for(int i=0;i<point_n;i++)
	{
		sum_spp+=weight_p[i]*(origin_p[i].x*origin_p[i].x+origin_p[i].y*origin_p[i].y);
		sum_spx+=weight_p[i]*origin_p[i].x;
		sum_spy+=weight_p[i]*origin_p[i].y;
	}

	para_matrix.at<float>(0,0)=sum_spp;
	para_matrix.at<float>(0,1)=sum_spwp;
	para_matrix.at<float>(0,2)=sum_spx;
	para_matrix.at<float>(0,3)=sum_spy;

	//b.�����1��ϵ��(������)
	float sum_spwx=-sum_spy,sum_spwy=sum_spx;

	para_matrix.at<float>(1,0)=sum_spwp;
	para_matrix.at<float>(1,1)=sum_spp;
	para_matrix.at<float>(1,2)=sum_spwx;
	para_matrix.at<float>(1,3)=sum_spwy;

	//c.�����2��ϵ��
	float sum_s=0;

	for(int i=0;i<point_n;i++)
	{
		sum_s+=weight_p[i];
	}

	para_matrix.at<float>(2,0)=sum_spx;
	para_matrix.at<float>(2,1)=-sum_spy;
	para_matrix.at<float>(2,2)=sum_s;
	para_matrix.at<float>(2,3)=0;

	//d.�����3��ϵ��
	para_matrix.at<float>(3,0)=sum_spy;
	para_matrix.at<float>(3,1)=sum_spx;
	para_matrix.at<float>(3,2)=0;
	para_matrix.at<float>(3,3)=sum_s;

	//��������para_b��������
	float sum_spp2=0,sum_spwp2=0,sum_sp2x=0,sum_sp2y=0;

	for(int i=0;i<point_n;i++)
	{
		sum_spp2+=weight_p[i]*(origin_p[i].x*match_p[i].x+origin_p[i].y*match_p[i].y);
		sum_spwp2+=weight_p[i]*(origin_p[i].x*match_p[i].y-origin_p[i].y*match_p[i].x);
		sum_sp2x+=weight_p[i]*match_p[i].x;
		sum_sp2y+=weight_p[i]*match_p[i].y;
	}
	para_b.at<float>(0,0)=sum_spp2;
	para_b.at<float>(1,0)=sum_spwp2;
	para_b.at<float>(2,0)=sum_sp2x;
	para_b.at<float>(3,0)=sum_sp2y;

	//3.��ⷽ��
	cv::solve(para_matrix,para_b,pose_cg);

	//��������£�����ߴ�仯Ϊ0����ҪУ��Ϊ1
	if(pose_cg.at<float>(0)<0.7||pose_cg.at<float>(0)>1.4)
	{
		pose_cg=cv::Mat::zeros(pose_cg.rows,pose_cg.cols,CV_32FC1);
		pose_cg.at<float>(0)=1;
	}

	return true;
}


/*	���ڱ�׼�������޳�

	���룺
	λ�˱仯			cv::Mat& pose_cg
	��׼�����ֵ		float std_th
	ԭʼ����������		vector<cv::Point2f>& origin_p		
	ƥ�����������		vector<cv::Point2f>& match_p

	�����
	�������			vector<float>& d_points
	�ڵ���			vector<bool>& inline_p
	*/
bool kickOutpoint(cv::Mat& pose_cg,float std_th,vector<cv::Point2f>& origin_p,vector<cv::Point2f>& match_p,vector<float>& d_points,vector<bool>& inline_p)
{
	//1.��ʼ���������
	int point_n=min(origin_p.size(),match_p.size());	//ƥ����������

	if(max(pose_cg.rows,pose_cg.cols)<4)
	{
		cout<<"error: in greateProPoseEstimate -> ����λ�˱仯����,������������"<<endl;
		return false;
	}

	if(point_n<=1)
	{
		cout<<"error: in greateProPoseEstimate -> ƥ���Թ���,�޷��޳����"<<endl;
		return false;
	}


	float* pose_cg_ptr=pose_cg.ptr<float>(0);

	//2.ͳ��λ�����
	vector<cv::Point2f> delta_pcg(point_n);		//ÿ��ƥ���λ�ñ仯���
	cv::Point2f aver_pcg;
	aver_pcg.x=0;
	aver_pcg.y=0;

	float delta_x,delta_y; 

	for(int i=0;i<point_n;i++)
	{
		cv::Point2f& orig_p=origin_p[i];
		cv::Point2f& new_p=match_p[i];

		delta_x=(orig_p.x*pose_cg_ptr[0]-orig_p.y*pose_cg_ptr[1]+pose_cg_ptr[2])-new_p.x;
		delta_y=(orig_p.x*pose_cg_ptr[1]+orig_p.y*pose_cg_ptr[0]+pose_cg_ptr[3])-new_p.y;

		delta_pcg[i].x=delta_x;
		delta_pcg[i].y=delta_y;

		aver_pcg=aver_pcg+delta_pcg[i];
	}
	if(point_n>0)
	{
		aver_pcg.x=aver_pcg.x/point_n;
		aver_pcg.y=aver_pcg.y/point_n;
	}

	//3.�������λ�����ı�׼��
	d_points=vector<float>(point_n);
	vector<float>dx_points(point_n);
	vector<float>dy_points(point_n);

	float std_dx=0;
	float std_dy=0;

	for(int i=0;i<point_n;i++)
	{
		float error_pcg_x=delta_pcg[i].x-aver_pcg.x;
		float error_pcg_y=delta_pcg[i].y-aver_pcg.y;
		float error_pcg_x2=error_pcg_x*error_pcg_x;
		float error_pcg_y2=error_pcg_y*error_pcg_y;
		float square_d=error_pcg_x2+error_pcg_y2;

		d_points[i]=sqrt(square_d);
		dx_points[i]=abs(error_pcg_x);
		dy_points[i]=abs(error_pcg_y);

		std_dx+=error_pcg_x2;
		std_dy+=error_pcg_y2;
	}
	if(point_n>0)
	{
		std_dx=sqrt(std_dx/point_n);
		std_dy=sqrt(std_dy/point_n);
	}

	//4.�޳���ֵ��ĵ�
	inline_p=vector<bool>(point_n,true);

	float threshold_x=std_th*std_dx;
	float threshold_y=std_th*std_dy;
	
	for(int i=0;i<point_n;i++)
	{
		if(dx_points[i]>threshold_x||dy_points[i]>threshold_y)		//����ж�����
		{
			inline_p[i]=false;
		}
	}

	return true;
}


/*	��������һ��
	��֪�������꣬��������������õ���һ���ĵ�����

	���룺
	������������		vector<cv::Point2f>& pix_p
	�������			Camera_Para& cam_para

	�����
	��һ����������		vector<cv::Point2f>& cam_p
*/
bool pixCoordtoCameraCoord(vector<cv::Point2f>& pix_p,Camera_Para& cam_para,vector<cv::Point2f>& cam_p)
{
	//0.����׼��
	int point_n=pix_p.size();	//��ĸ���

	float cx=cam_para.cx;
	float cy=cam_para.cy;

	float fx=cam_para.fx;
	float fy=cam_para.fy;

	if(cx<=0||cy<=0||fx<=0||fy<=0)
	{
		cout<<"error: in pixCoordtoCameraCoord -> ������������������0"<<endl;
		return false;
	}

	//1.����ת��
	cam_p.resize(point_n);

	for(int i=0;i<point_n;i++)
	{
		cam_p[i].x=(pix_p[i].x-cx)/fx;
		cam_p[i].y=(pix_p[i].y-cy)/fy;
	}

	return true;
}


/*	������תƽ�ƺ��������λ�ñ仯
	���룺
	��������		cv::Mat& fun_in
	��������		cv::Mat& undt_para

	�����
	�������		float& fun_out
*/
void matchPoseChangeX(cv::Mat& fun_in,cv::Mat& undt_para,float& fun_out)
{
	float* fun_in_ptr=fun_in.ptr<float>(0);
	float* undt_para_ptr=undt_para.ptr<float>(0);

	fun_out=(undt_para_ptr[1]-fun_in_ptr[1]*undt_para_ptr[2])+(fun_in_ptr[0]*undt_para_ptr[1]-fun_in_ptr[1]*undt_para_ptr[0])*undt_para_ptr[3];
	fun_out=fun_out*fun_in_ptr[2];
}

void matchPoseChangeY(cv::Mat& fun_in,cv::Mat& undt_para,float& fun_out)
{
	float* fun_in_ptr=fun_in.ptr<float>(0);
	float* undt_para_ptr=undt_para.ptr<float>(0);

	fun_out=(-undt_para_ptr[0]+fun_in_ptr[0]*undt_para_ptr[2])+(fun_in_ptr[0]*undt_para_ptr[1]-fun_in_ptr[1]*undt_para_ptr[0])*undt_para_ptr[4];
	fun_out=fun_out*fun_in_ptr[2];
}
