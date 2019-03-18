/**********************************
	����SURF�������ٵ�Ŀ����ٲ���
	
***********************************/
#include <time.h>
#include "TrackerOnBoVW.h"

void main()
{
	TrackerOnBoVW tracker;
	tracker.loadBoVW();

	Aim_Position aim_p;
	aim_p.cx=100;
	aim_p.cy=100;
	aim_p.size_x=100;
	aim_p.size_y=100;

	cout<<"����ͷ��ʼ��..."<<endl;
	string data_file="E:/��������ʶ��/�Ա����/���ݼ�/qnxA1.avi";
	//cv::VideoCapture capture=cv::VideoCapture(data_file);//�洢����ͷ���������ָ��
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
		capture >> frame_in;//��ȡͼ��
		bool get_re=tracker.getNewFrame(frame_in);

		if(get_re==false)//ͼ���������ȷ
		{
			continue;
		}

		frame_n++;
		if(frame_n==1)
		{
			bool re=tracker.getAimPostion(aim_p);
			if(re==false)
			{
				frame_n=0;
				continue;
			}
		}

		float time_t=clock();
		tracker.updateAimPosition(aim_p);
		time_t = ((float)clock()-time_t);
		cout<<"updateAimPosition ��ʱ:"<<time_t<<"ms"<<endl;
	}
}