/**********************************
	基于SURF特征跟踪的目标跟踪测试
	
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

	cout<<"摄像头初始化..."<<endl;
	string data_file="E:/物体检测与识别/自编程序/数据集/qnxA1.avi";
	//cv::VideoCapture capture=cv::VideoCapture(data_file);//存储摄像头拍摄的数据指针
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
		capture >> frame_in;//获取图像
		bool get_re=tracker.getNewFrame(frame_in);

		if(get_re==false)//图像采样不正确
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
		cout<<"updateAimPosition 用时:"<<time_t<<"ms"<<endl;
	}
}