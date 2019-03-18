
#include "stdafx.h"
#include "trackeronbovwwindow.h"

TrackerOnBoVWwindow::TrackerOnBoVWwindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	//参数初始化
	tracker_.setDrawTrackResult(false);	//停止opencv绘制跟踪结果
	tracker_state_=0;//目标跟踪进程状态（初始化为停止）

	feature_type_=ui.selet_feature->currentText();	//特征类型
	_feature_type_=feature_type_.toStdString();

	bovw_path_=ui.text_BoVWpath->text();
	QTextCodec *code = QTextCodec::codecForName("gb18030");	//解决中文乱码
	_bovw_path_=code->fromUnicode(bovw_path_).data();

	cam_id_=-1;	//摄像头编号初始化-1 (表示没有摄像头)

	save_path_=ui.text_savePath->text();
	sp_interval_=ui.sample_interval->value();
}

TrackerOnBoVWwindow::~TrackerOnBoVWwindow()
{

}

//********	响应函数	*******

void TrackerOnBoVWwindow::setDatasetPath()
	//设置数据集路径
{
	//获得数据路径
	QString dataset_path_=QFileDialog::getOpenFileName(this, tr("Open Dataset"), ".", tr("Image Files(*.jpg *.jpeg *.png *.bmp *.avi)"));
	if (dataset_path_.isEmpty())	//文件路径无效
	{
		return;
	}

	//转为std string
	QTextCodec *code = QTextCodec::codecForName("gb18030");	//解决中文乱码
	_dataset_path_=code->fromUnicode(dataset_path_).data();

	//修改为opencv读取格式
	int last_p=_dataset_path_.find_last_of(".");	//最后一个点
	int last_t=_dataset_path_.find_last_of("/");	//最后一个斜杠
	string file_type="0";
	if (last_p>=0&&last_t>=0&&last_p>last_t)
	{
		file_type=_dataset_path_.substr(last_p,_dataset_path_.size());	//文件后缀名
	}
	if (file_type!=".jpeg"&&file_type!=".png"&&file_type!=".bmp"&&file_type!=".avi")	//文件后缀无效
	{
		return;
	}

	if(file_type!=".avi")	//修改图片集的读取，avi文件无需修改
	{
		int reset_id=0;
		for (int i = last_t; i < last_p; i++)
		{
			if (_dataset_path_[i]>='0'&&_dataset_path_[i]<='9')	//数字前插入百分号
			{
				reset_id=i;
				break;
			}
		}
		_dataset_path_.erase(reset_id);
		_dataset_path_.insert(reset_id,"%06d"+file_type);
		//_dataset_path_.insert(last_p+1,"d");//数字后插入'd'
	}

	//显示提示信息
	QString current_msg=QString::fromLocal8Bit("当前数据集路径: \n")+dataset_path_+"\n";
	ui.show_test->append(current_msg);

	current_msg=QString::fromLocal8Bit("运行跟踪请点击启动按钮: \n");
	ui.show_test->append(current_msg);
}

void TrackerOnBoVWwindow::updateCamera()
	//更新摄像头信息
{
	//获取设备信息
	QList<QCameraInfo> cameras = QCameraInfo::availableCameras();

	int cam_n=cameras.size();//camera 个数加上不选择
	int ought_n=cam_n+1;//camera 个数加上不选择
	int curernt_n=ui.open_camera->count();

	if(curernt_n>cam_n)	//列表设备数过多，删除部分
	{
		for (int i = ought_n; i < curernt_n; i++)
		{
			ui.open_camera->removeItem(i);
		}
	}
	else	//列表设备数过少，添加部分
	{
		for (int i = curernt_n; i < ought_n; i++)
		{
			ui.open_camera->addItem("");
		}
	}

	//获取名称
	camera_name_.resize(cam_n);
	for(int i=0;i<cam_n;i++)
	{
		camera_name_[i]=cameras[i].description();
	}

	//输出到选择列表中
	for(int i=0;i<cam_n;i++)
	{
		ui.open_camera->setItemText(i+1,camera_name_[i]);
	}
}

void TrackerOnBoVWwindow::setCamera()
	//选择摄像头
{
	QString current_cam=ui.open_camera->currentText(); 

	cam_id_=-1;
	int cam_n=camera_name_.size();
	for(int i=0;i<cam_n;i++)
	{
		if(current_cam==camera_name_[i])
		{
			cam_id_=i;	//设置摄像头id
		}
	}

	QString current_msg;
	if(cam_id_==-1)	//打开数据集
	{
		current_msg=QString::fromLocal8Bit("已选择数据集\n");
	}
	else			//打开摄像头
	{
		current_msg=QString::fromLocal8Bit("已选择摄像头:")+camera_name_[cam_id_]+'\n';
	}
	ui.show_test->append(current_msg);
}

void TrackerOnBoVWwindow::setFeatureType()
	//设置跟踪特征类别
{
	//读取特征名称
	feature_type_=ui.selet_feature->currentText();
	_feature_type_=feature_type_.toStdString();

	//显示提示信息
	QString current_msg=QString::fromLocal8Bit("\n跟踪特征更改为: \n")+feature_type_+"\n";
	ui.show_test->append(current_msg);

	current_msg=QString::fromLocal8Bit("运行跟踪请点击启动按钮: \n");
	ui.show_test->append(current_msg);

}
void TrackerOnBoVWwindow::setBoVWpath()
	//根据文本设置BoVW路径
{
	//读取BoVW路径
	bovw_path_=ui.text_BoVWpath->text();

	//转为std string
	QTextCodec *code = QTextCodec::codecForName("gb18030");	//解决中文乱码
	_bovw_path_=code->fromUnicode(bovw_path_).data();

	//显示提示信息
	QString current_msg=QString::fromLocal8Bit("BoVW路径修改为: \n")+bovw_path_+"\n";
	ui.show_test->append(current_msg);

	current_msg=QString::fromLocal8Bit("正在载入BoVW，可能需要几十秒，请耐心等待.... \n");
	ui.show_test->append(current_msg);
	QCoreApplication::processEvents(QEventLoop::AllEvents,1);

	//正在载入BoVW
	bool re=tracker_.loadBoVW(_bovw_path_);
	if(re==false)
	{
		current_msg=QString::fromLocal8Bit("BoVW载入完成 \n");
		ui.show_test->append(current_msg);
	}

	current_msg=QString::fromLocal8Bit("运行跟踪请点击启动按钮: \n");
	ui.show_test->append(current_msg);
}
void TrackerOnBoVWwindow::changeBoVWpath()
	//根据返回路径修改文本，设置BoVW路径
{
	//获得数据路径
	QString bovw_path=QFileDialog::getOpenFileName(this, tr("Open BoVW"), ".", tr("BoVW(*.dat *.txt)")); 

	//修改显示路径
	ui.text_BoVWpath->setText(bovw_path);
}

void TrackerOnBoVWwindow::setSampleInterval()
//设置保存图片的采样频率
{
	sp_interval_=ui.sample_interval->value();
	char number[20];
	itoa(sp_interval_,number,10);

	QString current_msg=QString::fromLocal8Bit(("图片采样频率设置为: "+string(number)+"\n 0表示不保存 \n").c_str());
	ui.show_test->append(current_msg);
}

void TrackerOnBoVWwindow::setSavePath()	
//设置图像保存路径
{
	save_path_=ui.text_savePath->text();
	if (save_path_[save_path_.length()-1]!='/'&&save_path_[save_path_.length()-1]!='\\')
	{
		save_path_=save_path_+'/';
	}
	QString current_msg=QString::fromLocal8Bit("图片保存路路径修改为: \n")+save_path_+"\n";
	ui.show_test->append(current_msg);
}

void TrackerOnBoVWwindow::changeSavePath()	
//根据返回路径修改文本，设置设置图像保存路径
{
	//获得数据路径
	QString save_path=QFileDialog::getExistingDirectory(this, "Save Path", save_path_);

	//修改显示路径
	ui.text_savePath->setText(save_path);
}

void TrackerOnBoVWwindow::runTracking()
	//执行跟踪
{
	//检查数据是否有效
	QString current_msg;//输出消息
	if(_feature_type_=="")
	{
		current_msg=QString::fromLocal8Bit("特征类型无效：请重新选择特征");
		ui.show_test->append(current_msg);
		return;
	}
	QFile file(bovw_path_);
	if(_bovw_path_==""||!file.exists())
	{
		current_msg=QString::fromLocal8Bit("BoVW路径无效，请输入有效的BoVW路径");
		ui.show_test->append(current_msg);
		return;
	}

	//1.标记目标初始位置
	if(tracker_state_==0)	//停止状态才允许标记
	{
		int set_re=setObjPosition();
		if(set_re==-1)	//标记失败
		{
			return;
		}

		current_msg=QString::fromLocal8Bit("请用鼠标选取跟踪区域 \n");
		ui.show_test->append(current_msg);
	}

	//2.执行跟踪
	if(tracker_state_==3)	//经过标记才允许重新运行跟踪
	{
		runObjTracking();
	}
	else
	{
		tracker_state_=1;
	}
}
void TrackerOnBoVWwindow::pauseTracking()
	//暂停跟踪
{
	if(tracker_state_==1)	//只有运行了跟踪才是有效暂停
	{
		tracker_state_=2;
	}

	QString current_msg=QString::fromLocal8Bit("暂停跟踪\n");
	ui.show_test->append(current_msg);
}

void TrackerOnBoVWwindow::stopTracking()
	//停止跟踪
{
	tracker_state_=0;

	QString current_msg=QString::fromLocal8Bit("停止跟踪\n");
	ui.show_test->append(current_msg);
}



//		*********	private函数	*************

void TrackerOnBoVWwindow::getCurrentTime(string& current_time)
	//获取当前时间并转换为字符串(-自定义连接符)
{
	time_t cur_time;
	time(&cur_time);
	tm* lc_tm=localtime(&cur_time);

	current_time.clear();

	//year
	char c_time[40];
	itoa(lc_tm->tm_year+1900,c_time,10);
	current_time=c_time+string("_");

	//month
	itoa(lc_tm->tm_mon+1,c_time,10);
	current_time=current_time+c_time+string("_");

	//day
	itoa(lc_tm->tm_mday,c_time,10);
	current_time=current_time+c_time+string("_");

	//hour
	itoa(lc_tm->tm_hour,c_time,10);
	current_time=current_time+c_time+string("_");

	//minute
	itoa(lc_tm->tm_min,c_time,10);
	current_time=current_time+c_time+string("_");

	//second
	itoa(lc_tm->tm_sec,c_time,10);
	current_time=current_time+c_time;
}

int TrackerOnBoVWwindow::setObjPosition()
	//手动标记目标区域
{
	//打开摄像头
	tracker_state_=3;

	QString current_msg;

	if(cam_id_>=0)	//打开摄像头
	{
		cap=cv::VideoCapture(cam_id_);

		current_msg=QString::fromLocal8Bit("打开摄像头: ")+camera_name_[cam_id_]+"\n";
		ui.show_test->append(current_msg);
	}
	else
	{
		string _dataset_path_c=_dataset_path_.c_str();
		cap=cv::VideoCapture(_dataset_path_c.c_str());

		current_msg=QString::fromLocal8Bit("打开数据集: ")+dataset_path_+"\n";
		ui.show_test->append(current_msg);
	}

	if(!cap.isOpened())
	{
		current_msg=QString::fromLocal8Bit("错误：无法打开摄像头或数据集: \n");
		ui.show_test->append(current_msg);
		tracker_state_=0;	//跟踪标记置0
		return -1;
	}

	//开始标注目标位置
	int im_num=0;
	int lt_x=0,lt_y=0,rb_x=0,rb_y=0;	//目标位置
	cv::Mat cur_image,show_im;
	QImage image_q;


	while(tracker_state_==3)
	{
		if(cam_id_>=0)	//摄像头动态标注
		{
			cap>>cur_image;
			im_num++;
		}
		else	//数据集静态标注
		{
			if(im_num==0)
			{
				cap>>cur_image;
				++im_num;
			}
		}

		if(cur_image.empty())	//图像数据有误
		{
			continue;
		}

		//格式转换
		ui.show_image->draw_lock=1;

		cv::resize(cur_image,cur_image,cv::Size(320,240));	//图像尺寸变换
		if(cur_image.channels()==1)	//灰度图需要扩展
		{
			cv::merge(vector<cv::Mat>(3,cur_image),show_im);
		}
		else
		{
			cur_image.copyTo(show_im);
		}
		cvMat2QImage(show_im,ui.show_image->image);			//格式转换cvMat->QImage

		ui.show_image->updatePaint();
		QCoreApplication::processEvents(QEventLoop::AllEvents);
		ui.show_image->draw_lock=0;

		//选择是否标注结束
		if(ui.show_image->finish_==1)
		{
			ui.show_image->finish_=0;

			QMessageBox::StandardButton reply;
			reply=QMessageBox::information (NULL,QString::fromLocal8Bit("标注成功"),
				QString::fromLocal8Bit("是否结束标注"),
				QMessageBox::Yes | QMessageBox::No);
			if (reply == QMessageBox::Yes)  
			{
				//计算目标位置
				ui.show_image->getRectPosition(lt_x,lt_y,rb_x,rb_y);

				aim_p_.cx=(lt_x+rb_x)/2;
				aim_p_.cy=(lt_y+rb_y)/2;

				aim_p_.size_x=abs(rb_x-lt_x);
				aim_p_.size_y=abs(rb_y-lt_y);

				//标注跟踪
				tracker_.getNewFrame(cur_image);
				tracker_.getAimPostion(aim_p_);

				return 0;
			}
			else
			{
				ui.show_image->setRectandDraw(-1,-1,-1,-1);	//坐标置0

				current_msg=QString::fromLocal8Bit("继续标注 \n");
				ui.show_test->append(current_msg);
			}
		}
	}
}

//运行跟踪函数
int TrackerOnBoVWwindow::runObjTracking()
{
	QString current_msg;//输出消息
	int pic_num=0;		//图像个数
	int app_update=0;	//应用程序刷新倒计时

	//将时间戳作为保存文件名称
	string cur_time;
	getCurrentTime(cur_time);

	//当为执行跟踪时，直接开始跟踪
	tracker_state_=1;
	cv::Mat image_in;
	QImage image_q;

	int empty_num=0;	//图像空缺次数
	while(tracker_state_==1||tracker_state_==2)
	{
		if(tracker_state_==2)	//暂停状态
		{
			Sleep(10);
			if(app_update==5)
			{
				QCoreApplication::processEvents(QEventLoop::AllEvents);
				app_update=0;
			}
			app_update++;
			continue;
		}

		//1.目标跟踪
		//采样
		cap>>image_in;

		if (image_in.empty())
		{
			cv::waitKey(1);

			++empty_num;
			if (empty_num>50)	//图像空缺次数过多，数据无效，结束
			{
				current_msg=QString::fromLocal8Bit("图像/数据集已结束\n");
				ui.show_test->append(current_msg);
				break;
			}
			continue;
		}

		//跟踪
		cv::resize(image_in,image_in,cv::Size(320,240));
		tracker_.getNewFrame(image_in);
		tracker_.updateAimPosition(aim_p_);
		pic_num++;

		//2.绘制目标位置
		int lt_x=max(aim_p_.cx-aim_p_.size_x/2,0);
		int lt_y=max(aim_p_.cy-aim_p_.size_y/2,0);
		int rb_x=min(aim_p_.cx+aim_p_.size_x/2,320);
		int rb_y=min(aim_p_.cy+aim_p_.size_y/2,240);

		//格式转换
		ui.show_image->draw_lock=1;
		cv::Mat show_im;

		if (image_in.channels()==1)	//扩展灰度图通道
		{
			cv::merge(vector<cv::Mat>(3,image_in),show_im);
			cvMat2QImage(show_im,ui.show_image->image);			//格式转换cvMat->QImage
		}
		else
		{
			image_in.copyTo(show_im);
			cvMat2QImage(show_im,ui.show_image->image);			//格式转换cvMat->QImage
		}

		//绘图
		ui.show_image->setRectandDraw(lt_x,lt_y,rb_x,rb_y);	//设置矩形位置并绘图
		ui.show_image->updatePaint();
		QCoreApplication::processEvents(QEventLoop::AllEvents);

		//保存图片
		if (sp_interval_>0&&pic_num%sp_interval_==0)
		{
			QString name(save_path_+cur_time.c_str()+QString::number(pic_num));
			name.resize(name.length()-1);

			ui.show_image->image_paint.save(name+".jpg","JPG",50);
		}
		ui.show_image->draw_lock=0;

		//3.输出位置信息
		current_msg=QString::fromLocal8Bit("目标位置: \n")+
			QString::fromLocal8Bit("中心坐标 (")+
			QString::number(aim_p_.cx)+","+QString::number(aim_p_.cy)+
			QString::fromLocal8Bit("), 宽高 (")+QString::number(aim_p_.size_x)+","+
			QString::number(aim_p_.size_y)+"\n";
		ui.show_test->append(current_msg);

		cv::waitKey(1);
	}

	//关闭摄像头
	cap.release();
	tracker_state_=0;	//跟踪标记置0

	//绘图变白
	ui.show_image->resetPaint();

	return 1;
}