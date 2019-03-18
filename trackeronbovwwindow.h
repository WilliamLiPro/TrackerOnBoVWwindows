#ifndef TRACKERONBOVWWINDOW_H
#define TRACKERONBOVWWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QCameraInfo>
#include <qfiledialog.h>
#include <qtextcodec.h>
#include <qmessagebox.h>

#include <time.h>
#include "ui_trackeronbovwwindow.h"

#include "TrackerOnBoVW.h"
#include "cvMatQImageTrans.h"

class TrackerOnBoVWwindow : public QMainWindow
{
	Q_OBJECT

public:
	TrackerOnBoVWwindow(QWidget *parent = 0);
	~TrackerOnBoVWwindow();

public slots:
	void setDatasetPath();	//设置数据集路径
	void updateCamera();	//更新摄像头信息
	void setCamera();		//选择摄像头

	void setFeatureType();	//设置跟踪特征类别
	void setBoVWpath();		//根据文本设置BoVW路径
	void changeBoVWpath();	//根据返回路径修改文本，设置设置BoVW路径

	void setSampleInterval();//设置保存图片的采样频率
	void setSavePath();		//设置图像保存路径
	void changeSavePath();	//根据返回路径修改文本，设置设置图像保存路径

	void runTracking();		//执行跟踪
	void pauseTracking();	//暂停跟踪
	void stopTracking();	//停止跟踪

private:
	Ui::TrackerOnBoVWwindowClass ui;

	vector<QString> camera_name_;	//摄像头名称
	int cam_id_;	//摄像头序号

	QString dataset_path_;	string _dataset_path_;	//数据集路径
	QString feature_type_;	string _feature_type_;	//特征类型
	QString bovw_path_;	string _bovw_path_;			//BoVW路径

	int tracker_state_;		//目标跟踪进程状态(0停止 1运行 2暂停 3标记目标)

	//跟踪线程
	TrackerOnBoVW tracker_;	//跟踪器
	Aim_Position aim_p_;	//目标位置
	cv::VideoCapture cap;	//视频流

	//获取当前时间并转换为字符串(-自定义连接符)
	void getCurrentTime(string& current_time);

	//手动标记目标
	int setObjPosition();

	//运行跟踪函数
	int runObjTracking();

	//保存路径
	QString save_path_;		//图片保存路径
	int sp_interval_;		//采样间隔
};

#endif // TRACKERONBOVWWINDOW_H
