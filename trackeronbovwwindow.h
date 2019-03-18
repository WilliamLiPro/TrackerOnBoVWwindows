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
	void setDatasetPath();	//�������ݼ�·��
	void updateCamera();	//��������ͷ��Ϣ
	void setCamera();		//ѡ������ͷ

	void setFeatureType();	//���ø����������
	void setBoVWpath();		//�����ı�����BoVW·��
	void changeBoVWpath();	//���ݷ���·���޸��ı�����������BoVW·��

	void setSampleInterval();//���ñ���ͼƬ�Ĳ���Ƶ��
	void setSavePath();		//����ͼ�񱣴�·��
	void changeSavePath();	//���ݷ���·���޸��ı�����������ͼ�񱣴�·��

	void runTracking();		//ִ�и���
	void pauseTracking();	//��ͣ����
	void stopTracking();	//ֹͣ����

private:
	Ui::TrackerOnBoVWwindowClass ui;

	vector<QString> camera_name_;	//����ͷ����
	int cam_id_;	//����ͷ���

	QString dataset_path_;	string _dataset_path_;	//���ݼ�·��
	QString feature_type_;	string _feature_type_;	//��������
	QString bovw_path_;	string _bovw_path_;			//BoVW·��

	int tracker_state_;		//Ŀ����ٽ���״̬(0ֹͣ 1���� 2��ͣ 3���Ŀ��)

	//�����߳�
	TrackerOnBoVW tracker_;	//������
	Aim_Position aim_p_;	//Ŀ��λ��
	cv::VideoCapture cap;	//��Ƶ��

	//��ȡ��ǰʱ�䲢ת��Ϊ�ַ���(-�Զ������ӷ�)
	void getCurrentTime(string& current_time);

	//�ֶ����Ŀ��
	int setObjPosition();

	//���и��ٺ���
	int runObjTracking();

	//����·��
	QString save_path_;		//ͼƬ����·��
	int sp_interval_;		//�������
};

#endif // TRACKERONBOVWWINDOW_H
