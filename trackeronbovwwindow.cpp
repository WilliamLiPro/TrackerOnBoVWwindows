
#include "stdafx.h"
#include "trackeronbovwwindow.h"

TrackerOnBoVWwindow::TrackerOnBoVWwindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	//������ʼ��
	tracker_.setDrawTrackResult(false);	//ֹͣopencv���Ƹ��ٽ��
	tracker_state_=0;//Ŀ����ٽ���״̬����ʼ��Ϊֹͣ��

	feature_type_=ui.selet_feature->currentText();	//��������
	_feature_type_=feature_type_.toStdString();

	bovw_path_=ui.text_BoVWpath->text();
	QTextCodec *code = QTextCodec::codecForName("gb18030");	//�����������
	_bovw_path_=code->fromUnicode(bovw_path_).data();

	cam_id_=-1;	//����ͷ��ų�ʼ��-1 (��ʾû������ͷ)

	save_path_=ui.text_savePath->text();
	sp_interval_=ui.sample_interval->value();
}

TrackerOnBoVWwindow::~TrackerOnBoVWwindow()
{

}

//********	��Ӧ����	*******

void TrackerOnBoVWwindow::setDatasetPath()
	//�������ݼ�·��
{
	//�������·��
	QString dataset_path_=QFileDialog::getOpenFileName(this, tr("Open Dataset"), ".", tr("Image Files(*.jpg *.jpeg *.png *.bmp *.avi)"));
	if (dataset_path_.isEmpty())	//�ļ�·����Ч
	{
		return;
	}

	//תΪstd string
	QTextCodec *code = QTextCodec::codecForName("gb18030");	//�����������
	_dataset_path_=code->fromUnicode(dataset_path_).data();

	//�޸�Ϊopencv��ȡ��ʽ
	int last_p=_dataset_path_.find_last_of(".");	//���һ����
	int last_t=_dataset_path_.find_last_of("/");	//���һ��б��
	string file_type="0";
	if (last_p>=0&&last_t>=0&&last_p>last_t)
	{
		file_type=_dataset_path_.substr(last_p,_dataset_path_.size());	//�ļ���׺��
	}
	if (file_type!=".jpeg"&&file_type!=".png"&&file_type!=".bmp"&&file_type!=".avi")	//�ļ���׺��Ч
	{
		return;
	}

	if(file_type!=".avi")	//�޸�ͼƬ���Ķ�ȡ��avi�ļ������޸�
	{
		int reset_id=0;
		for (int i = last_t; i < last_p; i++)
		{
			if (_dataset_path_[i]>='0'&&_dataset_path_[i]<='9')	//����ǰ����ٷֺ�
			{
				reset_id=i;
				break;
			}
		}
		_dataset_path_.erase(reset_id);
		_dataset_path_.insert(reset_id,"%06d"+file_type);
		//_dataset_path_.insert(last_p+1,"d");//���ֺ����'d'
	}

	//��ʾ��ʾ��Ϣ
	QString current_msg=QString::fromLocal8Bit("��ǰ���ݼ�·��: \n")+dataset_path_+"\n";
	ui.show_test->append(current_msg);

	current_msg=QString::fromLocal8Bit("���и�������������ť: \n");
	ui.show_test->append(current_msg);
}

void TrackerOnBoVWwindow::updateCamera()
	//��������ͷ��Ϣ
{
	//��ȡ�豸��Ϣ
	QList<QCameraInfo> cameras = QCameraInfo::availableCameras();

	int cam_n=cameras.size();//camera �������ϲ�ѡ��
	int ought_n=cam_n+1;//camera �������ϲ�ѡ��
	int curernt_n=ui.open_camera->count();

	if(curernt_n>cam_n)	//�б��豸�����࣬ɾ������
	{
		for (int i = ought_n; i < curernt_n; i++)
		{
			ui.open_camera->removeItem(i);
		}
	}
	else	//�б��豸�����٣���Ӳ���
	{
		for (int i = curernt_n; i < ought_n; i++)
		{
			ui.open_camera->addItem("");
		}
	}

	//��ȡ����
	camera_name_.resize(cam_n);
	for(int i=0;i<cam_n;i++)
	{
		camera_name_[i]=cameras[i].description();
	}

	//�����ѡ���б���
	for(int i=0;i<cam_n;i++)
	{
		ui.open_camera->setItemText(i+1,camera_name_[i]);
	}
}

void TrackerOnBoVWwindow::setCamera()
	//ѡ������ͷ
{
	QString current_cam=ui.open_camera->currentText(); 

	cam_id_=-1;
	int cam_n=camera_name_.size();
	for(int i=0;i<cam_n;i++)
	{
		if(current_cam==camera_name_[i])
		{
			cam_id_=i;	//��������ͷid
		}
	}

	QString current_msg;
	if(cam_id_==-1)	//�����ݼ�
	{
		current_msg=QString::fromLocal8Bit("��ѡ�����ݼ�\n");
	}
	else			//������ͷ
	{
		current_msg=QString::fromLocal8Bit("��ѡ������ͷ:")+camera_name_[cam_id_]+'\n';
	}
	ui.show_test->append(current_msg);
}

void TrackerOnBoVWwindow::setFeatureType()
	//���ø����������
{
	//��ȡ��������
	feature_type_=ui.selet_feature->currentText();
	_feature_type_=feature_type_.toStdString();

	//��ʾ��ʾ��Ϣ
	QString current_msg=QString::fromLocal8Bit("\n������������Ϊ: \n")+feature_type_+"\n";
	ui.show_test->append(current_msg);

	current_msg=QString::fromLocal8Bit("���и�������������ť: \n");
	ui.show_test->append(current_msg);

}
void TrackerOnBoVWwindow::setBoVWpath()
	//�����ı�����BoVW·��
{
	//��ȡBoVW·��
	bovw_path_=ui.text_BoVWpath->text();

	//תΪstd string
	QTextCodec *code = QTextCodec::codecForName("gb18030");	//�����������
	_bovw_path_=code->fromUnicode(bovw_path_).data();

	//��ʾ��ʾ��Ϣ
	QString current_msg=QString::fromLocal8Bit("BoVW·���޸�Ϊ: \n")+bovw_path_+"\n";
	ui.show_test->append(current_msg);

	current_msg=QString::fromLocal8Bit("��������BoVW��������Ҫ��ʮ�룬�����ĵȴ�.... \n");
	ui.show_test->append(current_msg);
	QCoreApplication::processEvents(QEventLoop::AllEvents,1);

	//��������BoVW
	bool re=tracker_.loadBoVW(_bovw_path_);
	if(re==false)
	{
		current_msg=QString::fromLocal8Bit("BoVW������� \n");
		ui.show_test->append(current_msg);
	}

	current_msg=QString::fromLocal8Bit("���и�������������ť: \n");
	ui.show_test->append(current_msg);
}
void TrackerOnBoVWwindow::changeBoVWpath()
	//���ݷ���·���޸��ı�������BoVW·��
{
	//�������·��
	QString bovw_path=QFileDialog::getOpenFileName(this, tr("Open BoVW"), ".", tr("BoVW(*.dat *.txt)")); 

	//�޸���ʾ·��
	ui.text_BoVWpath->setText(bovw_path);
}

void TrackerOnBoVWwindow::setSampleInterval()
//���ñ���ͼƬ�Ĳ���Ƶ��
{
	sp_interval_=ui.sample_interval->value();
	char number[20];
	itoa(sp_interval_,number,10);

	QString current_msg=QString::fromLocal8Bit(("ͼƬ����Ƶ������Ϊ: "+string(number)+"\n 0��ʾ������ \n").c_str());
	ui.show_test->append(current_msg);
}

void TrackerOnBoVWwindow::setSavePath()	
//����ͼ�񱣴�·��
{
	save_path_=ui.text_savePath->text();
	if (save_path_[save_path_.length()-1]!='/'&&save_path_[save_path_.length()-1]!='\\')
	{
		save_path_=save_path_+'/';
	}
	QString current_msg=QString::fromLocal8Bit("ͼƬ����··���޸�Ϊ: \n")+save_path_+"\n";
	ui.show_test->append(current_msg);
}

void TrackerOnBoVWwindow::changeSavePath()	
//���ݷ���·���޸��ı�����������ͼ�񱣴�·��
{
	//�������·��
	QString save_path=QFileDialog::getExistingDirectory(this, "Save Path", save_path_);

	//�޸���ʾ·��
	ui.text_savePath->setText(save_path);
}

void TrackerOnBoVWwindow::runTracking()
	//ִ�и���
{
	//��������Ƿ���Ч
	QString current_msg;//�����Ϣ
	if(_feature_type_=="")
	{
		current_msg=QString::fromLocal8Bit("����������Ч��������ѡ������");
		ui.show_test->append(current_msg);
		return;
	}
	QFile file(bovw_path_);
	if(_bovw_path_==""||!file.exists())
	{
		current_msg=QString::fromLocal8Bit("BoVW·����Ч����������Ч��BoVW·��");
		ui.show_test->append(current_msg);
		return;
	}

	//1.���Ŀ���ʼλ��
	if(tracker_state_==0)	//ֹͣ״̬��������
	{
		int set_re=setObjPosition();
		if(set_re==-1)	//���ʧ��
		{
			return;
		}

		current_msg=QString::fromLocal8Bit("�������ѡȡ�������� \n");
		ui.show_test->append(current_msg);
	}

	//2.ִ�и���
	if(tracker_state_==3)	//������ǲ������������и���
	{
		runObjTracking();
	}
	else
	{
		tracker_state_=1;
	}
}
void TrackerOnBoVWwindow::pauseTracking()
	//��ͣ����
{
	if(tracker_state_==1)	//ֻ�������˸��ٲ�����Ч��ͣ
	{
		tracker_state_=2;
	}

	QString current_msg=QString::fromLocal8Bit("��ͣ����\n");
	ui.show_test->append(current_msg);
}

void TrackerOnBoVWwindow::stopTracking()
	//ֹͣ����
{
	tracker_state_=0;

	QString current_msg=QString::fromLocal8Bit("ֹͣ����\n");
	ui.show_test->append(current_msg);
}



//		*********	private����	*************

void TrackerOnBoVWwindow::getCurrentTime(string& current_time)
	//��ȡ��ǰʱ�䲢ת��Ϊ�ַ���(-�Զ������ӷ�)
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
	//�ֶ����Ŀ������
{
	//������ͷ
	tracker_state_=3;

	QString current_msg;

	if(cam_id_>=0)	//������ͷ
	{
		cap=cv::VideoCapture(cam_id_);

		current_msg=QString::fromLocal8Bit("������ͷ: ")+camera_name_[cam_id_]+"\n";
		ui.show_test->append(current_msg);
	}
	else
	{
		string _dataset_path_c=_dataset_path_.c_str();
		cap=cv::VideoCapture(_dataset_path_c.c_str());

		current_msg=QString::fromLocal8Bit("�����ݼ�: ")+dataset_path_+"\n";
		ui.show_test->append(current_msg);
	}

	if(!cap.isOpened())
	{
		current_msg=QString::fromLocal8Bit("�����޷�������ͷ�����ݼ�: \n");
		ui.show_test->append(current_msg);
		tracker_state_=0;	//���ٱ����0
		return -1;
	}

	//��ʼ��עĿ��λ��
	int im_num=0;
	int lt_x=0,lt_y=0,rb_x=0,rb_y=0;	//Ŀ��λ��
	cv::Mat cur_image,show_im;
	QImage image_q;


	while(tracker_state_==3)
	{
		if(cam_id_>=0)	//����ͷ��̬��ע
		{
			cap>>cur_image;
			im_num++;
		}
		else	//���ݼ���̬��ע
		{
			if(im_num==0)
			{
				cap>>cur_image;
				++im_num;
			}
		}

		if(cur_image.empty())	//ͼ����������
		{
			continue;
		}

		//��ʽת��
		ui.show_image->draw_lock=1;

		cv::resize(cur_image,cur_image,cv::Size(320,240));	//ͼ��ߴ�任
		if(cur_image.channels()==1)	//�Ҷ�ͼ��Ҫ��չ
		{
			cv::merge(vector<cv::Mat>(3,cur_image),show_im);
		}
		else
		{
			cur_image.copyTo(show_im);
		}
		cvMat2QImage(show_im,ui.show_image->image);			//��ʽת��cvMat->QImage

		ui.show_image->updatePaint();
		QCoreApplication::processEvents(QEventLoop::AllEvents);
		ui.show_image->draw_lock=0;

		//ѡ���Ƿ��ע����
		if(ui.show_image->finish_==1)
		{
			ui.show_image->finish_=0;

			QMessageBox::StandardButton reply;
			reply=QMessageBox::information (NULL,QString::fromLocal8Bit("��ע�ɹ�"),
				QString::fromLocal8Bit("�Ƿ������ע"),
				QMessageBox::Yes | QMessageBox::No);
			if (reply == QMessageBox::Yes)  
			{
				//����Ŀ��λ��
				ui.show_image->getRectPosition(lt_x,lt_y,rb_x,rb_y);

				aim_p_.cx=(lt_x+rb_x)/2;
				aim_p_.cy=(lt_y+rb_y)/2;

				aim_p_.size_x=abs(rb_x-lt_x);
				aim_p_.size_y=abs(rb_y-lt_y);

				//��ע����
				tracker_.getNewFrame(cur_image);
				tracker_.getAimPostion(aim_p_);

				return 0;
			}
			else
			{
				ui.show_image->setRectandDraw(-1,-1,-1,-1);	//������0

				current_msg=QString::fromLocal8Bit("������ע \n");
				ui.show_test->append(current_msg);
			}
		}
	}
}

//���и��ٺ���
int TrackerOnBoVWwindow::runObjTracking()
{
	QString current_msg;//�����Ϣ
	int pic_num=0;		//ͼ�����
	int app_update=0;	//Ӧ�ó���ˢ�µ���ʱ

	//��ʱ�����Ϊ�����ļ�����
	string cur_time;
	getCurrentTime(cur_time);

	//��Ϊִ�и���ʱ��ֱ�ӿ�ʼ����
	tracker_state_=1;
	cv::Mat image_in;
	QImage image_q;

	int empty_num=0;	//ͼ���ȱ����
	while(tracker_state_==1||tracker_state_==2)
	{
		if(tracker_state_==2)	//��ͣ״̬
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

		//1.Ŀ�����
		//����
		cap>>image_in;

		if (image_in.empty())
		{
			cv::waitKey(1);

			++empty_num;
			if (empty_num>50)	//ͼ���ȱ�������࣬������Ч������
			{
				current_msg=QString::fromLocal8Bit("ͼ��/���ݼ��ѽ���\n");
				ui.show_test->append(current_msg);
				break;
			}
			continue;
		}

		//����
		cv::resize(image_in,image_in,cv::Size(320,240));
		tracker_.getNewFrame(image_in);
		tracker_.updateAimPosition(aim_p_);
		pic_num++;

		//2.����Ŀ��λ��
		int lt_x=max(aim_p_.cx-aim_p_.size_x/2,0);
		int lt_y=max(aim_p_.cy-aim_p_.size_y/2,0);
		int rb_x=min(aim_p_.cx+aim_p_.size_x/2,320);
		int rb_y=min(aim_p_.cy+aim_p_.size_y/2,240);

		//��ʽת��
		ui.show_image->draw_lock=1;
		cv::Mat show_im;

		if (image_in.channels()==1)	//��չ�Ҷ�ͼͨ��
		{
			cv::merge(vector<cv::Mat>(3,image_in),show_im);
			cvMat2QImage(show_im,ui.show_image->image);			//��ʽת��cvMat->QImage
		}
		else
		{
			image_in.copyTo(show_im);
			cvMat2QImage(show_im,ui.show_image->image);			//��ʽת��cvMat->QImage
		}

		//��ͼ
		ui.show_image->setRectandDraw(lt_x,lt_y,rb_x,rb_y);	//���þ���λ�ò���ͼ
		ui.show_image->updatePaint();
		QCoreApplication::processEvents(QEventLoop::AllEvents);

		//����ͼƬ
		if (sp_interval_>0&&pic_num%sp_interval_==0)
		{
			QString name(save_path_+cur_time.c_str()+QString::number(pic_num));
			name.resize(name.length()-1);

			ui.show_image->image_paint.save(name+".jpg","JPG",50);
		}
		ui.show_image->draw_lock=0;

		//3.���λ����Ϣ
		current_msg=QString::fromLocal8Bit("Ŀ��λ��: \n")+
			QString::fromLocal8Bit("�������� (")+
			QString::number(aim_p_.cx)+","+QString::number(aim_p_.cy)+
			QString::fromLocal8Bit("), ��� (")+QString::number(aim_p_.size_x)+","+
			QString::number(aim_p_.size_y)+"\n";
		ui.show_test->append(current_msg);

		cv::waitKey(1);
	}

	//�ر�����ͷ
	cap.release();
	tracker_state_=0;	//���ٱ����0

	//��ͼ���
	ui.show_image->resetPaint();

	return 1;
}