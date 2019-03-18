#include "stdafx.h"
#include "widgetpainter.h"

WidgetPainter::WidgetPainter(QWidget *parent):QWidget(parent)
{
	setAutoFillBackground(true);    //��������ؼ�������ɫ  
	setPalette(QPalette(Qt::white));

	image = QImage(320,240,QImage::Format_RGB888); // �����ĳ�ʼ����С��Ϊ400*300��ʹ��3ͨ��8λ��ɫ
	image.fill(Qt::white);
	image_paint=image.copy();

	draw_lock=0;	//��ͼ��־λ���������ͼ
	finish_=0;		//��ǽ�����1

	startPoint.setX(-1);startPoint.setY(-1);
	endPoint.setX(-1);endPoint.setY(-1);
}

WidgetPainter::~WidgetPainter()
{
}

//���þ���λ��
void WidgetPainter::setRectandDraw(int left_top_x,int left_top_y,int right_bot_x,int right_bot_y)
{
	//����λ��
	startPoint.setX(left_top_x);
	startPoint.setY(left_top_y);

	endPoint.setX(right_bot_x);
	endPoint.setY(right_bot_y);

	//��ͼ
	//paint(image);
}

//��ȡ����λ��
void WidgetPainter::getRectPosition(int& left_top_x,int& left_top_y,int& right_bot_x,int& right_bot_y)
{
	left_top_x=startPoint.x();
	left_top_y=startPoint.y();

	right_bot_x=endPoint.x();
	right_bot_y=endPoint.y();
}

//���»�ͼ
void WidgetPainter::updatePaint()
{
	image_paint=image.copy();
	WidgetPainter::paint(image_paint);
}

//��ԭ
void WidgetPainter::resetPaint()
{
	image = QImage(320,240,QImage::Format_RGB888); // �����ĳ�ʼ����С��Ϊ400*300��ʹ��3ͨ��8λ��ɫ
	image.fill(Qt::white);
	image_paint=image.copy();

	startPoint.setX(-1);startPoint.setY(-1);
	endPoint.setX(-1);endPoint.setY(-1);

	update();
}

//	*********** private	***********
void WidgetPainter::paintEvent(QPaintEvent *) //�ػ��¼�
{
	QPainter painter(this);
	painter.drawImage(0,0,image_paint);
}

void WidgetPainter::paint(QImage &theImage)	//��ͼ���ϻ��ƾ���
{
	if(draw_lock==0)	//�������ͼ
	{
		return;
	}

	QPainter pp(&theImage); //�� theImage �ϻ�ͼ
	pp.setPen(QColor(255,255,0));
	pp.drawRect(QRect(startPoint,endPoint)); //����ʼ�������ֹ�������ֱ��
	update(); //���и��½�����ʾ�������𴰿��ػ��¼����ػ洰��
}

void WidgetPainter::mousePressEvent(QMouseEvent *mEvent)
{
	if(mEvent->button() == Qt::LeftButton)	//�������
	{
		startPoint = mEvent->pos();
		endPoint = mEvent->pos();
	}
}
   
void WidgetPainter::mouseMoveEvent(QMouseEvent *mEvent)
{
	if(mEvent->buttons() & Qt::LeftButton)	//�������
	{
		endPoint = mEvent->pos();
		paint(image);//����ͼ��
	}
}
    
void WidgetPainter::mouseReleaseEvent(QMouseEvent *mEvent)
{
	endPoint = mEvent->pos();
	finish_=1;	//��ǽ���
}