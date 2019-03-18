#include "stdafx.h"
#include "widgetpainter.h"

WidgetPainter::WidgetPainter(QWidget *parent):QWidget(parent)
{
	setAutoFillBackground(true);    //允许调整控件背景颜色  
	setPalette(QPalette(Qt::white));

	image = QImage(320,240,QImage::Format_RGB888); // 画布的初始化大小设为400*300，使用3通道8位颜色
	image.fill(Qt::white);
	image_paint=image.copy();

	draw_lock=0;	//绘图标志位，不允许绘图
	finish_=0;		//标记结束置1

	startPoint.setX(-1);startPoint.setY(-1);
	endPoint.setX(-1);endPoint.setY(-1);
}

WidgetPainter::~WidgetPainter()
{
}

//设置矩形位置
void WidgetPainter::setRectandDraw(int left_top_x,int left_top_y,int right_bot_x,int right_bot_y)
{
	//矩形位置
	startPoint.setX(left_top_x);
	startPoint.setY(left_top_y);

	endPoint.setX(right_bot_x);
	endPoint.setY(right_bot_y);

	//绘图
	//paint(image);
}

//获取矩形位置
void WidgetPainter::getRectPosition(int& left_top_x,int& left_top_y,int& right_bot_x,int& right_bot_y)
{
	left_top_x=startPoint.x();
	left_top_y=startPoint.y();

	right_bot_x=endPoint.x();
	right_bot_y=endPoint.y();
}

//更新绘图
void WidgetPainter::updatePaint()
{
	image_paint=image.copy();
	WidgetPainter::paint(image_paint);
}

//复原
void WidgetPainter::resetPaint()
{
	image = QImage(320,240,QImage::Format_RGB888); // 画布的初始化大小设为400*300，使用3通道8位颜色
	image.fill(Qt::white);
	image_paint=image.copy();

	startPoint.setX(-1);startPoint.setY(-1);
	endPoint.setX(-1);endPoint.setY(-1);

	update();
}

//	*********** private	***********
void WidgetPainter::paintEvent(QPaintEvent *) //重绘事件
{
	QPainter painter(this);
	painter.drawImage(0,0,image_paint);
}

void WidgetPainter::paint(QImage &theImage)	//在图像上绘制矩形
{
	if(draw_lock==0)	//不允许绘图
	{
		return;
	}

	QPainter pp(&theImage); //在 theImage 上绘图
	pp.setPen(QColor(255,255,0));
	pp.drawRect(QRect(startPoint,endPoint)); //由起始坐标和终止坐标绘制直线
	update(); //进行更新界面显示，可引起窗口重绘事件，重绘窗口
}

void WidgetPainter::mousePressEvent(QMouseEvent *mEvent)
{
	if(mEvent->button() == Qt::LeftButton)	//左键按下
	{
		startPoint = mEvent->pos();
		endPoint = mEvent->pos();
	}
}
   
void WidgetPainter::mouseMoveEvent(QMouseEvent *mEvent)
{
	if(mEvent->buttons() & Qt::LeftButton)	//左键按下
	{
		endPoint = mEvent->pos();
		paint(image);//绘制图形
	}
}
    
void WidgetPainter::mouseReleaseEvent(QMouseEvent *mEvent)
{
	endPoint = mEvent->pos();
	finish_=1;	//标记结束
}