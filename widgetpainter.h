#ifndef WIDGETPAINTER_H
#define WIDGETPAINTER_H

#include <QtWidgets/qwidget>
#include <QMouseEvent>
#include <QPoint>
#include <QPainter>

class WidgetPainter : public QWidget
{
	Q_OBJECT

public:
	WidgetPainter(QWidget *parent = 0);
	~WidgetPainter();

	//设置矩形位置
	void setRectandDraw(int left_top_x,int left_top_y,int right_bot_x,int right_bot_y);

	//获取矩形位置
	void getRectPosition(int& left_top_x,int& left_top_y,int& right_bot_x,int& right_bot_y);

	//更新绘图
	void updatePaint();

	//复原
	void resetPaint();

	QImage image;	//QImage 类对象，用于保存显示的图像
	QImage image_paint;	//QImage 类对象，在image之上绘制方框并显示
	int draw_lock;	//绘图状态锁
	bool finish_;	//标记结束

protected:
	void paintEvent(QPaintEvent *);		//重绘事件
	void paint(QImage &theImage);	//绘制图像
	void mousePressEvent(QMouseEvent *mEvent);
    void mouseMoveEvent(QMouseEvent *mEvent);
    void mouseReleaseEvent(QMouseEvent *mEvent);

private:
	QRgb backColor; //QRgb 颜色对象，存储 image 的背景色
	QPoint startPoint,endPoint; //定义两个坐标对象存放鼠标指针的前后两个坐标
};

#endif //WIDGETPAINTER_H