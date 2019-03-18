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

	//���þ���λ��
	void setRectandDraw(int left_top_x,int left_top_y,int right_bot_x,int right_bot_y);

	//��ȡ����λ��
	void getRectPosition(int& left_top_x,int& left_top_y,int& right_bot_x,int& right_bot_y);

	//���»�ͼ
	void updatePaint();

	//��ԭ
	void resetPaint();

	QImage image;	//QImage ��������ڱ�����ʾ��ͼ��
	QImage image_paint;	//QImage �������image֮�ϻ��Ʒ�����ʾ
	int draw_lock;	//��ͼ״̬��
	bool finish_;	//��ǽ���

protected:
	void paintEvent(QPaintEvent *);		//�ػ��¼�
	void paint(QImage &theImage);	//����ͼ��
	void mousePressEvent(QMouseEvent *mEvent);
    void mouseMoveEvent(QMouseEvent *mEvent);
    void mouseReleaseEvent(QMouseEvent *mEvent);

private:
	QRgb backColor; //QRgb ��ɫ���󣬴洢 image �ı���ɫ
	QPoint startPoint,endPoint; //��������������������ָ���ǰ����������
};

#endif //WIDGETPAINTER_H