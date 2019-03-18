/*********************************************
	cv::Mat��Qt��QImage ת������
	�ο�CSDN���ͣ������޸�
**********************************************/
#include <opencv2\opencv.hpp>
#include <Qimage.h>

/*	QImageתcvMat����
	share=true �����cvMat��QImage����ͼ�����ݣ����ܻ�ı����ݸ�ʽ

	���룺
	QImage& image_in
	bool share

	�����
	cv::Mat& image_out
*/
int QImage2cvMat(QImage& image_in,cv::Mat& image_out,bool share=false);

/*	cvMatתQImage����
	share=true �����QImage��cvMat����ͼ�����ݣ����ܻ�ı����ݸ�ʽ

	���룺
	cv::Mat &image_in

	�����
	QImage& image_out
*/
int cvMat2QImage(cv::Mat& image_in,QImage& image_out,bool share=false);