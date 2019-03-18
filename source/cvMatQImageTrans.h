/*********************************************
	cv::Mat与Qt的QImage 转换函数
	参考CSDN博客，稍作修改
**********************************************/
#include <opencv2\opencv.hpp>
#include <Qimage.h>

/*	QImage转cvMat函数
	share=true 输出的cvMat与QImage共享图像数据，可能会改变数据格式

	输入：
	QImage& image_in
	bool share

	输出：
	cv::Mat& image_out
*/
int QImage2cvMat(QImage& image_in,cv::Mat& image_out,bool share=false);

/*	cvMat转QImage函数
	share=true 输出的QImage与cvMat共享图像数据，可能会改变数据格式

	输入：
	cv::Mat &image_in

	输出：
	QImage& image_out
*/
int cvMat2QImage(cv::Mat& image_in,QImage& image_out,bool share=false);