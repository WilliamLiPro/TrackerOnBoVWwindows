#include "stdafx.h"
#include "cvMatQImageTrans.h"

/*	QImage转cvMat函数

	输入：
	QImage& image_in
	bool share

	输出：
	cv::Mat& image_out
*/
int QImage2cvMat(QImage& image_in,cv::Mat& image_out,bool share)
{
	int data_type=0;	//图像数据类型
	switch (image_in.format())
	{
	case QImage::Format_ARGB32:
	case QImage::Format_RGB32:
	case QImage::Format_ARGB32_Premultiplied:
		data_type=CV_8UC4;
		break;
	case QImage::Format_RGB888:
		data_type=CV_8UC3;
		break;
	case QImage::Format_Indexed8:
		data_type=CV_8UC1;
		break;
	default:
		return -1;
	}

	image_out=cv::Mat(image_in.height(),image_in.width(),data_type,(void*)image_in.bits(),image_in.bytesPerLine());

	if (!share)
	{
		cv::Mat mid_im=image_out;
		image_out=cv::Mat();
		mid_im.copyTo(image_out);
	}
	return 0;
}


/*	cvMat转QImage函数

	输入：
	cv::Mat &image_in
	bool share

	输出：
	QImage& image_out
*/
int cvMat2QImage(cv::Mat& image_in,QImage& image_out,bool share)
{
	if (image_in.type()==CV_8UC1)	//单通道8位图
	{
		image_out=QImage(image_in.cols,image_in.rows,QImage::Format_Indexed8);
		image_out.setColorCount(256);
		
		for (int i = 0; i < 256; i++)
		{
			image_out.setColor(i,qRgb(i,i,i));
		}

		uchar *pSrc=image_in.data;
		uchar *pdst;
		for (int i = 0; i < image_in.rows; i++)
		{
			pdst=image_out.scanLine(i);
			memcpy(pdst,pSrc,image_in.cols);
			pSrc+=image_in.step;
		}
	}
	else if (image_in.type()==CV_8UC3)
	{
		cv::Mat midmat;
		if(share==true)
			midmat=image_in;
		else
		{
			midmat=image_in.clone();
		}
		
		cv::cvtColor(image_in,midmat,CV_BGR2RGB);
		if (share==true)
		{
			image_out=QImage(midmat.data,midmat.cols,midmat.rows,midmat.step,
				QImage::Format_RGB888);
		}
		else
		{
			image_out=QImage(midmat.data,midmat.cols,midmat.rows,midmat.step,
				QImage::Format_RGB888).copy();
		}
	}
	else if (image_in.type()==CV_8UC4)
	{
		cv::Mat midmat;
		if(share==true)
			midmat=image_in;
		else
		{
			midmat=image_in.clone();
		}

		cv::cvtColor(image_in,midmat,CV_BGRA2RGBA);
		if (share==true)
		{
			image_out=QImage(midmat.data,midmat.cols,midmat.rows,midmat.step,
				QImage::Format_ARGB32);
		}
		else
		{
			image_out=QImage(midmat.data,midmat.cols,midmat.rows,midmat.step,
				QImage::Format_ARGB32).copy();
		}
	}
	else
	{
		return -1;
	}

	return 0;
}
