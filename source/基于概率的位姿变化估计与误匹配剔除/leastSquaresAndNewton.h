/***************************************************************
	>名称：最小二乘以及非线性方程组的Newton迭代求解
	>作者：李维鹏
	>联系方式：248636779@163.com
	>实现最小二乘以及非线性方程组的Newton迭代求解
	>技术要点：
	>1.一般方程的Newton迭代求解
	>1.1 函数形式已知的Newton迭代求解
	>1.2 函数半已知的非线性最小二乘Newton迭代求解
	>2.非线性最小化问题的Newton迭代求解

****************************************************************/

#pragma once

// C++标准库
#include <iostream>
#include <fstream>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

using namespace std;

//	************************* 编写函数的头文件 ***************************

//	1.Newton迭代求解

//	1.1 函数形式已知的Newton迭代求解
/*	输入：

	void (*pfun)(vector<float>&,vector<float>&,vector<float>&)  待求解函数
	输入参数从左至右依次为：函数预留基本参数，待求解量输入，函数输出

	int n_src					函数输出结果个数
	vector<float>& base_para	函数基本参数
	vector<float>& src			函数输出量初值
	float ex_cond				函数退出误差条件(均方误差限)

	输出：
	vector<float>& src	函数输出量
	*/
bool normalNewtonIteration(void (*pfun) (vector<float>&,vector<float>&,vector<float>&),int n_src,vector<float>& base_para,vector<float>& src,float ex_cond);
bool normalNewtonIteration(void (*pfun)(vector<double>&,vector<double>&,vector<double>&),int n_src,vector<double>& base_para,vector<double>& src,double ex_cond);
bool normalNewtonIteration(void (*pfun) (cv::Mat&,cv::Mat&,cv::Mat&),int n_src,cv::Mat& base_para,cv::Mat& src,float ex_cond);
bool normalNewtonIteration(void (*pfun)(cv::Mat&,cv::Mat&,cv::Mat&),int n_src,cv::Mat& base_para,cv::Mat& src,double ex_cond);

//	1.2 函数半已知的非线性最小二乘Newton迭代求解
bool aLeastSquareNewtonSolver(vector<void (*)(vector<float>&,vector<float>&,float&)> pfuns,float d_step,float ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out);
bool aLeastSquareNewtonSolver(vector<void (*)(vector<double>&,vector<double>&,double&)> pfuns,double d_step,double ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out);
bool aLeastSquareNewtonSolver(vector<void (*)(cv::Mat&,cv::Mat&,float&)> pfuns,float d_step,float ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out);
bool aLeastSquareNewtonSolver(vector<void (*)(cv::Mat&,cv::Mat&,double&)> pfuns,double d_step,double ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out);

//	2.一般非线性函数最小化问题的Newton迭代求解
bool nolinearFunMinistNewtonSolver(void (*pfun)(vector<float>&,vector<float>&,float&),float d_step,float ex_cond,cv::Mat& undt_para,cv::Mat& const_para);
bool nolinearFunMinistNewtonSolver(void (*pfun)(vector<double>&,vector<double>&,double&),double d_step,double ex_cond,cv::Mat& undt_para,cv::Mat& const_para);
bool nolinearFunMinistNewtonSolver(void (*pfun)(cv::Mat&,cv::Mat&,float&),float d_step,float ex_cond,cv::Mat& undt_para,cv::Mat& const_para);
bool nolinearFunMinistNewtonSolver(void (*pfun)(cv::Mat&,cv::Mat&,double&),double d_step,double ex_cond,cv::Mat& undt_para,cv::Mat& const_para);

/*	测试函数	*/
bool leastSquaresTest();
void testFunction1(vector<double>& base_para,vector<double>& fun_in,vector<double>& fun_out);
void testFunction1b(cv::Mat& base_para,cv::Mat& fun_in,cv::Mat& fun_out);
void testFunction2(vector<double>& base_para,vector<double>& fun_in,double& fun_out);
void testFunction2b(cv::Mat& base_para,cv::Mat& fun_in,double& fun_out);
void testFunction3(vector<double>& base_para,vector<double>& fun_in,double& fun_out);
void testFunction3b(cv::Mat& base_para,cv::Mat& fun_in,double& fun_out);