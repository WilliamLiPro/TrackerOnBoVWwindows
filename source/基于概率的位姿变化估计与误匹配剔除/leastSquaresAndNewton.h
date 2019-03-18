/***************************************************************
	>���ƣ���С�����Լ������Է������Newton�������
	>���ߣ���ά��
	>��ϵ��ʽ��248636779@163.com
	>ʵ����С�����Լ������Է������Newton�������
	>����Ҫ�㣺
	>1.һ�㷽�̵�Newton�������
	>1.1 ������ʽ��֪��Newton�������
	>1.2 ��������֪�ķ�������С����Newton�������
	>2.��������С�������Newton�������

****************************************************************/

#pragma once

// C++��׼��
#include <iostream>
#include <fstream>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

using namespace std;

//	************************* ��д������ͷ�ļ� ***************************

//	1.Newton�������

//	1.1 ������ʽ��֪��Newton�������
/*	���룺

	void (*pfun)(vector<float>&,vector<float>&,vector<float>&)  ����⺯��
	�������������������Ϊ������Ԥ����������������������룬�������

	int n_src					��������������
	vector<float>& base_para	������������
	vector<float>& src			�����������ֵ
	float ex_cond				�����˳��������(���������)

	�����
	vector<float>& src	���������
	*/
bool normalNewtonIteration(void (*pfun) (vector<float>&,vector<float>&,vector<float>&),int n_src,vector<float>& base_para,vector<float>& src,float ex_cond);
bool normalNewtonIteration(void (*pfun)(vector<double>&,vector<double>&,vector<double>&),int n_src,vector<double>& base_para,vector<double>& src,double ex_cond);
bool normalNewtonIteration(void (*pfun) (cv::Mat&,cv::Mat&,cv::Mat&),int n_src,cv::Mat& base_para,cv::Mat& src,float ex_cond);
bool normalNewtonIteration(void (*pfun)(cv::Mat&,cv::Mat&,cv::Mat&),int n_src,cv::Mat& base_para,cv::Mat& src,double ex_cond);

//	1.2 ��������֪�ķ�������С����Newton�������
bool aLeastSquareNewtonSolver(vector<void (*)(vector<float>&,vector<float>&,float&)> pfuns,float d_step,float ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out);
bool aLeastSquareNewtonSolver(vector<void (*)(vector<double>&,vector<double>&,double&)> pfuns,double d_step,double ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out);
bool aLeastSquareNewtonSolver(vector<void (*)(cv::Mat&,cv::Mat&,float&)> pfuns,float d_step,float ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out);
bool aLeastSquareNewtonSolver(vector<void (*)(cv::Mat&,cv::Mat&,double&)> pfuns,double d_step,double ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out);

//	2.һ������Ժ�����С�������Newton�������
bool nolinearFunMinistNewtonSolver(void (*pfun)(vector<float>&,vector<float>&,float&),float d_step,float ex_cond,cv::Mat& undt_para,cv::Mat& const_para);
bool nolinearFunMinistNewtonSolver(void (*pfun)(vector<double>&,vector<double>&,double&),double d_step,double ex_cond,cv::Mat& undt_para,cv::Mat& const_para);
bool nolinearFunMinistNewtonSolver(void (*pfun)(cv::Mat&,cv::Mat&,float&),float d_step,float ex_cond,cv::Mat& undt_para,cv::Mat& const_para);
bool nolinearFunMinistNewtonSolver(void (*pfun)(cv::Mat&,cv::Mat&,double&),double d_step,double ex_cond,cv::Mat& undt_para,cv::Mat& const_para);

/*	���Ժ���	*/
bool leastSquaresTest();
void testFunction1(vector<double>& base_para,vector<double>& fun_in,vector<double>& fun_out);
void testFunction1b(cv::Mat& base_para,cv::Mat& fun_in,cv::Mat& fun_out);
void testFunction2(vector<double>& base_para,vector<double>& fun_in,double& fun_out);
void testFunction2b(cv::Mat& base_para,cv::Mat& fun_in,double& fun_out);
void testFunction3(vector<double>& base_para,vector<double>& fun_in,double& fun_out);
void testFunction3b(cv::Mat& base_para,cv::Mat& fun_in,double& fun_out);