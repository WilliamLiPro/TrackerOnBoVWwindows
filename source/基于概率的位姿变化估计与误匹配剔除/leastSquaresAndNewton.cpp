/***************************************************************
	>���ƣ���С�����Լ������Է������Newton�������
	>���ߣ���ά��
	>��ϵ��ʽ��248636779@163.com
	>ʵ����С�����Լ������Է������Newton�������
	>����Ҫ�㣺
	>1.Newton�������
	>1.1 ������ʽ��֪��Newton�������
	>1.2 ��������֪�ķ�������С���˵�Newton�������
	>2.��������С�������

****************************************************************/
#include "stdafx.h"
#include "leastSquaresAndNewton.h"

//	vector��
template<typename T_p> bool normalNewtonIterationBasic(void (*pfun)(vector<T_p>&,vector<T_p>&,vector<T_p>&),int n_src,vector<T_p>& base_para,vector<T_p>& src,T_p ex_cond);
template<typename T_p> bool aLeastSquareNewtonSolverBasic(vector<void (*)(vector<T_p>&,vector<T_p>&,T_p&)> pfuns,T_p d_step,T_p ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out);
template<typename T_p> bool nolinearFunMinistNewtonSolverBasic(void (*pfuns)(vector<T_p>&,vector<T_p>&,T_p&),T_p d_step,T_p ex_cond,cv::Mat& undt_para,cv::Mat& const_para);
template<typename T_p> bool partialDifferential(void (*pfun)(vector<T_p>&,vector<T_p>&,T_p&),T_p d_step,cv::Mat& base_para,cv::Mat& input,cv::Mat& d_fun);
template<typename T_p> bool partial2Differential(void (*pfun)(vector<T_p>&,vector<T_p>&,T_p&),T_p d_step,cv::Mat& base_para,cv::Mat& input,cv::Mat& dd_fun);

//	Mat��
template<typename T_p> bool normalNewtonIterationBasic(void (*pfun)(cv::Mat&,cv::Mat&,cv::Mat&),int n_src,cv::Mat& base_para,cv::Mat& src,T_p ex_cond);
template<typename T_p> bool aLeastSquareNewtonSolverBasic(vector<void (*)(cv::Mat&,cv::Mat&,T_p&)> pfuns,T_p d_step,T_p ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out);
template<typename T_p> bool nolinearFunMinistNewtonSolverBasic(void (*pfuns)(cv::Mat&,cv::Mat&,T_p&),T_p d_step,T_p ex_cond,cv::Mat& undt_para,cv::Mat& const_para);
template<typename T_p> bool partialDifferential(void (*pfun)(cv::Mat&,cv::Mat&,T_p&),T_p d_step,cv::Mat& base_para,cv::Mat& input,cv::Mat& d_fun);
template<typename T_p> bool partial2Differential(void (*pfun)(cv::Mat&,cv::Mat&,T_p&),T_p d_step,cv::Mat& base_para,cv::Mat& input,cv::Mat& dd_fun);

//	1.Newton�������

//	1.1 ������ʽ��֪��Newton�������

bool normalNewtonIteration(void (*pfun)(vector<float>&,vector<float>&,vector<float>&),int n_src,vector<float>& base_para,vector<float>& src,float ex_cond)
{
	bool f_re=normalNewtonIterationBasic(pfun,n_src,base_para,src,ex_cond);
	return f_re;
}

bool normalNewtonIteration(void (*pfun)(vector<double>&,vector<double>&,vector<double>&),int n_src,vector<double>& base_para,vector<double>& src,double ex_cond)
{
	bool f_re=normalNewtonIterationBasic(pfun,n_src,base_para,src,ex_cond);
	return f_re;
}

bool normalNewtonIteration(void (*pfun) (cv::Mat&,cv::Mat&,cv::Mat&),int n_src,cv::Mat& base_para,cv::Mat& src,float ex_cond)
{
	bool f_re=normalNewtonIterationBasic(pfun,n_src,base_para,src,ex_cond);
	return f_re;
}

bool normalNewtonIteration(void (*pfun)(cv::Mat&,cv::Mat&,cv::Mat&),int n_src,cv::Mat& base_para,cv::Mat& src,double ex_cond)
{
	bool f_re=normalNewtonIterationBasic(pfun,n_src,base_para,src,ex_cond);
	return f_re;
}

/*	1.1 ������ʽ��֪��Newton�������(template����)
	���룺

	void (*pfun)(vector<T_p>&,vector<T_p>&,vector<T_p>&)  ����⺯��
	�������������������Ϊ������Ԥ����������������������룬�������

	int n_src					��������������
	vector<T_p>& base_para	������������
	vector<T_p>& src			�����������ֵ
	T_p ex_cond				�����˳��������(���������)

	�����
	vector<T_p>& src	���������
	*/
template<typename T_p>
bool normalNewtonIterationBasic(void (*pfun)(vector<T_p>&,vector<T_p>&,vector<T_p>&),int n_src,vector<T_p>& base_para,vector<T_p>& src,T_p ex_cond)
{
	//1.��ʼ��׼��
	//���ݺ�������������������������ߴ�
	if(n_src!=src.size())
	{
		src.resize(n_src,0);
	}
	vector<T_p> src_origin=src;	//�洢��ʼ��������

	//�����������ݷ�Χ������΢�ֲ���
	T_p d_step=0.01;
	for(int i=0;i<n_src;i++)
	{
		d_step+=src[i]*src[i];
	}
	if(d_step>0.1)
		d_step=0.001;
	else
		d_step/=100;

	//������������
	int data_type=CV_32F;
	if(typeid(T_p)==typeid(double))
		data_type=CV_64F;

	//����pfun�������
	vector<T_p> fun_out_v(n_src,0);
	vector<T_p> fun_out_l(n_src,0);
	vector<T_p> fun_out_u(n_src,0);

	//�������
	T_p error_c,error_bf=10E5;

	//�������ݴ洢����
	cv::Mat func_input(n_src,1,CV_MAKETYPE(data_type,1));
	cv::Mat func_out(n_src,1,CV_MAKETYPE(data_type,1));
	cv::Mat func_partial_d(n_src,n_src,CV_MAKETYPE(data_type,1));

	T_p* p_func_input=(T_p*)func_input.data;
	T_p* p_func_out=(T_p*)func_out.data;
	T_p* p_func_partial_d;


	//2.�������
	int iter_max=n_src*5;
	for(int iter_t=0;iter_t<iter_max;iter_t++)
	{
		//���㵱ǰ�������
		pfun(base_para,src,fun_out_v);	//����ֵ
		for(int i=0;i<n_src;i++)
		{
			p_func_input[i]=src[i];
			p_func_out[i]=fun_out_v[i];
		}

		//����������,�ж���������
		error_c=0;
		for(int i=0;i<n_src;i++)
			error_c+=fun_out_v[i]*fun_out_v[i];
		error_c/=n_src;
		//cout<<"error: "<<error_c<<endl;//test

		if(error_c<ex_cond)
			break;
		else if(error_c>4*error_bf)		//�����󣬻�ԭ
			src=src_origin;
		else if(error_c-error_bf>-ex_cond)	//�����������С�󵼲���
			d_step/=2;

		error_bf=error_c;//�������

		//���㵱ǰλ�õ�ƫ����(ÿ��Ϊ��ͬһ��������ƫ����)
		vector<T_p> tp_input;
		for(int i=0;i<n_src;i++)	//�����仯
		{
			//����low����
			tp_input=src;
			tp_input[i]-=d_step;
			pfun(base_para,tp_input,fun_out_l);

			//����up����
			tp_input=src;
			tp_input[i]+=d_step;
			pfun(base_para,tp_input,fun_out_u);

			//����ƫ����
			p_func_partial_d=func_partial_d.ptr<T_p>(i);
			for(int j=0;j<n_src;j++)
				p_func_partial_d[j]=fun_out_u[j]-fun_out_l[j];

			//cout<<func_partial_d<<endl;//test
		}
		func_partial_d=func_partial_d.t()/(2*d_step);
		//cout<<func_partial_d<<endl;//test

		//���½��
		cv::Mat mid_vec;
		bool so_re=cv::solve(func_partial_d,func_out,mid_vec);
		if(so_re==false)
			break;

		func_input=func_input-mid_vec;

		for(int i=0;i<n_src;i++)
			src[i]=p_func_input[i];
	}

	return true;
}

template<typename T_p> bool normalNewtonIterationBasic(void (*pfun)(cv::Mat&,cv::Mat&,cv::Mat&),int n_src,cv::Mat& base_para,cv::Mat& src,T_p ex_cond)
{
	//1.��ʼ��׼��
	//���������������
	if(src.rows<src.cols)
	{
		src.t();
	}
	//���ݺ�������������������������ߴ�
	if(n_src!=src.rows)
	{
		src.resize(n_src,0);
	}
	cv::Mat src_origin=src.clone();	//�洢��ʼ��������

	//�����������ݷ�Χ������΢�ֲ���
	T_p d_step=0.01;
	T_p* p_src=(T_p*)src.data;
	for(int i=0;i<n_src;i++)
	{
		d_step+=p_src[i]*p_src[i];
	}
	if(d_step>0.1)
		d_step=0.001;
	else
		d_step/=100;

	//������������
	int data_type=CV_32F;
	if(typeid(T_p)==typeid(double))
		data_type=CV_64F;

	//�������
	T_p error_c,error_bf=10E5;

	//�������ݴ洢����
	cv::Mat func_out(n_src,1,CV_MAKETYPE(data_type,1));
	cv::Mat func_partial_d(n_src,n_src,CV_MAKETYPE(data_type,1));

	T_p* p_func_out=(T_p*)func_out.data;
	T_p* p_func_partial_d;

	cv::Mat fun_out_l(n_src,1,CV_MAKETYPE(data_type,1));
	cv::Mat fun_out_u(n_src,1,CV_MAKETYPE(data_type,1));

	T_p* p_fun_out_l=(T_p*)fun_out_l.data;
	T_p* p_fun_out_u=(T_p*)fun_out_u.data;

	//2.�������
	int iter_max=n_src*5;
	for(int iter_t=0;iter_t<iter_max;iter_t++)
	{
		//���㵱ǰ�������
		pfun(base_para,src,func_out);	//����ֵ

		//����������,�ж���������
		error_c=0;
		for(int i=0;i<n_src;i++)
			error_c+=p_func_out[i]*p_func_out[i];
		error_c/=n_src;
		//cout<<"error: "<<error_c<<endl;//test

		if(error_c<ex_cond)
			break;
		else if(error_c>4*error_bf)		//�����󣬻�ԭ
			src=src_origin.clone();
		else if(error_c-error_bf>-ex_cond)	//�����������С�󵼲���
			d_step/=2;

		error_bf=error_c;//�������

		//���㵱ǰλ�õ�ƫ����(ÿ��Ϊ��ͬһ��������ƫ����)
		cv::Mat tp_input=src.clone();
		cv::Mat fun_out_l,fun_out_u;

		T_p* p_tp_input=(T_p*)tp_input.data;
		for(int i=0;i<n_src;i++)	//�����仯
		{
			//����low����
			p_tp_input[i]-=d_step;
			pfun(base_para,tp_input,fun_out_l);
			p_tp_input[i]+=d_step;

			//����up����
			p_tp_input[i]+=d_step;
			pfun(base_para,tp_input,fun_out_u);
			p_tp_input[i]-=d_step;

			//����ƫ����
			p_func_partial_d=func_partial_d.ptr<T_p>(i);
			for(int j=0;j<n_src;j++)
				p_func_partial_d[j]=p_fun_out_u[j]-p_fun_out_l[j];

			//cout<<func_partial_d<<endl;//test
		}
		func_partial_d=func_partial_d.t()/(2*d_step);
		//cout<<func_partial_d<<endl;//test

		//���½��
		cv::Mat mid_vec;
		bool so_re=cv::solve(func_partial_d,func_out,mid_vec);
		if(so_re==false)
			break;

		src=src-mid_vec;
	}

	return true;
}

//	1.2 ��������֪�ķ�������С����Newton�������
bool aLeastSquareNewtonSolver(vector<void (*)(vector<float>&,vector<float>&,float&)> pfuns,float d_step,float ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out)
{
	bool f_re=aLeastSquareNewtonSolverBasic(pfuns,d_step,ex_cond,undt_para,fun_input,obv_out);
	return f_re;
}
bool aLeastSquareNewtonSolver(vector<void (*)(vector<double>&,vector<double>&,double&)> pfuns,double d_step,double ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out)
{
	bool f_re=aLeastSquareNewtonSolverBasic(pfuns,d_step,ex_cond,undt_para,fun_input,obv_out);
	return f_re;
}
bool aLeastSquareNewtonSolver(vector<void (*)(cv::Mat&,cv::Mat&,float&)> pfuns,float d_step,float ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out)
{
	bool f_re=aLeastSquareNewtonSolverBasic(pfuns,d_step,ex_cond,undt_para,fun_input,obv_out);
	return f_re;
}
bool aLeastSquareNewtonSolver(vector<void (*)(cv::Mat&,cv::Mat&,double&)> pfuns,double d_step,double ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out)
{
	bool f_re=aLeastSquareNewtonSolverBasic(pfuns,d_step,ex_cond,undt_para,fun_input,obv_out);
	return f_re;
}

/*	1.2 ��������֪�ķ�������С����Newton�������(template����)
	���룺

	����⺯������ vector<void (*)(vector<T_p>&,vector<T_p>&,T_p&)> pfuns
	�������������������Ϊ���������룬���������������������

	T_p d_step				��ƫ��������
	T_p ex_cond
	cv::Mat& undt_para		������������(��ֵ)
	cv::Mat& fun_input		����������ͬ������Ϊһ�У�
	cv::Mat& obv_out		�����۲�����ͬ��۲���Ϊһ�У�

	�����
	cv::Mat& undt_para		������������

*/
template<typename T_p> bool aLeastSquareNewtonSolverBasic(vector<void (*)(vector<T_p>&,vector<T_p>&,T_p&)> pfuns,T_p d_step,T_p ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out)
{
	//1.��ʼ��׼��
	//��һ��������
	if(fun_input.rows<fun_input.cols)
		fun_input=fun_input.t();

	if(obv_out.rows<obv_out.cols)
		obv_out=obv_out.t();

	if(undt_para.rows<undt_para.cols)
		undt_para=undt_para.t();

	//ͳ�����ݸ���
	int n_obv=min(fun_input.rows,obv_out.rows);	//��Ч�۲���������(ȡ����������۲������еĽ�Сֵ)
	int n_funin=fun_input.cols;					//����ÿ���������ݸ���
	int n_para=undt_para.rows;					//������������
	int n_pfuns=pfuns.size();					//��������

	//Newton��⣬һ�׵�������׵��������ʼ��
	int data_type=CV_32F;
	if(typeid(T_p)==typeid(double))
		data_type=CV_64F;

	cv::Mat diff_mat;	//һ�׵�������
	cv::Mat diff2_mat;	//���׵�������

	cv::Mat sub_diff_mat;	//�Ӻ���һ�׵�������
	cv::Mat sub_diff2_mat;	//�Ӻ������׵�������
	T_p pfunc_out;			//�Ӻ������

	//�����������ԭ��
	cv::Mat undt_para_orig;
	undt_para.convertTo(undt_para_orig,data_type);		//�����������ԭ��

	vector<T_p> vundt_para(n_para);
	T_p* p_para=(T_p*) undt_para.data;		//��������ָ��

	//����ָ��
	T_p* p_obv_out;		//�۲�ֵָ��
	T_p* p_diff_mat;	//һ��ƫ��ָ��

	//�������
	T_p error_c,error_bf=10E6,error_best=10E6;
	T_p d_funout;

	cv::Mat best_result=cv::Mat::zeros(undt_para.rows,undt_para.cols,CV_MAKETYPE(data_type,1));//���Ž��

	//2.�������
	int iter_max=n_para*5;
	for(int iter_t=0;iter_t<iter_max;iter_t++)
	{
		diff_mat=cv::Mat::zeros(n_para,1,CV_MAKETYPE(data_type,1));
		diff2_mat=cv::Mat::zeros(n_para,n_para,CV_MAKETYPE(data_type,1));

		for(int i=0;i<n_para;i++)
			vundt_para[i]=p_para[i];

		//����ÿ��ʵ�����ݵ�ƫ��������ƫ����
		for(int i=0;i<n_obv;i++)
		{
			T_p* p_fun_in=fun_input.ptr<T_p>(i);
			vector<T_p> v_fun_in(n_funin);
			for(int j=0;j<n_funin;j++)
				v_fun_in[j]=p_fun_in[j];

			p_obv_out=obv_out.ptr<T_p>(i);

			//����ÿ��������ƫ����
			for(int j=0;j<n_pfuns;j++)
			{
				partialDifferential(pfuns[j],d_step,fun_input.row(i),undt_para,sub_diff_mat);	//һ�׵���
				partial2Differential(pfuns[j],d_step,fun_input.row(i),undt_para,sub_diff2_mat);	//���׵���

				pfuns[j](v_fun_in,vundt_para,pfunc_out);//�Ӻ������

				//����һ��ƫ��
				//cout<<undt_para<<endl;

				d_funout=pfunc_out-p_obv_out[j];
				diff_mat=diff_mat+sub_diff_mat*d_funout;
				diff2_mat=diff2_mat+sub_diff2_mat*d_funout+sub_diff_mat*sub_diff_mat.t();

				//cout<<"sub_diff2_mat"<<sub_diff2_mat<<endl;//test
			}
		}

		//cout<<"diff_mat"<<diff_mat<<endl;//test
		//cout<<"diff2_mat"<<diff2_mat<<endl;//test

		//����������,�ж���������
		p_diff_mat=(T_p*) diff_mat.data;

		error_c=0;
		for(int i=0;i<n_para;i++)
			error_c+=p_diff_mat[i]*p_diff_mat[i];

		error_c/=n_para;
		//cout<<"error: "<<error_c<<endl;//test

		if(error_c<error_best)
		{
			best_result=undt_para;
			error_best=error_c;
		}

		if(error_c<ex_cond)
			break;
		else if(error_c>4*error_bf)		//�����󣬻�ԭ
			undt_para=undt_para_orig.clone();
		else if(error_c-error_bf>-ex_cond)	//�����������С�󵼲���
			d_step/=2;

		error_bf=error_c;//�������

		//δ֪����Newton����
		diff2_mat=(diff2_mat+diff2_mat.t())/2;//����Գƾ���ļ��㷢ɢ
		cv::Mat mid_vec;
		bool so_re=cv::solve(diff2_mat,diff_mat,mid_vec);
		if(so_re==false)
			break;

		undt_para=undt_para-mid_vec;

		//cout<<"mid_vec"<<endl<<mid_vec<<endl;//test
		//cout<<undt_para<<endl;//test
	}

	undt_para=best_result.clone();

	return true;
}

template<typename T_p> bool aLeastSquareNewtonSolverBasic(vector<void (*)(cv::Mat&,cv::Mat&,T_p&)> pfuns,T_p d_step,T_p ex_cond,cv::Mat& undt_para,cv::Mat& fun_input,cv::Mat& obv_out)
{
	//1.��ʼ��׼��
	//��һ��������
	if(fun_input.rows<fun_input.cols)
		fun_input=fun_input.t();

	if(obv_out.rows<obv_out.cols)
		obv_out=obv_out.t();

	if(undt_para.rows<undt_para.cols)
		undt_para=undt_para.t();

	//ͳ�����ݸ���
	int n_obv=min(fun_input.rows,obv_out.rows);	//��Ч�۲���������(ȡ����������۲������еĽ�Сֵ)
	int n_funin=fun_input.cols;					//����ÿ���������ݸ���
	int n_para=undt_para.rows;					//������������
	int n_pfuns=pfuns.size();					//��������

	//Newton��⣬һ�׵�������׵��������ʼ��
	int data_type=CV_32F;
	if(typeid(T_p)==typeid(double))
		data_type=CV_64F;

	cv::Mat diff_mat;	//һ�׵�������
	cv::Mat diff2_mat;	//���׵�������

	cv::Mat sub_diff_mat;	//�Ӻ���һ�׵�������
	cv::Mat sub_diff2_mat;	//�Ӻ������׵�������
	T_p func_out;			//�Ӻ������

	//�����������ԭ��
	cv::Mat undt_para_orig;
	undt_para.convertTo(undt_para_orig,data_type);		//�����������ԭ��

	T_p* p_para=(T_p*) undt_para.data;		//��������ָ��

	//����ָ��
	T_p* p_obv_out,*p_fun_in;	//�۲�ֵָ��
	T_p* p_diff_mat;			//һ��ƫ��ָ��

	//�������
	T_p error_c,error_bf=10E8,error_best=10E8;
	T_p d_funout;

	cv::Mat best_result=cv::Mat::zeros(undt_para.rows,undt_para.cols,CV_MAKETYPE(data_type,1));//���Ž��

	//��ֹ��ɢ������newton�������岽��
	T_p iter_step=1;

	//2.�������
	int iter_max=n_para*5;
	for(int iter_t=0;iter_t<iter_max;iter_t++)
	{
		diff_mat=cv::Mat::zeros(n_para,1,CV_MAKETYPE(data_type,1));
		diff2_mat=cv::Mat::zeros(n_para,n_para,CV_MAKETYPE(data_type,1));

		//����ÿ��ʵ�����ݵ�ƫ��������ƫ����
		for(int i=0;i<n_obv;i++)
		{
			p_fun_in=fun_input.ptr<T_p>(i);
			p_obv_out=obv_out.ptr<T_p>(i);

			//����ÿ��������ƫ����
			for(int j=0;j<n_pfuns;j++)
			{
				partialDifferential(pfuns[j],d_step,fun_input.row(i),undt_para,sub_diff_mat);	//һ�׵���
				partial2Differential(pfuns[j],d_step,fun_input.row(i),undt_para,sub_diff2_mat);	//���׵���

				pfuns[j](fun_input.row(i),undt_para,func_out);//�Ӻ������

				//����һ��ƫ��
				//cout<<undt_para<<endl;

				d_funout=func_out-p_obv_out[j];
				diff_mat=diff_mat+sub_diff_mat*d_funout;
				diff2_mat=diff2_mat+sub_diff2_mat*d_funout+sub_diff_mat*sub_diff_mat.t();

				//cout<<"diff_mat"<<diff_mat<<endl;//test
			}
		}

		//cout<<"diff_mat"<<diff_mat<<endl;//test
		//cout<<"diff2_mat"<<diff2_mat<<endl;//test

		//����������,�ж���������
		p_diff_mat=(T_p*) diff_mat.data;

		error_c=0;
		for(int i=0;i<n_para;i++)
			error_c+=p_diff_mat[i]*p_diff_mat[i];

		error_c/=n_para;
		//cout<<"error: "<<error_c<<endl;//test
		
		if(error_c<error_best)
		{
			best_result=undt_para;
			error_best=error_c;
		}

		if(error_c<ex_cond)
			break;
		else if(error_c>4*error_bf)		//�����󣬻�ԭ
			undt_para=undt_para_orig.clone();
		else if(error_c-error_bf>-ex_cond)	//�����������С�󵼲���
		{
			d_step/=2;
			iter_step*=error_bf/error_c;
		}

		error_bf=error_c;//�������

		//δ֪����Newton����
		diff2_mat=(diff2_mat+diff2_mat.t())/2;//����Գƾ���ļ��㷢ɢ
		cv::Mat mid_vec;
		bool so_re=cv::solve(diff2_mat,diff_mat,mid_vec);
		if(so_re==false)
			break;

		undt_para=undt_para-iter_step*mid_vec;

		//cout<<"mid_vec"<<endl<<mid_vec<<endl;//test
		//cout<<undt_para<<endl;//test
	}

	undt_para=best_result.clone();

	return true;
}

//	2.һ������Ժ�����С�������Newton�������
bool nolinearFunMinistNewtonSolver(void (*pfun)(vector<float>&,vector<float>&,float&),float d_step,float ex_cond,cv::Mat& undt_para,cv::Mat& const_para)
{
	bool f_re=nolinearFunMinistNewtonSolverBasic(pfun,d_step,ex_cond,undt_para,const_para);

	return f_re;
}

bool nolinearFunMinistNewtonSolver(void (*pfun)(vector<double>&,vector<double>&,double&),double d_step,double ex_cond,cv::Mat& undt_para,cv::Mat& const_para)
{
	bool f_re=nolinearFunMinistNewtonSolverBasic(pfun,d_step,ex_cond,undt_para,const_para);

	return f_re;
}

bool nolinearFunMinistNewtonSolver(void (*pfun)(cv::Mat&,cv::Mat&,float&),float d_step,float ex_cond,cv::Mat& undt_para,cv::Mat& const_para)
{
	bool f_re=nolinearFunMinistNewtonSolverBasic(pfun,d_step,ex_cond,undt_para,const_para);
	return f_re;
}

bool nolinearFunMinistNewtonSolver(void (*pfun)(cv::Mat&,cv::Mat&,double&),double d_step,double ex_cond,cv::Mat& undt_para,cv::Mat& const_para)
{
	bool f_re=nolinearFunMinistNewtonSolverBasic(pfun,d_step,ex_cond,undt_para,const_para);
	return f_re;
}

/*	2.һ������Ժ�����С�������Newton�������
	���룺

	����⺯�� void (*pfun)(vector<T_p>&,vector<T_p>&,T_p&)
	�������������������Ϊ���������룬���������������������

	T_p d_step				��ƫ��������
	T_p ex_cond				��������
	cv::Mat& undt_para		�������Ż�ֵ(��ֵ)
	cv::Mat& const_para		��������

	�����
	cv::Mat& undt_para		�����Ż�ֵ

*/
template<typename T_p> bool nolinearFunMinistNewtonSolverBasic(void (*pfuns)(vector<T_p>&,vector<T_p>&,T_p&),T_p d_step,T_p ex_cond,cv::Mat& undt_para,cv::Mat& const_para)
{
	//1.��ʼ��׼��
	//��һ��������
	if(const_para.rows>const_para.cols)	//��Ϊһ��
		const_para=const_para.t();

	if(undt_para.rows>undt_para.cols)	//��Ϊһ��
		undt_para=undt_para.t();

	//ͳ�����ݸ���
	int n_const=max(const_para.cols,const_para.rows);	//�����������ݸ���
	int n_undt=max(undt_para.rows,undt_para.cols);		//������������

	//Newton��⣬һ�׵�������׵��������ʼ��
	int data_type=CV_32F;
	if(typeid(T_p)==typeid(double))
		data_type=CV_64F;

	cv::Mat diff_mat;	//һ�׵�������
	cv::Mat diff2_mat;	//���׵�������

	//��������
	vector<T_p> v_const(n_const);
	T_p* p_const=(T_p*) const_para.data;
	for(int i=0;i<n_const;i++)
		v_const[i]=p_const[i];

	vector<T_p> v_undt(n_undt);
	T_p* p_undt=(T_p*) undt_para.data;

	//�����������ԭ��
	cv::Mat undt_para_orig;
	undt_para.convertTo(undt_para_orig,data_type);		//�����������ԭ��

	//�������
	T_p error_c,error_bf=10E6,error_origin=0;;

	//��ֹ��ɢ������newton�������岽��
	T_p iter_step=1;


	//2.�������
	//cout<<const_para<<endl;//test
	//cout<<undt_para<<endl;//test

	int iter_max=n_undt*5;
	for(int iter_t=0;iter_t<iter_max;iter_t++)
	{
		//����1��ƫ��
		partialDifferential(pfuns,d_step,const_para,undt_para,diff_mat);

		//����2��ƫ��
		partial2Differential(pfuns,d_step,const_para,undt_para,diff2_mat);

		//cout<<diff_mat<<endl;//test
		//cout<<diff2_mat<<endl;//test

		//��������
		for(int i=0;i<n_undt;i++)
			v_undt[i]=p_undt[i];

		pfuns(v_const,v_undt,error_c);	//Ŀ�꺯��ֵ��Ϊ���

		if(iter_t==0)
			error_origin=error_c;

		if(error_c<ex_cond||abs(error_c-error_bf)<abs(ex_cond))
			break;
		else if(error_c>2*error_origin)		//�����󣬻�ԭ
		{
			undt_para=undt_para_orig;
			d_step/=2;
		}
		else if(error_c-error_bf>-ex_cond)	//�����������С�󵼲���
		{
			d_step/=2;
			iter_step*=error_bf/error_c;
		}

		error_bf=error_c;

		//�����м�����
		cv::Mat middle_vec;
		bool so_re=cv::solve(diff2_mat,diff_mat,middle_vec);
		if(so_re==false)
			break;

		middle_vec=middle_vec*iter_step;

		//���ݵ����������δ֪��
		T_p* p_diff=(T_p*)diff_mat.data;
		T_p* p_mid=(T_p*)middle_vec.data;
		for(int i=0;i<n_undt;i++)
		{
			if(p_diff[i]<0)						//����С��0����
				p_undt[i]+=abs(p_mid[i]);
			else								//��������0����
				p_undt[i]-=abs(p_mid[i]);
		}
		
		//cout<<undt_para<<endl;//test
	}

	return true;
}

template<typename T_p> bool nolinearFunMinistNewtonSolverBasic(void (*pfuns)(cv::Mat&,cv::Mat&,T_p&),T_p d_step,T_p ex_cond,cv::Mat& undt_para,cv::Mat& const_para)
{
	//1.��ʼ��׼��
	//��һ��������
	if(const_para.rows>const_para.cols)	//��Ϊһ��
		const_para=const_para.t();

	if(undt_para.cols>undt_para.rows)	//��Ϊһ��
		undt_para=undt_para.t();

	//ͳ�����ݸ���
	int n_const=max(const_para.cols,const_para.rows);	//�����������ݸ���
	int n_undt=max(undt_para.rows,undt_para.cols);		//������������

	//Newton��⣬һ�׵�������׵��������ʼ��
	int data_type=CV_32F;
	if(typeid(T_p)==typeid(double))
		data_type=CV_64F;

	cv::Mat diff_mat;	//һ�׵�������
	cv::Mat diff2_mat;	//���׵�������

	T_p* p_const=(T_p*) const_para.data;
	T_p* p_undt=(T_p*) undt_para.data;

	//�����������ԭ��
	cv::Mat undt_para_orig;
	undt_para.convertTo(undt_para_orig,data_type);		//�����������ԭ��

	//�������
	T_p error_c,error_bf=10E6,error_origin=0;;

	//��ֹ��ɢ������newton�������岽��
	T_p iter_step=1;


	//2.�������
	//cout<<const_para<<endl;//test
	//cout<<undt_para<<endl;//test

	int iter_max=n_undt*5;
	for(int iter_t=0;iter_t<iter_max;iter_t++)
	{
		//����1��ƫ��
		partialDifferential(pfuns,d_step,const_para,undt_para,diff_mat);

		//����2��ƫ��
		partial2Differential(pfuns,d_step,const_para,undt_para,diff2_mat);

		//cout<<diff_mat<<endl;//test
		//cout<<diff2_mat<<endl;//test

		//��������
		pfuns(const_para,undt_para,error_c);	//Ŀ�꺯��ֵ��Ϊ���

		if(iter_t==0)
			error_origin=error_c;

		if(error_c<ex_cond||abs(error_c-error_bf)<abs(ex_cond))
			break;
		else if(error_c>2*error_origin)		//�����󣬻�ԭ
		{
			undt_para=undt_para_orig.clone();
			d_step/=2;
		}
		else if(error_c-error_bf>-ex_cond)	//�����������С�󵼲���
		{
			d_step/=2;
			iter_step*=error_bf/error_c;
		}

		error_bf=error_c;

		//�����м�����
		cv::Mat middle_vec;
		bool so_re=cv::solve(diff2_mat,diff_mat,middle_vec);
		if(so_re==false)
			break;

		middle_vec=middle_vec*iter_step;

		//���ݵ����������δ֪��
		T_p* p_diff=(T_p*)diff_mat.data;
		T_p* p_mid=(T_p*)middle_vec.data;
		for(int i=0;i<n_undt;i++)
		{
			if(p_diff[i]<0)						//����С��0����
				p_undt[i]+=abs(p_mid[i]);
			else								//��������0����
				p_undt[i]-=abs(p_mid[i]);
		}
		
		//cout<<undt_para<<endl;//test
	}

	return true;
}

/*	���㺯����ƫ����
	���룺

	��΢�ֺ��� void (*pfun)(vector<T_p>&,vector<T_p>&,T_p& )
	�������������������Ϊ������Ԥ�������������������룬�������

	T_p d_step				��ƫ��������
	cv::Mat& base_para		������������
	cv::Mat& input			��������

	�����
	cv::Mat& d_fun			����������������ƫ����
*/
template<typename T_p> bool partialDifferential(void (*pfun)(vector<T_p>&,vector<T_p>&,T_p&),T_p d_step,cv::Mat& base_para,cv::Mat& input,cv::Mat& d_fun)
{
	//1.��ʼ��׼��
	//�������ݸ���
	int n_base=base_para.rows*base_para.cols;	//������������
	int n_input=input.rows*input.cols;			//�����������ݸ���

	//ƫ����������ʼ��
	int data_type=CV_32F;
	if(typeid(T_p)==typeid(double))
		data_type=CV_64F;
	if(d_fun.rows!=n_input||d_fun.cols!=1||d_fun.depth()!=data_type)
		d_fun.create(n_input,1,CV_MAKETYPE(data_type,1));

	//���������������������
	vector<T_p> vec_base(n_base);
	vector<T_p> vec_in(n_input);
	T_p pfun_outl,pfun_outu;		//pfun����������ֱ�����ֵ

	T_p* p_base=(T_p*)base_para.data;
	T_p* p_input=(T_p*)input.data;
	T_p* p_dfun=(T_p*)d_fun.data;

	for(int i=0;i<n_base;i++)
		vec_base[i]=p_base[i];

	for(int i=0;i<n_input;i++)
		vec_in[i]=p_input[i];


	//2.����ƫ����
	vector<T_p> vec_incalcu=vec_in;	//���㺯��ֵ������
	T_p d_step2=2*d_step;

	for(int i=0;i<n_input;i++)
	{
		//���㺯��ֵlow
		vec_incalcu[i]-=d_step;
		pfun(vec_base,vec_incalcu,pfun_outl);
		vec_incalcu[i]+=d_step;

		//���㺯��ֵup
		vec_incalcu[i]+=d_step;
		pfun(vec_base,vec_incalcu,pfun_outu);
		vec_incalcu[i]-=d_step;

		//����ƫ����
		p_dfun[i]=(pfun_outu-pfun_outl)/d_step2;
	}

	return true;
}

template<typename T_p> bool partialDifferential(void (*pfun)(cv::Mat&,cv::Mat&,T_p&),T_p d_step,cv::Mat& base_para,cv::Mat& input,cv::Mat& d_fun)
{
	//1.��ʼ��׼��
	//�������ݸ���
	int n_base=base_para.rows*base_para.cols;	//������������
	int n_input=input.rows*input.cols;			//�����������ݸ���

	//ƫ����������ʼ��
	int data_type=CV_32F;
	if(typeid(T_p)==typeid(double))
		data_type=CV_64F;
	if(d_fun.rows!=n_input||d_fun.cols!=1||d_fun.depth()!=data_type)
		d_fun.create(n_input,1,CV_MAKETYPE(data_type,1));

	//����������ָ��
	T_p pfun_outl,pfun_outu;		//pfun����������ֱ�����ֵ

	T_p* p_base=(T_p*)base_para.data;
	T_p* p_input=(T_p*)input.data;
	T_p* p_dfun=(T_p*)d_fun.data;


	//2.����ƫ����
	cv::Mat in_incalcu=input.clone();	//���㺯��ֵ������
	T_p* p_in_incalcu=(T_p*)in_incalcu.data;
	T_p d_step2=2*d_step;

	for(int i=0;i<n_input;i++)
	{
		//���㺯��ֵlow
		p_in_incalcu[i]-=d_step;
		pfun(base_para,in_incalcu,pfun_outl);
		p_in_incalcu[i]+=d_step;

		//���㺯��ֵup
		p_in_incalcu[i]+=d_step;
		pfun(base_para,in_incalcu,pfun_outu);
		p_in_incalcu[i]-=d_step;

		//����ƫ����
		p_dfun[i]=(pfun_outu-pfun_outl)/d_step2;
	}

	return true;
}

/*	���㺯���Ķ���ƫ����
	���룺

	��΢�ֺ��� void (*pfun)(vector<T_p>&,vector<T_p>&,T_p& )
	�������������������Ϊ������Ԥ�������������������룬�������

	T_p d_step				��ƫ��������
	cv::Mat& base_para		������������
	cv::Mat& input			��������

	�����
	cv::Mat& dd_fun			���������������Ķ���ƫ����
*/
template<typename T_p> bool partial2Differential(void (*pfun)(vector<T_p>&,vector<T_p>&,T_p&),T_p d_step,cv::Mat& base_para,cv::Mat& input,cv::Mat& dd_fun)
{
	//1.��ʼ��׼��
	//�������ݸ���
	int n_base=base_para.rows*base_para.cols;	//������������
	int n_input=input.rows*input.cols;			//�����������ݸ���

	//ƫ����������ʼ��
	int data_type=CV_32F;
	if(typeid(T_p)==typeid(double))
		data_type=CV_64F;
	dd_fun.create(n_input,n_input,CV_MAKETYPE(data_type,1));

	//���������������������
	cv::Mat dpfun_outl(n_input,1,CV_MAKETYPE(data_type,1));		//pfun���������������ƫ����
	cv::Mat dpfun_outu(n_input,1,CV_MAKETYPE(data_type,1));
	cv::Mat input_usel,input_useu;

	input.convertTo(input_usel,input.type());
	input.convertTo(input_useu,input.type());

	T_p d_step2=2*d_step;

	T_p* p_dpfun_outl=(T_p*)dpfun_outl.data;
	T_p* p_dpfun_outu=(T_p*)dpfun_outu.data;
	T_p* p_input_usel=(T_p*)input_usel.data;
	T_p* p_input_useu=(T_p*)input_useu.data;
	T_p* p_ddfun;


	//2.���㲻ͬ��1��ƫ����
	for(int i=0;i<n_input;i++)
	{
		//1��ƫ����low
		p_input_usel[i]-=d_step;
		partialDifferential(pfun,d_step,base_para,input_usel,dpfun_outl);
		p_input_usel[i]+=d_step;

		//1��ƫ����up
		p_input_useu[i]+=d_step;
		partialDifferential(pfun,d_step,base_para,input_useu,dpfun_outu);
		p_input_useu[i]-=d_step;

		//cout<<dpfun_outl<<endl<<dpfun_outu<<endl;		//test

		//2��ƫ����
		p_ddfun=dd_fun.ptr<T_p>(i);
		for(int j=0;j<n_input;j++)
		{
			p_ddfun[j]=(p_dpfun_outu[j]-p_dpfun_outl[j])/d_step2;
		}
	}

	return true;
}

template<typename T_p> bool partial2Differential(void (*pfun)(cv::Mat&,cv::Mat&,T_p&),T_p d_step,cv::Mat& base_para,cv::Mat& input,cv::Mat& dd_fun)
{
	//1.��ʼ��׼��
	//�������ݸ���
	int n_base=base_para.rows*base_para.cols;	//������������
	int n_input=input.rows*input.cols;			//�����������ݸ���

	//ƫ����������ʼ��
	int data_type=CV_32F;
	if(typeid(T_p)==typeid(double))
		data_type=CV_64F;
	dd_fun.create(n_input,n_input,CV_MAKETYPE(data_type,1));

	//���������������������
	cv::Mat dpfun_outl(n_input,1,CV_MAKETYPE(data_type,1));		//pfun���������������ƫ����
	cv::Mat dpfun_outu(n_input,1,CV_MAKETYPE(data_type,1));
	cv::Mat input_usel,input_useu;

	input_usel=input.clone();
	input_useu=input.clone();

	T_p d_step2=2*d_step;

	T_p* p_dpfun_outl=(T_p*)dpfun_outl.data;
	T_p* p_dpfun_outu=(T_p*)dpfun_outu.data;
	T_p* p_input_usel=(T_p*)input_usel.data;
	T_p* p_input_useu=(T_p*)input_useu.data;
	T_p* p_ddfun;


	//2.���㲻ͬ��1��ƫ����
	for(int i=0;i<n_input;i++)
	{
		//1��ƫ����low
		p_input_usel[i]-=d_step;
		partialDifferential(pfun,d_step,base_para,input_usel,dpfun_outl);
		p_input_usel[i]+=d_step;

		//1��ƫ����up
		p_input_useu[i]+=d_step;
		partialDifferential(pfun,d_step,base_para,input_useu,dpfun_outu);
		p_input_useu[i]-=d_step;

		//cout<<dpfun_outl<<endl<<dpfun_outu<<endl;		//test

		//2��ƫ����
		p_ddfun=dd_fun.ptr<T_p>(i);
		for(int j=0;j<n_input;j++)
		{
			p_ddfun[j]=(p_dpfun_outu[j]-p_dpfun_outl[j])/d_step2;
		}
	}

	return true;
}




/*	���Ժ���	*/
bool leastSquaresTest()
{
	//����newton����
	vector<double> base_para;vector<double> fun_result(3,0.01);
	double ex_cond=0.0001;
	normalNewtonIteration(testFunction1,3,base_para,fun_result,ex_cond);
	cout<<"����newton������"<<endl;
	for(int i=0;i<fun_result.size();i++)
		cout<<fun_result[i]<<endl;

	cv::Mat base_para_m(base_para);cv::Mat fun_result_m(fun_result);
	normalNewtonIteration(testFunction1b,3,base_para_m,fun_result_m,ex_cond);
	cout<<"����newton������"<<endl<<fun_result_m<<endl;

	//���㺯���Ķ���ƫ����
	cv::Mat base_para2=cv::Mat::zeros(1,3,CV_64FC1);
	cv::Mat input=cv::Mat::ones(1,3,CV_64FC1);
	cv::Mat dd_fun;

	partialDifferential(testFunction2,0.001,base_para2,input,dd_fun);
	cout<<"һ��΢�ֽ����"<<endl<<dd_fun<<endl;
	partial2Differential(testFunction2,0.001,base_para2,input,dd_fun);
	cout<<"����΢�ֽ����"<<endl<<dd_fun<<endl;

	partialDifferential(testFunction2b,0.001,base_para2,input,dd_fun);
	cout<<"һ��΢�ֽ����"<<endl<<dd_fun<<endl;
	partial2Differential(testFunction2b,0.001,base_para2,input,dd_fun);
	cout<<"����΢�ֽ����"<<endl<<dd_fun<<endl;

	//��С���˵�Newton����
	vector<double> v_base(3,1);
	v_base[1]=1.5;v_base[2]=2;
	vector<double> vfun_in(3,1);

	cv::Mat m_base(v_base,true);
	m_base.at<double>(0)+=0.9;
	m_base.at<double>(1)-=0.8;
	m_base.at<double>(2)+=0.9;

	cv::Mat fun_input(8,3,CV_64FC1);
	cv::Mat fun_out(8,1,CV_64FC1);
	double* p_fun_out=(double*)fun_out.data;

	//cv::randu(fun_input,cv::Mat::zeros(8,3,CV_64FC1),cv::Mat::ones(8,3,CV_64FC1));

	for(int i=0;i<8;i++)
	{
		double* p_fun_in=fun_input.ptr<double>(i);

		for(int j=0;j<3;j++)
		{
			p_fun_in[j]=double(rand())/RAND_MAX;
			vfun_in[j]=p_fun_in[j];
		}

		testFunction3(vfun_in,v_base,p_fun_out[i]);
	}

	vector<void (*)(vector<double>&,vector<double>&,double&)> pfuns(1);
	pfuns[0]=&testFunction3;

	aLeastSquareNewtonSolver(pfuns,0.001,ex_cond,m_base,fun_input,fun_out);
	cout<<"��С����Newton���������"<<endl<<m_base<<endl;

	vector<void (*)(cv::Mat&,cv::Mat&,double&)> pfuns2(1);
	pfuns2[0]=&testFunction3b;
	aLeastSquareNewtonSolver(pfuns2,0.001,ex_cond,m_base,fun_input,fun_out);
	cout<<"��С����Newton���������"<<endl<<m_base<<endl;

	//һ�㺯��newton����
	nolinearFunMinistNewtonSolverBasic(testFunction2,0.001,ex_cond,input,base_para2);
	cout<<"min�����Ժ���Newton���������"<<endl<<input<<endl;
	nolinearFunMinistNewtonSolverBasic(testFunction2b,0.001,ex_cond,input,base_para2);
	cout<<"min�����Ժ���Newton���������"<<endl<<input<<endl;

	return true;
}
void testFunction1(vector<double>& base_para,vector<double>& fun_in,vector<double>& fun_out)
{
	int n_para=3;
	fun_in.resize(3,0);
	fun_out.resize(3,0);
	
	for(int i=0;i<3;i++)
	{
		fun_out[i]=0;
		for(int j=0;j<3;j++)
		{
			fun_out[i]+=(i-j+1)*sin(fun_in[j])-0.2;
		}
	}
}
void testFunction1b(cv::Mat& base_para,cv::Mat& fun_in,cv::Mat& fun_out)
{
	int n_para=3;
	fun_in.resize(3,0);
	fun_out.resize(3,0);
	
	double* p_in=(double*)fun_in.data;
	double* p_out=(double*)fun_out.data;

	for(int i=0;i<3;i++)
	{
		p_out[i]=0;
		for(int j=0;j<3;j++)
		{
			p_out[i]+=(i-j+1)*sin(p_in[j])-0.2;
		}
	}
}

void testFunction2(vector<double>& base_para,vector<double>& fun_in,double& fun_out)
{
	fun_out=0;
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
			fun_out+=(i*j+0.5)*fun_in[i]*fun_in[j];
	}
	/*for(int i=0;i<3;i++)
		fun_out+=(0.5*i+1)*fun_in[i];*/
}
void testFunction2b(cv::Mat& base_para,cv::Mat& fun_in,double& fun_out)
{
	double* p_in=(double*)fun_in.data;
	double* p_base_para=(double*)base_para.data;

	fun_out=0;
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
			fun_out+=(i*j+0.5)*p_in[i]*p_in[j];
	}
	/*for(int i=0;i<3;i++)
		fun_out+=(0.5*i+1)*fun_in[i];*/
}

void testFunction3(vector<double>& base_para,vector<double>& fun_in,double& fun_out)
{
	fun_out=0;
	for(int i=0;i<3;i++)
		fun_out+=base_para[i]*fun_in[i];
}
void testFunction3b(cv::Mat& base_para,cv::Mat& fun_in,double& fun_out)
{
	double* p_in=(double*)fun_in.data;
	double* p_base_para=(double*)base_para.data;

	fun_out=0;
	for(int i=0;i<3;i++)
		fun_out+=p_base_para[i]*p_in[i];
}