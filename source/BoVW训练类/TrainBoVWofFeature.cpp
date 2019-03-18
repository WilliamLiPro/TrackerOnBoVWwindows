/*************************************************
	>BoVW of Different Kind of Features

	>programmed by 李维鹏
	>2016.5.13

	>1.基于opencv库的特征采集
	>2.采用k-means算法的特征聚类，构建不同类特征的BoVW
	>3.基于欧氏距离的特征索引

*************************************************/
#include "stdafx.h"
#include "TrainBoVWofFeature.h"

/*********************  public函数  **********************/

/* 类初始化*/
TrainBoVWofFeature::TrainBoVWofFeature(string feature_type,string descriptor_type)
{
	cout<<"************* All right reserved by Author 李维鹏 ***************"<<endl;
	cout<<"欢迎使用：TrainBoVWofFeature 类"<<endl;
	cout<<"本程序采用m层k分支树构建不同类型特征视觉词典"<<endl;
	cout<<"特征类型为"<<feature_type<<endl;

	//数据初始化：默认视觉词典大小
	bag_of_visual_word_.tree_levels=5;
	bag_of_visual_word_.branches=4;

	//特征检测器初始化
	cv::initModule_nonfree();		//允许调用SURF和SIFT
	feature_type_=feature_type;
	feature_detector_=cv::FeatureDetector::create(feature_type);

	//特征描述器初始化
	descriptor_type_=descriptor_type;
	feature_extractor_=cv::DescriptorExtractor::create(feature_type);

	//特征向量存储器初始化
	samples_of_feature_.reserve(100000);

	//是否绘制采样结果
	draw_keysampling_=false;
}

/*	特征采集与BoVW计算
	输入:
	data_type	0为摄像头采集；1为相片集
	camera_id	启动的摄像头序号*/
bool TrainBoVWofFeature::getSamplesOfFeature(int data_type,int camera_id)
{
	cv::VideoCapture capture;
	if(data_type==0)//选择摄像头
	{
		//初始化摄像头
		cout<<"摄像头初始化..."<<endl;
		capture=cv::VideoCapture(camera_id);//存储摄像头拍摄的数据指针
		if(!capture.isOpened()) // 打开摄像头
		{
			MessageBox(0, L"无法打开摄像头: 请尝试其他摄像头",L"error", MB_ICONERROR);
			return -1;
		}
	}
	if(data_type==1)//选择图片集
	{
		cout<<"请输入图片路径 (eg.img_%02d.jpg)"<<endl;
		string picture_file_path;
		cin>>picture_file_path;
		cout<<"打开文件... "<<picture_file_path<<endl;

		capture=cv::VideoCapture(picture_file_path.c_str());
		if(!capture.isOpened()) // 打开图片路径
		{
			picture_file_path="E:/控制与系统/SLAM/KITTI数据集/data_odometry_gray/00/image_0/%06d.png";
			capture=cv::VideoCapture(picture_file_path.c_str());
			if(!capture.isOpened()) // 打开图片路径
			{
				MessageBox(0, L"无法打开指定文件",L"error", MB_ICONERROR);
				return -1;
			}
		}
	}

	//选择是否继续上一次的采样
	int result=MessageBox(0, L"是否继续上一次的采样？",L"选择", MB_YESNO);
	string feature_nm=feature_type_+" samples.dat";
	string file_name="feature samples/"+feature_nm;
	ios_base::openmode open_mode;
	int lines_of_file=0;
	if(result==IDYES)
	{
		//不覆盖上一次的采样结果
		open_mode=ios::app;
		//读取文件行数
		ifstream infile(file_name);
		if(!infile)//文件不存在
		{
			file_name="../"+file_name;
			infile.open(file_name);
			if(!infile.is_open())
				cout<<"文件打不开:"<<file_name<<endl;
		}
		string data;
		while(!infile.eof())
		{
			getline( infile, data );//获取一行数据
			if(data.size()==0)
			{
				//空行
				continue;
			}
			lines_of_file++;//统计行数
		}
	}
	else
	{
		//覆盖上一次的采样结果
		open_mode=ios::trunc;
	}
	ofstream outfile(file_name,open_mode);
	if(!outfile)//文件不存在
	{
		file_name="../"+file_name;
		outfile.open(file_name);
	}

	//开始读取图像
	cout<<"请按任意键退出采样"<<endl;
	Sleep(500);
	cv::Mat frame_in;	//存储摄像头截取的图像
	while(1)
	{
		capture >> frame_in;//获取图像
		bool catch_result=catchFrame(frame_in);//图像降采样
		if(catch_result==false)//图像采样不正确
		{
			continue;
		}

		bool ft_result=detectFeature();//特征采样主函数
		if(ft_result==false)//图片没有有效特征
		{
			continue;
		}

		int total_sp=samples_of_feature_.size();	//特征总数
		cout<<"当前总的特征数目为"<<total_sp+lines_of_file<<endl;

		if(cv::waitKey(5)>0||total_sp+lines_of_file>1000000)//特征数目过多，强制退出
		{
			break;
		}
	}

	//存储采样结果
	int feature_n=samples_of_feature_.size();
	for(int i=0;i<feature_n;i++)
	{
		int feature_l=samples_of_feature_[i].size();
		for(int j=0;j<feature_l;j++)
		{
			outfile<<samples_of_feature_[i][j]<<" ";
		}
		outfile<<"\n";
	}
}

//设置特征类别
bool TrainBoVWofFeature::setFeatureType(string feature_type,string descriptor_type)
{
	//feature_detector_->~FeatureDetector();
	//feature_extractor_->~DescriptorExtractor();

	feature_detector_=cv::FeatureDetector::create(feature_type);
	feature_extractor_=cv::DescriptorExtractor::create(descriptor_type);

	feature_type_=feature_type;
	descriptor_type_=descriptor_type;

	return true;
}

//设置k-d树的层数以及分支数
bool TrainBoVWofFeature::setBoVWlevelsAndBranches(int tree_levels,int branches)
{
	//判断输入是否有效
	if(tree_levels<2||tree_levels>12)
	{
		cout<<"error: 输入K-D树层数不在有效范围内 2~12"<<endl;
		return false;
	}
	if(branches<2||branches>10)
	{
		cout<<"error: 输入K-D树分支数不在有效范围内 2~10"<<endl;
		return false;
	}

	bag_of_visual_word_.tree_levels=tree_levels;
	bag_of_visual_word_.branches=branches; 

	return true;
}

//BoVW训练函数
//输入：tree_levels树的层数,branches每层分支树
bool TrainBoVWofFeature::buildBagofVisvalWord()
{
	//选择是否训练BagofVisvalWord
	int result=MessageBox(0, L"是否开始训练视觉词典",L"选择", MB_YESNO);
	if(result==IDNO)//退出buildBagofVisvalWord
	{
		cout<<"退出视觉词典程序..."<<endl;
		Sleep(500);
		return false;
	}

	cout<<"词典构造函数，BoVW训练结果存于工程目录：BoVW 文件夹下"<<endl;
	cout<<"文件名："<<feature_type_<<" BoVW.dat"<<endl;

	//采用k-means聚类计算BoVW

	//0.载入samples_of_feature_
	samples_of_feature_.clear();
	string file_nm=feature_type_+" samples.dat";
	string file_name_in="feature samples/"+file_nm;
	cout<<"正在打开文件: "<<file_name_in<<" ..."<<endl;
	ifstream infile(file_name_in);
	//开始写入数据
	if(!infile)//文件不存在
	{
		file_name_in="../"+file_name_in;
		infile.open(file_name_in);
		if(!infile.is_open())
		{
			cout<<"无法打开文件:"<<file_name_in<<endl;
			return false;
		}
	}
	string file_data;//一行的文字
	vector<float> input_data;//输入的数字
	input_data.reserve(128);
	cout<<"正在获取特征样本..."<<endl;
	int feature_nums=0;
	while(!infile.eof())//一直读到文件末尾
	{
		getline( infile, file_data );	//获取一行数据
		int data_l=file_data.size();	//数据长度
		if(data_l==0)
		{
			//空行
			continue;
		}
		//获取一行数据的空格数
		vector<int> space_position;
		space_position.reserve(128);
		for(int i=0;i<data_l;i++)	//搜索space的位置
		{
			if(file_data[i]==' ')
				space_position.push_back(i);
		}
		int space_position_sz=space_position.size();
		input_data.resize(space_position_sz);

		input_data[0]=atof(file_data.substr(0,space_position[0]).c_str());	//特征数据

		for(int i=1;i<space_position_sz;i++)
		{
			input_data[i]=atof(file_data.substr(space_position[i-1]+1,space_position[i]).c_str());	//特征数据
		}
		samples_of_feature_.push_back(input_data);

		feature_nums++;
		if(feature_nums%1000==0)
		{
			cout<<"获取第 "<<feature_nums<<" 个特征样本"<<endl;
		}
	}
	cout<<"特征样本加载完毕"<<endl;
	
	//1.检查输入层数和分支树是否小于样本数
	int tree_levels,branches;
	tree_levels=bag_of_visual_word_.tree_levels;
	branches=bag_of_visual_word_.branches;

	int total_center_nums=1;
	for(int i=0;i<tree_levels;i++)
		total_center_nums=total_center_nums*branches;

	if(total_center_nums*10>feature_nums)
	{
		cout<<"样本过少或BoVW末端分支过多，请继续采样或减小k-d树分支树及层数"<<endl;
		return false;
	}

	//2.分层级k-means聚类
	vector<vector<int>> samples_belones_upper_level;//k-d树上一层级所有样本所属
	vector<vector<int>> samples_belones_cureent_level;//k-d树当前层级所有样本所属

	vector<vector<float>> k_means_result;//聚类结果(聚类中心)
	vector<int> samples_belones_result;//样本所属
	int groups_in_each_level=1;//k-d树每一层的样本分组数

	for(int i=0;i<tree_levels;i++)
	{
		//根据上一层的分组，计算当前分组个数
		if(i==0)//如果是第1层
		{
			groups_in_each_level=groups_in_each_level*branches;//更新当前层总的样本分组数量
			samples_belones_cureent_level.resize(groups_in_each_level);
			cout<<i<<" 层k-means均值聚类"<<endl;

			//调用k-means算法聚类
			samples_belones_result.clear();//清空样本所属分支
			k_means_result.clear();//清空k-means聚类中心结果
			bool k_means_effect=myK_Means(samples_of_feature_,branches,k_means_result,samples_belones_result);

			//将k-means聚类中心存入分支树数据结构中
			int k_means_result_size=k_means_result.size();
			for(int j=0;j<k_means_result_size;j++)
			{
				bag_of_visual_word_.kd_tree_data.push_back(k_means_result[j]);
			}

			//将样本分组结果存于samples_belones_upper_level
			samples_belones_upper_level.resize(branches);
			int samples_belones_result_sz=samples_belones_result.size();
			for(int j=0;j<samples_belones_result_sz;j++)
			{
				samples_belones_upper_level[samples_belones_result[j]].push_back(j);//将属于某一分支的样本ID存入该分支
			}
			continue;
		}

		//层数大于1时的聚类结果
		groups_in_each_level=groups_in_each_level*branches;//更新当前层总的样本分组数量
		samples_belones_cureent_level.clear();
		samples_belones_cureent_level.resize(groups_in_each_level);//更新当前层总的样本分组数量
		cout<<i<<" 层k-means均值聚类"<<endl;

		int samples_belones_upper_level_sz=samples_belones_upper_level.size();
		for(int j=0;j<samples_belones_upper_level_sz;j++)
		{
			cout<<i<<" 层k-means均值聚类,分支："<<j<<endl;

			//提取上一层i分支所属样本
			vector<vector<float>> samples_used_now;
			for(int k=0;k<samples_belones_upper_level[j].size();k++)
			{
				samples_used_now.push_back(samples_of_feature_[samples_belones_upper_level[j][k]]);
			}

			//调用k-means算法聚类
			samples_belones_result.clear();//清空样本所属分支
			k_means_result.clear();//清空k-means聚类中心结果
			bool k_means_effect=myK_Means(samples_used_now,branches,k_means_result,samples_belones_result);
			if(k_means_effect==false)//如果聚类结果无效
			{
				//该分支所有聚类中心等于父节点
				int upper_index=branches;//计算父节点索引
				for(int k=0;k<i-1;k++)
				{
					upper_index=upper_index*branches;
				}
				upper_index=(upper_index-branches)/(branches-1);
				upper_index=upper_index+j;

				k_means_result.resize(branches);
				for(int k=0;k<branches;k++)
				{
					k_means_result[k]=bag_of_visual_word_.kd_tree_data[upper_index];
				}
			}

			//将k-means聚类中心存入分支树数据结构中
			int k_means_result_sz=k_means_result.size();
			for(int k=0;k<k_means_result_sz;k++)
			{
				bag_of_visual_word_.kd_tree_data.push_back(k_means_result[k]);
			}

			//将样本分组结果存于samples_belones_upper_level
			int samples_belones_result_sz=samples_belones_result.size();
			for(int k=0;k<samples_belones_result_sz;k++)
			{
				samples_belones_cureent_level[j*branches+samples_belones_result[k]].push_back(samples_belones_upper_level[j][k]);//将属于某一分支的样本ID存入该分支
			}
		}

		//将当前层作为upper_level
		samples_belones_upper_level=samples_belones_cureent_level;
	}

	//选择是否储存本次训练结果BagofVisvalWord
	result=MessageBox(0, L"是否储存本次 BoVW 训练结果?",L"选择", MB_YESNO);
	if(result==IDNO)//退出buildBagofVisvalWord
	{
		cout<<"退出视觉词典训练程序..."<<endl;
		Sleep(500);
		return false;
	}
	cout<<"储存 BoVW  ..."<<endl;

	//3.将聚类结果存入dat文档
	string save_nm=feature_type_+" BoVW.dat";
	string file_name="BoVW/"+save_nm;
	ofstream outfile(file_name);
	//打开文件
	if(!outfile)//文件不存在
	{
		file_name="../"+file_name;
		outfile.open(file_name);
		if(!outfile.is_open())
		{
			cout<<"无法打开文件:"<<file_name<<endl;
			return false;
		}
	}

	//写入BoVW层数与分支数
	cout<<"写入BoVW层数与分支数"<<endl;
	outfile<<tree_levels<<endl;//BoVW层数
	outfile<<branches<<endl;//BoVW分支数

	//写入聚类中心
	cout<<"写入聚类中心..."<<endl;
	for(int i=0;i<bag_of_visual_word_.kd_tree_data.size();i++)
	{
		outfile<<" ";
		for(int j=0;j<bag_of_visual_word_.kd_tree_data[i].size();j++)
		{
			outfile<<bag_of_visual_word_.kd_tree_data[i][j]<<" ";
		}
		outfile<<"\n";
	}

	return true;
}

//载入视觉词典BoVW
bool TrainBoVWofFeature::loadBagofVisvalWord(string& file_name)
{
	//清空当前词典
	bag_of_visual_word_.kd_tree_data.clear();

	//读取BoVW文档
	ifstream infile(file_name);
	//开始写入数据
	if(!infile)//文件不处于写的状态
	{
		file_name="../"+file_name;
		infile.open(file_name);
		if(!infile.is_open())
		{
			cout<<"error:无法打开文件:"<<file_name<<endl;
			return false;
		}
	}

	string file_data;//一行的文字
	vector<float> input_data;//输入的数字
	int line_num=0;//读取行数
	while(!infile.eof())//一直读到文件末尾
	{
		getline( infile, file_data );//获取一行数据
		if(file_data.size()==0)
		{
			//空行
			continue;
		}

		//读取BoVW层数
		if(line_num==0)
		{
			bag_of_visual_word_.tree_levels=atof(file_data.c_str());//BoVW层数
			cout<<"读取BoVW层数:"<<bag_of_visual_word_.tree_levels<<endl;
			line_num++;
			continue;
		}
		//读取BoVW分支数
		if(line_num==1)
		{
			bag_of_visual_word_.branches=atof(file_data.c_str());//BoVW每层分支数
			cout<<"读取BoVW每层分支数:"<<bag_of_visual_word_.branches<<endl;
			line_num++;
			continue;
		}

		//读取BoVW聚类中心数据
		//获取一行数据的空格数
		vector<int> space_position;
		for(int i=0;i<file_data.length();i++)//搜索space的位置
		{
			if(file_data[i]==' ')
				space_position.push_back(i);
		}
		//根据空格数存入一行数据
		input_data.clear();
		for(int i=0;i<space_position.size()-1;i++)
		{
			float data_here=atof(file_data.substr(space_position[i]+1,space_position[i+1]).c_str());//聚类中心数据
			input_data.push_back(data_here);
		}
		bag_of_visual_word_.kd_tree_data.push_back(input_data);

		if(line_num%100==0)
		{
			cout<<"读取第 "<<line_num-1<<" 行数据"<<endl;
		}
		line_num++;
	}

	//检查数据的完整性
	int datas_num_real=line_num-2;//获得的聚类中心数量
	int datas_num_theory=bag_of_visual_word_.branches;//理论上的总和为：[branches^(tree_levels+1)-1]/(branches-1)
	for(int i=0;i<bag_of_visual_word_.tree_levels;i++)
	{
		datas_num_theory=datas_num_theory*bag_of_visual_word_.branches;
	}
	datas_num_theory=datas_num_theory-bag_of_visual_word_.branches;
	datas_num_theory=datas_num_theory/(bag_of_visual_word_.branches-1);

	if(datas_num_theory==datas_num_real)//理论个数与实际相符
		return true;
	else
		return true;
}

//输出视觉词典第level层第branches分支的描述符
bool TrainBoVWofFeature::getDataFromBagofVisvalWord(vector<int>& data_index,vector<float>& data)
{
	//判断索引是否溢出
	if(data_index.size()>bag_of_visual_word_.tree_levels)
	{
		cout<<"error: 索引层数过大,最大层数索引为:"<<bag_of_visual_word_.tree_levels<<endl;
		return false;
	}
	bool flag=true;
	int data_index_size=data_index.size();
	for(int i=0;i<data_index_size;i++)
	{
		if(data_index[i]<0||data_index[i]>bag_of_visual_word_.branches-1)
		{
			cout<<"error: 第 "<<i<<" 层分支索引不在BoVW范围内 ,分支索引从0开始,最大索引为:"<<bag_of_visual_word_.branches-1<<endl;
			flag=false;
			break;
		}
	}
	if(flag==false)
		return false;

	//计算目标数据在bag_of_visual_word_中的索引
	int data_id=0;
	int ranges_in_each_level=1;
	int data_index_sz=data_index.size();
	for(int i=0;i<data_index_sz-1;i++)
	{
		ranges_in_each_level=ranges_in_each_level*bag_of_visual_word_.branches;
		data_id=data_id+ranges_in_each_level;
	}

	int current_level_index=data_index[0];
	for(int i=1;i<data_index_sz;i++)
	{
		current_level_index=current_level_index*bag_of_visual_word_.branches+data_index[i];
	}
	data_id=data_id+current_level_index;

	if(data_id>bag_of_visual_word_.kd_tree_data.size())
	{
		cout<<"error: 索引超出范围"<<endl;
		return false;
	}

	//返回的数据
	data=bag_of_visual_word_.kd_tree_data[data_id];

	return true;
}

//计算特征向量索引
bool TrainBoVWofFeature::getIndexofFeature(vector<float>& input_feature_data,vector<int>& feature_index)
{
	//0.检测特征向量长度是否符合要求
	if(bag_of_visual_word_.kd_tree_data.size()==0)
	{
		cout<<"error:未载入视觉词典"<<endl;
		return false;
	}
	if(input_feature_data.size()!=bag_of_visual_word_.kd_tree_data[0].size())
	{
		cout<<"error:输入特征长度不匹配，正确长度为："<<bag_of_visual_word_.kd_tree_data[0].size()<<endl;
		return false;
	}

	//1.计算在视觉词典中各层的索引
	//采用欧氏距离
	int level_start_index=0;//各层的初始位置
	int upper_index=0;//上一层索引
	int upper_positon=0;//上一层所属分支的本层级位置
	int best_index=0;
	float best_distance=100;
	vector<float> bovw_data;//当前视觉词典特征向量
	feature_index.clear();
	for(int i=0;i<bag_of_visual_word_.tree_levels;i++)
	{
		//计算最优距离
		best_distance=100;
		for(int j=0;j<bag_of_visual_word_.branches;j++)
		{
			//计算索引
			int now_posiotion=level_start_index+upper_positon*bag_of_visual_word_.branches+j;
			bovw_data=bag_of_visual_word_.kd_tree_data[now_posiotion];
			//计算距离
			int bovw_data_size=bovw_data.size();
			float distance_now=0;
			for(int k=0;k<bovw_data_size;k++)
			{
				distance_now+=(bovw_data[k]-input_feature_data[k])*(bovw_data[k]-input_feature_data[k]);
			}
			//判断最优距离
			if(distance_now<best_distance)
			{
				best_distance=distance_now;
				best_index=j;
			}
		}
		//保存最优索引
		feature_index.push_back(best_index);

		//更新上一级索引
		level_start_index=(level_start_index+1)*bag_of_visual_word_.branches;

		upper_index=best_index;//上一层索引
		upper_positon=upper_positon*bag_of_visual_word_.branches+best_index;//上一层所属分支的本层级位置
	}

	return true;
}

//计算特征向量索引
bool TrainBoVWofFeature::getIndexofFeature(cv::Mat& input_feature_data,cv::Mat& feature_index)
{
	//0.检测特征向量长度是否符合要求
	if(bag_of_visual_word_.kd_tree_data.size()==0)
	{
		cout<<"error:未载入视觉词典"<<endl;
		return false;
	}
	int feature_size=input_feature_data.cols;
	if(feature_size!=bag_of_visual_word_.kd_tree_data[0].size())
	{
		cout<<"error:输入特征长度不正确。正确长度为："<<bag_of_visual_word_.kd_tree_data[0].size()<<endl;
		return false;
	}

	//1.计算在视觉词典中各层的索引
	//采用欧氏距离
	int level_start_index=0;//各层的初始位置
	int upper_index=0;//上一层索引
	int upper_positon=0;//上一层所属分支的本层级位置
	int best_index=0;
	float best_distance=100;
	vector<float> bovw_data;//当前视觉词典特征向量

	int dep=feature_index.depth();
	if(feature_index.cols!=bag_of_visual_word_.tree_levels||feature_index.channels()!=1)
	{
		feature_index=cv::Mat(1,bag_of_visual_word_.tree_levels,CV_8UC1);
	}

	float* feature_ptr=input_feature_data.ptr<float>(0);
	uchar* index_ptr=feature_index.ptr<uchar>(0);

	for(int i=0;i<bag_of_visual_word_.tree_levels;i++)
	{
		//计算最优距离
		best_distance=100;
		for(int j=0;j<bag_of_visual_word_.branches;j++)
		{
			//计算索引
			int now_posiotion=level_start_index+upper_positon*bag_of_visual_word_.branches+j;
			bovw_data=bag_of_visual_word_.kd_tree_data[now_posiotion];

			//计算距离
			int bovw_data_size=bovw_data.size();
			float distance_now=0,dt;
			for(int k=0;k<bovw_data_size;k++)
			{
				dt=bovw_data[k]-feature_ptr[k];
				distance_now+=dt*dt;
			}

			//判断最优距离
			if(distance_now<best_distance)
			{
				best_distance=distance_now;
				best_index=j;
			}
		}
		//保存最优索引
		index_ptr[i]=best_index;

		//更新上一级索引
		level_start_index=(level_start_index+1)*bag_of_visual_word_.branches;

		upper_index=best_index;//上一层索引
		upper_positon=upper_positon*bag_of_visual_word_.branches+best_index;//上一层所属分支的本层级位置
	}

	return true;
}


//获得视觉词典规模
void TrainBoVWofFeature::getSizeofBoVW(int& tree_levels,int& branches)
{
	tree_levels=bag_of_visual_word_.tree_levels;
	branches=bag_of_visual_word_.branches;
}

//输出视觉词典树
K_D_tree TrainBoVWofFeature::getBagofVisvalWord()
{
	K_D_tree returnBagofVisvalWord=bag_of_visual_word_;
	return returnBagofVisvalWord;
}

/*********************  private函数  **********************/

//输入采集图像
bool TrainBoVWofFeature::catchFrame(cv::Mat& input_frame)
{
	if(input_frame.rows<8||input_frame.cols<8)//图像大小不对
		return false;

	cv::pyrDown(input_frame,frame_now_,cv::Size(input_frame.cols/2,input_frame.rows/2));
	return true;
}

//特征采样
bool TrainBoVWofFeature::detectFeature()
{
	//1.对frame_now_进行特征检测
	vector<cv::KeyPoint> feature_keypoints;
	feature_detector_->detect(frame_now_,feature_keypoints);//特征点检测
	cv::Mat feature_descriptor;
	feature_extractor_->compute(frame_now_,feature_keypoints,feature_descriptor);//生成特征向量
	cv::Mat draw_mat;
	cv::drawKeypoints(frame_now_,feature_keypoints,draw_mat,cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);//绘制特征
	cv::imshow( "keypoints", draw_mat );
	if(feature_keypoints.size()==0)
	{
		cout<<"error: 当前图片没有特征!"<<endl;
		return false;
	}

	//2.将特征向量存入特征样本集
	vector<float> feature_descriptor_ready;
	feature_descriptor_ready.resize(feature_descriptor.cols);//提取特征描述
	for(int i=0;i<feature_descriptor.rows;i++)
	{
		float* ft_descriptor_ptr=feature_descriptor.ptr<float>(i);
		for(int j=0;j<feature_descriptor.cols;j++)
		{
			feature_descriptor_ready[j]=ft_descriptor_ptr[j];
		}
		samples_of_feature_.push_back(feature_descriptor_ready);//存入样本集
	}
}

bool TrainBoVWofFeature::myK_Means(vector<vector<float>>& samples,int classify_num,vector<vector<float>>& k_meas_result,vector<int>& samples_belones_result)
{
	cout<<"当前聚类采用的样本个数："<<samples.size()<<endl;
	//通过k-means计算BoVW各层的聚类中心
	if(samples.size()<classify_num)
	{
		cout<<"eror: k-means聚类样本无效！"<<endl;
		Sleep(500);
		return false;
	}

	//1.根据classify_num确定k-means聚类中心个数
	k_meas_result.resize(classify_num);

	//2.初始化聚类中心
	for(int i=0;i<classify_num;i++)
	{
		k_meas_result[i].resize(samples[0].size());
	}
	//统计samples的范围
	int sample_l=samples[0].size();
	vector<float> min_of_samples(sample_l),max_of_samples(sample_l);
	
	int min_of_samples_size=min_of_samples.size();
	for(int i=0;i<min_of_samples_size;i++)
	{
		min_of_samples[i]=100;
		max_of_samples[i]=-100;
	}
	int samples_size=samples.size();
	for(int i=0;i<samples_size;i++)
	{
		int samples_i_size=samples[i].size();
		for(int j=0;j<samples_i_size;j++)
		{
			if(samples[i][j]>max_of_samples[j])
			{
				max_of_samples[j]=samples[i][j];
			}
			if(samples[i][j]<min_of_samples[j])
			{
				min_of_samples[j]=samples[i][j];
			}
		}
	}

	//根据统计结果给出随机初始化的聚类中心
	//  统计min_of_samples与max_of_samples极差最大的前若干个坐标轴,作为聚类参考轴
	vector<float> div_of_coordinate,coordinate_order;
	div_of_coordinate.resize(min_of_samples_size);
	coordinate_order.resize(min_of_samples_size);
	for(int i=0;i<min_of_samples_size;i++)//计算极差
	{
		div_of_coordinate[i]=max_of_samples[i]-min_of_samples[i];
		coordinate_order[i]=i;
	}
	for(int i=1;i<min_of_samples_size;i++)//由大到小排序
	{
		for(int j=0;j<i;j++)
		{
			if(div_of_coordinate[i]>div_of_coordinate[j])
			{
				float middle_num=div_of_coordinate[i];
				div_of_coordinate[i]=div_of_coordinate[j];
				div_of_coordinate[j]=middle_num;
				int middle_i=coordinate_order[i];
				coordinate_order[i]=coordinate_order[j];
				coordinate_order[j]=middle_i;
			}
		}
	}
	int divide_nums=classify_num/2;
	vector<int> initial_k_means_coordinate;
	initial_k_means_coordinate.resize(divide_nums+1);
	for(int i=0;i<divide_nums+1;i++)
	{
		initial_k_means_coordinate[i]=coordinate_order[i];
	}
	//  根据聚类参考轴，给出聚类中心
	vector<float> center_of_coordinate;//各个轴上的中心
	center_of_coordinate.resize(min_of_samples_size);
	for(int i=0;i<min_of_samples_size;i++)
	{
		center_of_coordinate[i]=(max_of_samples[i]+min_of_samples[i])/2;
	}
	for(int i=0;i<divide_nums;i++)
	{
		k_meas_result[2*i]=center_of_coordinate;
		k_meas_result[2*i+1]=center_of_coordinate;
		int different_coordinate=initial_k_means_coordinate[i];//有区别的轴
		k_meas_result[2*i][different_coordinate]=center_of_coordinate[different_coordinate]-(max_of_samples[different_coordinate]-min_of_samples[different_coordinate])/4;
		k_meas_result[2*i+1][different_coordinate]=center_of_coordinate[different_coordinate]+(max_of_samples[different_coordinate]-min_of_samples[different_coordinate])/4;
		//加入随机干扰
		k_meas_result[2*i][different_coordinate]+=(0.2*rand()/(RAND_MAX+1)-0.1)*(max_of_samples[different_coordinate]-min_of_samples[different_coordinate])/4;
		k_meas_result[2*i+1][different_coordinate]+=(0.2*rand()/(RAND_MAX+1)-0.1)*(max_of_samples[different_coordinate]-min_of_samples[different_coordinate])/4;
	}
	//如果classify_num为奇数，需要额外加上一个聚类中心
	if(classify_num-divide_nums*2==1)
	{
		int different_coordinate=initial_k_means_coordinate[divide_nums];//有区别的轴
		k_meas_result[classify_num-1]=center_of_coordinate;
		k_meas_result[classify_num-1][different_coordinate]=center_of_coordinate[different_coordinate]-(max_of_samples[different_coordinate]-min_of_samples[different_coordinate])/4;
		//加入随机干扰
		k_meas_result[classify_num-1][different_coordinate]+=(0.2*rand()/(RAND_MAX+1)-0.1)*(max_of_samples[different_coordinate]-min_of_samples[different_coordinate])/4;
	}

	//3.开始迭代k-means
	vector<vector<float>> k_meas_result_before;//前一次结果
	float jump_condition;//跳出条件
	for(int iteration_nums=0;iteration_nums<100;iteration_nums++)//最大迭代次数为100
	{
		cout<<"k-means迭代次数："<<iteration_nums+1<<endl;
		//根据距离，更新各个样本所属中心
		vector<int> samples_belonges;//样本的所属聚类中心
		samples_belonges.resize(samples_size);
		for(int i=0;i<samples_size;i++)
		{
			float distance_to_center=1E8;
			int belones;
			int k_meas_result_size=k_meas_result.size();
			int samples_i_size=samples[i].size();
			for(int j=0;j<k_meas_result_size;j++)
			{
				float diatance_now=0;
				for(int k=0;k<samples_i_size;k++)
				{
					diatance_now=diatance_now+(samples[i][k]-k_meas_result[j][k])*(samples[i][k]-k_meas_result[j][k]);
				}
				if(diatance_now<distance_to_center)
				{
					distance_to_center=diatance_now;//更新到中心距离
					belones=j;//更新所属中心
				}
			}
			samples_belonges[i]=belones;
		}
		samples_belones_result=samples_belonges;//给输出样本所属赋值
		//根据样本所属，更新聚类中心坐标
		int k_meas_result_size=k_meas_result.size();
		for(int i=0;i<k_meas_result_size;i++)//聚类中心归零
		{
			int k_meas_result_i_size=k_meas_result[i].size();
			for(int j=0;j<k_meas_result_i_size;j++)
			{
				k_meas_result[i][j]=0;
			}
		}
		vector<int> samples_nums;//各个聚类中心包含的样本数
		samples_nums.resize(k_meas_result_size);
		for(int i=0;i<k_meas_result_size;i++)
			samples_nums[i]=0;

		for(int i=0;i<samples_size;i++)//计算累计值
		{
			int samples_i_size=samples[i].size();
			for(int j=0;j<samples_i_size;j++)
			{
				k_meas_result[samples_belonges[i]][j]+=samples[i][j];
			}
			samples_nums[samples_belonges[i]]++;//所属个数
		}
		for(int i=0;i<k_meas_result_size;i++)//计算均值
		{
			if(samples_nums[i]==0)//如果当前聚类中心没有所属SURF特征
				continue;
			int k_meas_result_i_size=k_meas_result[i].size();
			for(int j=0;j<k_meas_result_i_size;j++)
			{
				k_meas_result[i][j]=k_meas_result[i][j]/samples_nums[i];
			}
		}

		//4.判断是否满足跳出条件
		if(iteration_nums==1)//计算跳出条件阈值，实为距离
		{
			jump_condition=0;
			for(int i=0;i<k_meas_result_size;i++)
			{
				int k_meas_result_i_size=k_meas_result[i].size();
				for(int j=0;j<k_meas_result_i_size;j++)
				{
					jump_condition+=(k_meas_result_before[i][j]-k_meas_result[i][j])*(k_meas_result_before[i][j]-k_meas_result[i][j]);
				}
			}
			jump_condition=jump_condition/100;
			if(jump_condition<0.000001)//修正过小的跳出条件阈值
			{
				jump_condition=0.000001;
			}
			cout<<"k-means终止阈值："<<jump_condition<<endl;
		}
		if(iteration_nums>1)
		{
			float judge_num=0;
			for(int i=0;i<k_meas_result_size;i++)
			{
				int k_meas_result_i_size=k_meas_result[i].size();
				for(int j=0;j<k_meas_result_i_size;j++)
				{
					judge_num+=(k_meas_result_before[i][j]-k_meas_result[i][j])*(k_meas_result_before[i][j]-k_meas_result[i][j]);
				}
			}
			cout<<"当前k-means跳出条件数："<<judge_num<<endl;
			if(judge_num<jump_condition&&iteration_nums>10)//满足跳出条件
			{
				return true;
			}
		}

		//将当前时刻中心做为前一时刻中心
		k_meas_result_before=k_meas_result;
	}

	return false;
}