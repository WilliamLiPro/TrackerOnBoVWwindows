/*************************************************
	>BoVW of Different Kind of Features

	>programmed by ��ά��
	>2016.5.13

	>1.����opencv��������ɼ�
	>2.����k-means�㷨���������࣬������ͬ��������BoVW
	>3.����ŷ�Ͼ������������

*************************************************/
#include "stdafx.h"
#include "TrainBoVWofFeature.h"

/*********************  public����  **********************/

/* ���ʼ��*/
TrainBoVWofFeature::TrainBoVWofFeature(string feature_type,string descriptor_type)
{
	cout<<"************* All right reserved by Author ��ά�� ***************"<<endl;
	cout<<"��ӭʹ�ã�TrainBoVWofFeature ��"<<endl;
	cout<<"���������m��k��֧��������ͬ���������Ӿ��ʵ�"<<endl;
	cout<<"��������Ϊ"<<feature_type<<endl;

	//���ݳ�ʼ����Ĭ���Ӿ��ʵ��С
	bag_of_visual_word_.tree_levels=5;
	bag_of_visual_word_.branches=4;

	//�����������ʼ��
	cv::initModule_nonfree();		//�������SURF��SIFT
	feature_type_=feature_type;
	feature_detector_=cv::FeatureDetector::create(feature_type);

	//������������ʼ��
	descriptor_type_=descriptor_type;
	feature_extractor_=cv::DescriptorExtractor::create(feature_type);

	//���������洢����ʼ��
	samples_of_feature_.reserve(100000);

	//�Ƿ���Ʋ������
	draw_keysampling_=false;
}

/*	�����ɼ���BoVW����
	����:
	data_type	0Ϊ����ͷ�ɼ���1Ϊ��Ƭ��
	camera_id	����������ͷ���*/
bool TrainBoVWofFeature::getSamplesOfFeature(int data_type,int camera_id)
{
	cv::VideoCapture capture;
	if(data_type==0)//ѡ������ͷ
	{
		//��ʼ������ͷ
		cout<<"����ͷ��ʼ��..."<<endl;
		capture=cv::VideoCapture(camera_id);//�洢����ͷ���������ָ��
		if(!capture.isOpened()) // ������ͷ
		{
			MessageBox(0, L"�޷�������ͷ: �볢����������ͷ",L"error", MB_ICONERROR);
			return -1;
		}
	}
	if(data_type==1)//ѡ��ͼƬ��
	{
		cout<<"������ͼƬ·�� (eg.img_%02d.jpg)"<<endl;
		string picture_file_path;
		cin>>picture_file_path;
		cout<<"���ļ�... "<<picture_file_path<<endl;

		capture=cv::VideoCapture(picture_file_path.c_str());
		if(!capture.isOpened()) // ��ͼƬ·��
		{
			picture_file_path="E:/������ϵͳ/SLAM/KITTI���ݼ�/data_odometry_gray/00/image_0/%06d.png";
			capture=cv::VideoCapture(picture_file_path.c_str());
			if(!capture.isOpened()) // ��ͼƬ·��
			{
				MessageBox(0, L"�޷���ָ���ļ�",L"error", MB_ICONERROR);
				return -1;
			}
		}
	}

	//ѡ���Ƿ������һ�εĲ���
	int result=MessageBox(0, L"�Ƿ������һ�εĲ�����",L"ѡ��", MB_YESNO);
	string feature_nm=feature_type_+" samples.dat";
	string file_name="feature samples/"+feature_nm;
	ios_base::openmode open_mode;
	int lines_of_file=0;
	if(result==IDYES)
	{
		//��������һ�εĲ������
		open_mode=ios::app;
		//��ȡ�ļ�����
		ifstream infile(file_name);
		if(!infile)//�ļ�������
		{
			file_name="../"+file_name;
			infile.open(file_name);
			if(!infile.is_open())
				cout<<"�ļ��򲻿�:"<<file_name<<endl;
		}
		string data;
		while(!infile.eof())
		{
			getline( infile, data );//��ȡһ������
			if(data.size()==0)
			{
				//����
				continue;
			}
			lines_of_file++;//ͳ������
		}
	}
	else
	{
		//������һ�εĲ������
		open_mode=ios::trunc;
	}
	ofstream outfile(file_name,open_mode);
	if(!outfile)//�ļ�������
	{
		file_name="../"+file_name;
		outfile.open(file_name);
	}

	//��ʼ��ȡͼ��
	cout<<"�밴������˳�����"<<endl;
	Sleep(500);
	cv::Mat frame_in;	//�洢����ͷ��ȡ��ͼ��
	while(1)
	{
		capture >> frame_in;//��ȡͼ��
		bool catch_result=catchFrame(frame_in);//ͼ�񽵲���
		if(catch_result==false)//ͼ���������ȷ
		{
			continue;
		}

		bool ft_result=detectFeature();//��������������
		if(ft_result==false)//ͼƬû����Ч����
		{
			continue;
		}

		int total_sp=samples_of_feature_.size();	//��������
		cout<<"��ǰ�ܵ�������ĿΪ"<<total_sp+lines_of_file<<endl;

		if(cv::waitKey(5)>0||total_sp+lines_of_file>1000000)//������Ŀ���࣬ǿ���˳�
		{
			break;
		}
	}

	//�洢�������
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

//�����������
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

//����k-d���Ĳ����Լ���֧��
bool TrainBoVWofFeature::setBoVWlevelsAndBranches(int tree_levels,int branches)
{
	//�ж������Ƿ���Ч
	if(tree_levels<2||tree_levels>12)
	{
		cout<<"error: ����K-D������������Ч��Χ�� 2~12"<<endl;
		return false;
	}
	if(branches<2||branches>10)
	{
		cout<<"error: ����K-D����֧��������Ч��Χ�� 2~10"<<endl;
		return false;
	}

	bag_of_visual_word_.tree_levels=tree_levels;
	bag_of_visual_word_.branches=branches; 

	return true;
}

//BoVWѵ������
//���룺tree_levels���Ĳ���,branchesÿ���֧��
bool TrainBoVWofFeature::buildBagofVisvalWord()
{
	//ѡ���Ƿ�ѵ��BagofVisvalWord
	int result=MessageBox(0, L"�Ƿ�ʼѵ���Ӿ��ʵ�",L"ѡ��", MB_YESNO);
	if(result==IDNO)//�˳�buildBagofVisvalWord
	{
		cout<<"�˳��Ӿ��ʵ����..."<<endl;
		Sleep(500);
		return false;
	}

	cout<<"�ʵ乹�캯����BoVWѵ��������ڹ���Ŀ¼��BoVW �ļ�����"<<endl;
	cout<<"�ļ�����"<<feature_type_<<" BoVW.dat"<<endl;

	//����k-means�������BoVW

	//0.����samples_of_feature_
	samples_of_feature_.clear();
	string file_nm=feature_type_+" samples.dat";
	string file_name_in="feature samples/"+file_nm;
	cout<<"���ڴ��ļ�: "<<file_name_in<<" ..."<<endl;
	ifstream infile(file_name_in);
	//��ʼд������
	if(!infile)//�ļ�������
	{
		file_name_in="../"+file_name_in;
		infile.open(file_name_in);
		if(!infile.is_open())
		{
			cout<<"�޷����ļ�:"<<file_name_in<<endl;
			return false;
		}
	}
	string file_data;//һ�е�����
	vector<float> input_data;//���������
	input_data.reserve(128);
	cout<<"���ڻ�ȡ��������..."<<endl;
	int feature_nums=0;
	while(!infile.eof())//һֱ�����ļ�ĩβ
	{
		getline( infile, file_data );	//��ȡһ������
		int data_l=file_data.size();	//���ݳ���
		if(data_l==0)
		{
			//����
			continue;
		}
		//��ȡһ�����ݵĿո���
		vector<int> space_position;
		space_position.reserve(128);
		for(int i=0;i<data_l;i++)	//����space��λ��
		{
			if(file_data[i]==' ')
				space_position.push_back(i);
		}
		int space_position_sz=space_position.size();
		input_data.resize(space_position_sz);

		input_data[0]=atof(file_data.substr(0,space_position[0]).c_str());	//��������

		for(int i=1;i<space_position_sz;i++)
		{
			input_data[i]=atof(file_data.substr(space_position[i-1]+1,space_position[i]).c_str());	//��������
		}
		samples_of_feature_.push_back(input_data);

		feature_nums++;
		if(feature_nums%1000==0)
		{
			cout<<"��ȡ�� "<<feature_nums<<" ����������"<<endl;
		}
	}
	cout<<"���������������"<<endl;
	
	//1.�����������ͷ�֧���Ƿ�С��������
	int tree_levels,branches;
	tree_levels=bag_of_visual_word_.tree_levels;
	branches=bag_of_visual_word_.branches;

	int total_center_nums=1;
	for(int i=0;i<tree_levels;i++)
		total_center_nums=total_center_nums*branches;

	if(total_center_nums*10>feature_nums)
	{
		cout<<"�������ٻ�BoVWĩ�˷�֧���࣬������������Сk-d����֧��������"<<endl;
		return false;
	}

	//2.�ֲ㼶k-means����
	vector<vector<int>> samples_belones_upper_level;//k-d����һ�㼶������������
	vector<vector<int>> samples_belones_cureent_level;//k-d����ǰ�㼶������������

	vector<vector<float>> k_means_result;//������(��������)
	vector<int> samples_belones_result;//��������
	int groups_in_each_level=1;//k-d��ÿһ�������������

	for(int i=0;i<tree_levels;i++)
	{
		//������һ��ķ��飬���㵱ǰ�������
		if(i==0)//����ǵ�1��
		{
			groups_in_each_level=groups_in_each_level*branches;//���µ�ǰ���ܵ�������������
			samples_belones_cureent_level.resize(groups_in_each_level);
			cout<<i<<" ��k-means��ֵ����"<<endl;

			//����k-means�㷨����
			samples_belones_result.clear();//�������������֧
			k_means_result.clear();//���k-means�������Ľ��
			bool k_means_effect=myK_Means(samples_of_feature_,branches,k_means_result,samples_belones_result);

			//��k-means�������Ĵ����֧�����ݽṹ��
			int k_means_result_size=k_means_result.size();
			for(int j=0;j<k_means_result_size;j++)
			{
				bag_of_visual_word_.kd_tree_data.push_back(k_means_result[j]);
			}

			//����������������samples_belones_upper_level
			samples_belones_upper_level.resize(branches);
			int samples_belones_result_sz=samples_belones_result.size();
			for(int j=0;j<samples_belones_result_sz;j++)
			{
				samples_belones_upper_level[samples_belones_result[j]].push_back(j);//������ĳһ��֧������ID����÷�֧
			}
			continue;
		}

		//��������1ʱ�ľ�����
		groups_in_each_level=groups_in_each_level*branches;//���µ�ǰ���ܵ�������������
		samples_belones_cureent_level.clear();
		samples_belones_cureent_level.resize(groups_in_each_level);//���µ�ǰ���ܵ�������������
		cout<<i<<" ��k-means��ֵ����"<<endl;

		int samples_belones_upper_level_sz=samples_belones_upper_level.size();
		for(int j=0;j<samples_belones_upper_level_sz;j++)
		{
			cout<<i<<" ��k-means��ֵ����,��֧��"<<j<<endl;

			//��ȡ��һ��i��֧��������
			vector<vector<float>> samples_used_now;
			for(int k=0;k<samples_belones_upper_level[j].size();k++)
			{
				samples_used_now.push_back(samples_of_feature_[samples_belones_upper_level[j][k]]);
			}

			//����k-means�㷨����
			samples_belones_result.clear();//�������������֧
			k_means_result.clear();//���k-means�������Ľ��
			bool k_means_effect=myK_Means(samples_used_now,branches,k_means_result,samples_belones_result);
			if(k_means_effect==false)//�����������Ч
			{
				//�÷�֧���о������ĵ��ڸ��ڵ�
				int upper_index=branches;//���㸸�ڵ�����
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

			//��k-means�������Ĵ����֧�����ݽṹ��
			int k_means_result_sz=k_means_result.size();
			for(int k=0;k<k_means_result_sz;k++)
			{
				bag_of_visual_word_.kd_tree_data.push_back(k_means_result[k]);
			}

			//����������������samples_belones_upper_level
			int samples_belones_result_sz=samples_belones_result.size();
			for(int k=0;k<samples_belones_result_sz;k++)
			{
				samples_belones_cureent_level[j*branches+samples_belones_result[k]].push_back(samples_belones_upper_level[j][k]);//������ĳһ��֧������ID����÷�֧
			}
		}

		//����ǰ����Ϊupper_level
		samples_belones_upper_level=samples_belones_cureent_level;
	}

	//ѡ���Ƿ񴢴汾��ѵ�����BagofVisvalWord
	result=MessageBox(0, L"�Ƿ񴢴汾�� BoVW ѵ�����?",L"ѡ��", MB_YESNO);
	if(result==IDNO)//�˳�buildBagofVisvalWord
	{
		cout<<"�˳��Ӿ��ʵ�ѵ������..."<<endl;
		Sleep(500);
		return false;
	}
	cout<<"���� BoVW  ..."<<endl;

	//3.������������dat�ĵ�
	string save_nm=feature_type_+" BoVW.dat";
	string file_name="BoVW/"+save_nm;
	ofstream outfile(file_name);
	//���ļ�
	if(!outfile)//�ļ�������
	{
		file_name="../"+file_name;
		outfile.open(file_name);
		if(!outfile.is_open())
		{
			cout<<"�޷����ļ�:"<<file_name<<endl;
			return false;
		}
	}

	//д��BoVW�������֧��
	cout<<"д��BoVW�������֧��"<<endl;
	outfile<<tree_levels<<endl;//BoVW����
	outfile<<branches<<endl;//BoVW��֧��

	//д���������
	cout<<"д���������..."<<endl;
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

//�����Ӿ��ʵ�BoVW
bool TrainBoVWofFeature::loadBagofVisvalWord(string& file_name)
{
	//��յ�ǰ�ʵ�
	bag_of_visual_word_.kd_tree_data.clear();

	//��ȡBoVW�ĵ�
	ifstream infile(file_name);
	//��ʼд������
	if(!infile)//�ļ�������д��״̬
	{
		file_name="../"+file_name;
		infile.open(file_name);
		if(!infile.is_open())
		{
			cout<<"error:�޷����ļ�:"<<file_name<<endl;
			return false;
		}
	}

	string file_data;//һ�е�����
	vector<float> input_data;//���������
	int line_num=0;//��ȡ����
	while(!infile.eof())//һֱ�����ļ�ĩβ
	{
		getline( infile, file_data );//��ȡһ������
		if(file_data.size()==0)
		{
			//����
			continue;
		}

		//��ȡBoVW����
		if(line_num==0)
		{
			bag_of_visual_word_.tree_levels=atof(file_data.c_str());//BoVW����
			cout<<"��ȡBoVW����:"<<bag_of_visual_word_.tree_levels<<endl;
			line_num++;
			continue;
		}
		//��ȡBoVW��֧��
		if(line_num==1)
		{
			bag_of_visual_word_.branches=atof(file_data.c_str());//BoVWÿ���֧��
			cout<<"��ȡBoVWÿ���֧��:"<<bag_of_visual_word_.branches<<endl;
			line_num++;
			continue;
		}

		//��ȡBoVW������������
		//��ȡһ�����ݵĿո���
		vector<int> space_position;
		for(int i=0;i<file_data.length();i++)//����space��λ��
		{
			if(file_data[i]==' ')
				space_position.push_back(i);
		}
		//���ݿո�������һ������
		input_data.clear();
		for(int i=0;i<space_position.size()-1;i++)
		{
			float data_here=atof(file_data.substr(space_position[i]+1,space_position[i+1]).c_str());//������������
			input_data.push_back(data_here);
		}
		bag_of_visual_word_.kd_tree_data.push_back(input_data);

		if(line_num%100==0)
		{
			cout<<"��ȡ�� "<<line_num-1<<" ������"<<endl;
		}
		line_num++;
	}

	//������ݵ�������
	int datas_num_real=line_num-2;//��õľ�����������
	int datas_num_theory=bag_of_visual_word_.branches;//�����ϵ��ܺ�Ϊ��[branches^(tree_levels+1)-1]/(branches-1)
	for(int i=0;i<bag_of_visual_word_.tree_levels;i++)
	{
		datas_num_theory=datas_num_theory*bag_of_visual_word_.branches;
	}
	datas_num_theory=datas_num_theory-bag_of_visual_word_.branches;
	datas_num_theory=datas_num_theory/(bag_of_visual_word_.branches-1);

	if(datas_num_theory==datas_num_real)//���۸�����ʵ�����
		return true;
	else
		return true;
}

//����Ӿ��ʵ��level���branches��֧��������
bool TrainBoVWofFeature::getDataFromBagofVisvalWord(vector<int>& data_index,vector<float>& data)
{
	//�ж������Ƿ����
	if(data_index.size()>bag_of_visual_word_.tree_levels)
	{
		cout<<"error: ������������,����������Ϊ:"<<bag_of_visual_word_.tree_levels<<endl;
		return false;
	}
	bool flag=true;
	int data_index_size=data_index.size();
	for(int i=0;i<data_index_size;i++)
	{
		if(data_index[i]<0||data_index[i]>bag_of_visual_word_.branches-1)
		{
			cout<<"error: �� "<<i<<" ���֧��������BoVW��Χ�� ,��֧������0��ʼ,�������Ϊ:"<<bag_of_visual_word_.branches-1<<endl;
			flag=false;
			break;
		}
	}
	if(flag==false)
		return false;

	//����Ŀ��������bag_of_visual_word_�е�����
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
		cout<<"error: ����������Χ"<<endl;
		return false;
	}

	//���ص�����
	data=bag_of_visual_word_.kd_tree_data[data_id];

	return true;
}

//����������������
bool TrainBoVWofFeature::getIndexofFeature(vector<float>& input_feature_data,vector<int>& feature_index)
{
	//0.����������������Ƿ����Ҫ��
	if(bag_of_visual_word_.kd_tree_data.size()==0)
	{
		cout<<"error:δ�����Ӿ��ʵ�"<<endl;
		return false;
	}
	if(input_feature_data.size()!=bag_of_visual_word_.kd_tree_data[0].size())
	{
		cout<<"error:�����������Ȳ�ƥ�䣬��ȷ����Ϊ��"<<bag_of_visual_word_.kd_tree_data[0].size()<<endl;
		return false;
	}

	//1.�������Ӿ��ʵ��и��������
	//����ŷ�Ͼ���
	int level_start_index=0;//����ĳ�ʼλ��
	int upper_index=0;//��һ������
	int upper_positon=0;//��һ��������֧�ı��㼶λ��
	int best_index=0;
	float best_distance=100;
	vector<float> bovw_data;//��ǰ�Ӿ��ʵ���������
	feature_index.clear();
	for(int i=0;i<bag_of_visual_word_.tree_levels;i++)
	{
		//�������ž���
		best_distance=100;
		for(int j=0;j<bag_of_visual_word_.branches;j++)
		{
			//��������
			int now_posiotion=level_start_index+upper_positon*bag_of_visual_word_.branches+j;
			bovw_data=bag_of_visual_word_.kd_tree_data[now_posiotion];
			//�������
			int bovw_data_size=bovw_data.size();
			float distance_now=0;
			for(int k=0;k<bovw_data_size;k++)
			{
				distance_now+=(bovw_data[k]-input_feature_data[k])*(bovw_data[k]-input_feature_data[k]);
			}
			//�ж����ž���
			if(distance_now<best_distance)
			{
				best_distance=distance_now;
				best_index=j;
			}
		}
		//������������
		feature_index.push_back(best_index);

		//������һ������
		level_start_index=(level_start_index+1)*bag_of_visual_word_.branches;

		upper_index=best_index;//��һ������
		upper_positon=upper_positon*bag_of_visual_word_.branches+best_index;//��һ��������֧�ı��㼶λ��
	}

	return true;
}

//����������������
bool TrainBoVWofFeature::getIndexofFeature(cv::Mat& input_feature_data,cv::Mat& feature_index)
{
	//0.����������������Ƿ����Ҫ��
	if(bag_of_visual_word_.kd_tree_data.size()==0)
	{
		cout<<"error:δ�����Ӿ��ʵ�"<<endl;
		return false;
	}
	int feature_size=input_feature_data.cols;
	if(feature_size!=bag_of_visual_word_.kd_tree_data[0].size())
	{
		cout<<"error:�����������Ȳ���ȷ����ȷ����Ϊ��"<<bag_of_visual_word_.kd_tree_data[0].size()<<endl;
		return false;
	}

	//1.�������Ӿ��ʵ��и��������
	//����ŷ�Ͼ���
	int level_start_index=0;//����ĳ�ʼλ��
	int upper_index=0;//��һ������
	int upper_positon=0;//��һ��������֧�ı��㼶λ��
	int best_index=0;
	float best_distance=100;
	vector<float> bovw_data;//��ǰ�Ӿ��ʵ���������

	int dep=feature_index.depth();
	if(feature_index.cols!=bag_of_visual_word_.tree_levels||feature_index.channels()!=1)
	{
		feature_index=cv::Mat(1,bag_of_visual_word_.tree_levels,CV_8UC1);
	}

	float* feature_ptr=input_feature_data.ptr<float>(0);
	uchar* index_ptr=feature_index.ptr<uchar>(0);

	for(int i=0;i<bag_of_visual_word_.tree_levels;i++)
	{
		//�������ž���
		best_distance=100;
		for(int j=0;j<bag_of_visual_word_.branches;j++)
		{
			//��������
			int now_posiotion=level_start_index+upper_positon*bag_of_visual_word_.branches+j;
			bovw_data=bag_of_visual_word_.kd_tree_data[now_posiotion];

			//�������
			int bovw_data_size=bovw_data.size();
			float distance_now=0,dt;
			for(int k=0;k<bovw_data_size;k++)
			{
				dt=bovw_data[k]-feature_ptr[k];
				distance_now+=dt*dt;
			}

			//�ж����ž���
			if(distance_now<best_distance)
			{
				best_distance=distance_now;
				best_index=j;
			}
		}
		//������������
		index_ptr[i]=best_index;

		//������һ������
		level_start_index=(level_start_index+1)*bag_of_visual_word_.branches;

		upper_index=best_index;//��һ������
		upper_positon=upper_positon*bag_of_visual_word_.branches+best_index;//��һ��������֧�ı��㼶λ��
	}

	return true;
}


//����Ӿ��ʵ��ģ
void TrainBoVWofFeature::getSizeofBoVW(int& tree_levels,int& branches)
{
	tree_levels=bag_of_visual_word_.tree_levels;
	branches=bag_of_visual_word_.branches;
}

//����Ӿ��ʵ���
K_D_tree TrainBoVWofFeature::getBagofVisvalWord()
{
	K_D_tree returnBagofVisvalWord=bag_of_visual_word_;
	return returnBagofVisvalWord;
}

/*********************  private����  **********************/

//����ɼ�ͼ��
bool TrainBoVWofFeature::catchFrame(cv::Mat& input_frame)
{
	if(input_frame.rows<8||input_frame.cols<8)//ͼ���С����
		return false;

	cv::pyrDown(input_frame,frame_now_,cv::Size(input_frame.cols/2,input_frame.rows/2));
	return true;
}

//��������
bool TrainBoVWofFeature::detectFeature()
{
	//1.��frame_now_�����������
	vector<cv::KeyPoint> feature_keypoints;
	feature_detector_->detect(frame_now_,feature_keypoints);//��������
	cv::Mat feature_descriptor;
	feature_extractor_->compute(frame_now_,feature_keypoints,feature_descriptor);//������������
	cv::Mat draw_mat;
	cv::drawKeypoints(frame_now_,feature_keypoints,draw_mat,cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);//��������
	cv::imshow( "keypoints", draw_mat );
	if(feature_keypoints.size()==0)
	{
		cout<<"error: ��ǰͼƬû������!"<<endl;
		return false;
	}

	//2.������������������������
	vector<float> feature_descriptor_ready;
	feature_descriptor_ready.resize(feature_descriptor.cols);//��ȡ��������
	for(int i=0;i<feature_descriptor.rows;i++)
	{
		float* ft_descriptor_ptr=feature_descriptor.ptr<float>(i);
		for(int j=0;j<feature_descriptor.cols;j++)
		{
			feature_descriptor_ready[j]=ft_descriptor_ptr[j];
		}
		samples_of_feature_.push_back(feature_descriptor_ready);//����������
	}
}

bool TrainBoVWofFeature::myK_Means(vector<vector<float>>& samples,int classify_num,vector<vector<float>>& k_meas_result,vector<int>& samples_belones_result)
{
	cout<<"��ǰ������õ�����������"<<samples.size()<<endl;
	//ͨ��k-means����BoVW����ľ�������
	if(samples.size()<classify_num)
	{
		cout<<"eror: k-means����������Ч��"<<endl;
		Sleep(500);
		return false;
	}

	//1.����classify_numȷ��k-means�������ĸ���
	k_meas_result.resize(classify_num);

	//2.��ʼ����������
	for(int i=0;i<classify_num;i++)
	{
		k_meas_result[i].resize(samples[0].size());
	}
	//ͳ��samples�ķ�Χ
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

	//����ͳ�ƽ�����������ʼ���ľ�������
	//  ͳ��min_of_samples��max_of_samples��������ǰ���ɸ�������,��Ϊ����ο���
	vector<float> div_of_coordinate,coordinate_order;
	div_of_coordinate.resize(min_of_samples_size);
	coordinate_order.resize(min_of_samples_size);
	for(int i=0;i<min_of_samples_size;i++)//���㼫��
	{
		div_of_coordinate[i]=max_of_samples[i]-min_of_samples[i];
		coordinate_order[i]=i;
	}
	for(int i=1;i<min_of_samples_size;i++)//�ɴ�С����
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
	//  ���ݾ���ο��ᣬ������������
	vector<float> center_of_coordinate;//�������ϵ�����
	center_of_coordinate.resize(min_of_samples_size);
	for(int i=0;i<min_of_samples_size;i++)
	{
		center_of_coordinate[i]=(max_of_samples[i]+min_of_samples[i])/2;
	}
	for(int i=0;i<divide_nums;i++)
	{
		k_meas_result[2*i]=center_of_coordinate;
		k_meas_result[2*i+1]=center_of_coordinate;
		int different_coordinate=initial_k_means_coordinate[i];//���������
		k_meas_result[2*i][different_coordinate]=center_of_coordinate[different_coordinate]-(max_of_samples[different_coordinate]-min_of_samples[different_coordinate])/4;
		k_meas_result[2*i+1][different_coordinate]=center_of_coordinate[different_coordinate]+(max_of_samples[different_coordinate]-min_of_samples[different_coordinate])/4;
		//�����������
		k_meas_result[2*i][different_coordinate]+=(0.2*rand()/(RAND_MAX+1)-0.1)*(max_of_samples[different_coordinate]-min_of_samples[different_coordinate])/4;
		k_meas_result[2*i+1][different_coordinate]+=(0.2*rand()/(RAND_MAX+1)-0.1)*(max_of_samples[different_coordinate]-min_of_samples[different_coordinate])/4;
	}
	//���classify_numΪ��������Ҫ�������һ����������
	if(classify_num-divide_nums*2==1)
	{
		int different_coordinate=initial_k_means_coordinate[divide_nums];//���������
		k_meas_result[classify_num-1]=center_of_coordinate;
		k_meas_result[classify_num-1][different_coordinate]=center_of_coordinate[different_coordinate]-(max_of_samples[different_coordinate]-min_of_samples[different_coordinate])/4;
		//�����������
		k_meas_result[classify_num-1][different_coordinate]+=(0.2*rand()/(RAND_MAX+1)-0.1)*(max_of_samples[different_coordinate]-min_of_samples[different_coordinate])/4;
	}

	//3.��ʼ����k-means
	vector<vector<float>> k_meas_result_before;//ǰһ�ν��
	float jump_condition;//��������
	for(int iteration_nums=0;iteration_nums<100;iteration_nums++)//����������Ϊ100
	{
		cout<<"k-means����������"<<iteration_nums+1<<endl;
		//���ݾ��룬���¸���������������
		vector<int> samples_belonges;//������������������
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
					distance_to_center=diatance_now;//���µ����ľ���
					belones=j;//������������
				}
			}
			samples_belonges[i]=belones;
		}
		samples_belones_result=samples_belonges;//���������������ֵ
		//�����������������¾�����������
		int k_meas_result_size=k_meas_result.size();
		for(int i=0;i<k_meas_result_size;i++)//�������Ĺ���
		{
			int k_meas_result_i_size=k_meas_result[i].size();
			for(int j=0;j<k_meas_result_i_size;j++)
			{
				k_meas_result[i][j]=0;
			}
		}
		vector<int> samples_nums;//�����������İ�����������
		samples_nums.resize(k_meas_result_size);
		for(int i=0;i<k_meas_result_size;i++)
			samples_nums[i]=0;

		for(int i=0;i<samples_size;i++)//�����ۼ�ֵ
		{
			int samples_i_size=samples[i].size();
			for(int j=0;j<samples_i_size;j++)
			{
				k_meas_result[samples_belonges[i]][j]+=samples[i][j];
			}
			samples_nums[samples_belonges[i]]++;//��������
		}
		for(int i=0;i<k_meas_result_size;i++)//�����ֵ
		{
			if(samples_nums[i]==0)//�����ǰ��������û������SURF����
				continue;
			int k_meas_result_i_size=k_meas_result[i].size();
			for(int j=0;j<k_meas_result_i_size;j++)
			{
				k_meas_result[i][j]=k_meas_result[i][j]/samples_nums[i];
			}
		}

		//4.�ж��Ƿ�������������
		if(iteration_nums==1)//��������������ֵ��ʵΪ����
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
			if(jump_condition<0.000001)//������С������������ֵ
			{
				jump_condition=0.000001;
			}
			cout<<"k-means��ֹ��ֵ��"<<jump_condition<<endl;
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
			cout<<"��ǰk-means������������"<<judge_num<<endl;
			if(judge_num<jump_condition&&iteration_nums>10)//������������
			{
				return true;
			}
		}

		//����ǰʱ��������Ϊǰһʱ������
		k_meas_result_before=k_meas_result;
	}

	return false;
}