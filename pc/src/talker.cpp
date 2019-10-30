#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

int main(int argc,char** argv)
{
	ros::init(argc,argv,"talk");
	ros::NodeHandle n;
	ros::Publisher talker=n.advertise<sensor_msgs::PointCloud2>("points",100);
	ros::Rate rate(1);

	//pc.header.seq=111;
	//pc.header.stamp=ros::Time::now();
	//pc.header.frame_id="en?";
	int count=0;//依次读如11个txt文件
	while(count<12)
	{
		string filename="/home/sean/Desktop/point/dream";
		filename=filename+to_string(count)+".txt";
		ifstream in(filename);//将点集写入缓存
		int partpoint;//一个数字接一个数字的存入vector
		vector<int> iv;
		while(in>>partpoint)
		{
			iv.push_back(partpoint);
		}

		sensor_msgs::PointCloud2 pc;
		
		pc.header.seq=count;
		pc.header.stamp=ros::Time::now();//写入时间戳
		pc.header.frame_id=filename;

		pc.height=1;//为无序数组
		pc.width=(iv.size()/2);//点的个数

		pc.fields.resize(2);//Field数组含有两个成员，x与y
		
		pc.fields[0].name="x";
		pc.fields[0].offset=0;
		pc.fields[0].datatype=5;//我认为txt里数字格式是uint8 INT32
		pc.fields[0].count=1;//一个点内含有两个数字

		pc.fields[1].name="y";
                pc.fields[1].offset=0;
                pc.fields[1].datatype=5;//我认为txt里数字格式是uint8 INT32
                pc.fields[1].count=1;//一个点内含有两个数字


		pc.is_bigendian=false;//数据不是夸大的
		pc.point_step=32;//一个点占的比特数应该是32
		pc.row_step=(32*iv.size()/2);//一行占的比特数应该是32*（txt文本中的数字个数）
		
		pc.data.resize(iv.size());
		//输入data中的数据，就是将vector中的数字依次存入
		for(int i=0;i<iv.size();i++)
		{
			pc.data[i]=iv[i];
		}

		pc.is_dense=true;//不含无效数据
	
		ROS_INFO("seq is %d",pc.header.seq);
		
		talker.publish(pc);

		rate.sleep();

		count++;

	}

	return 0;
}
