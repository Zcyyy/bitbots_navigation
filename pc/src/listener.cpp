#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>

using namespace std;

void callback(sensor_msgs::PointCloud2 msg)
{
	ROS_INFO("seq is %d,stamp is %.5f,frame_id is ",msg.header.seq,msg.header.stamp.toSec());
	ROS_INFO("===============");
	ROS_INFO("I heard :%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5],msg.data[6],msg.data[7],msg.data[8],msg.data[9]);
       //ROS_INFO("data is %d %d %d",msg.data[0],msg.data[1000],msg.data[500]);
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"listen");
	ros::NodeHandle n;
	ros::Subscriber listener=n.subscribe("points",100,callback);
	ros::spin();
	return 0;
}
