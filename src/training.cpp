
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

using namespace std;


//global vars
float lastPointSet;
nav_msgs::Odometry current_pose;
float current_heading;
string filename;


//get current position of drone
void logPosition()
{
	std::ofstream outPos(filename.c_str(), std::ios::app);
  	outPos << current_pose.pose.pose.position.x << " " << current_pose.pose.pose.position.y << " " << current_pose.pose.pose.position.z  << " " << current_heading << "\n";
  	ROS_INFO("x: %f y: %f z: %f psi: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z, current_heading);
  	outPos.close();
}
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose = *msg;
  float q0 = current_pose.pose.pose.orientation.w;
  float q1 = current_pose.pose.pose.orientation.x;
  float q2 = current_pose.pose.pose.orientation.y;
  float q3 = current_pose.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  current_heading = -psi*(180/M_PI) + 90;
  //ROS_INFO("Current Heading %f ", current_heading);

  
  // ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
}
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	sensor_msgs::Joy joyMsg;
	joyMsg = *msg;
	if( ros::Time::now().toSec() - lastPointSet > 1.5 && joyMsg.axes[7] == 1)
	{
		lastPointSet =  ros::Time::now().toSec();
		cout<< "logging data" << endl;
		logPosition();
	}
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    ros::Subscriber currentPos = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, pose_cb);
    ros::Subscriber logPosition = nh.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    if (!nh.hasParam("/training/filename"))
	{
	ROS_INFO("No param named 'filename'");
	}else{
	string path;
	string file;
	ros::param::get("/training/filename", file);
	ros::param::get("/training/path", path);
	filename = path + file;
	cout << filename << endl;
	ROS_INFO("File name loaded %s ", filename.c_str());
	}

    ros::spin();

    return 0;
}