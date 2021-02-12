#ifndef APRIL_H
#define APRIL_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <iostream>

#define PI 3.141592
#define RAD2DEG(x) ((x)*180./PI)
#define DEG2RAD(X) ((X)*PI/180.)

using namespace std;

enum _tag
{
	tag0,
	tag1,
	tag2,
	tag3,
	tag4,
	tag5,
	
};
int tagSize =6;
string tagFrame[] = {"tag_0","tag_1","tag_2","tag_3","tag_4","tag_5"};

class TagSlam
{
public:
	TagSlam(double distanceTolerance, double angleTolerance);
	TagSlam(double distanceTolerance, double angleTolerance, string parentFrame);
	bool TagDetection(int tagNum);
	bool TagLocation(int tagNum, float tag_pose_x,float tag_pose_y, float tag_pose_th);
	bool TagLocation(int tagNum, float tag_pose_th);
	bool PDControl(float target_pose_x,float target_pose_y, float target_pose_th);

private:
	ros::NodeHandle node_;
	string parentFrame_;

	tf2_ros::Buffer tfBuffer_; // why have to define here??
	geometry_msgs::TransformStamped transformStamped_;
	tf2_ros::TransformListener tfListener_;

	double angleTolerance_;
	double distanceTolerance_;
	ros::Publisher cmd_pub;
	geometry_msgs::Twist vel;
	
};


#endif