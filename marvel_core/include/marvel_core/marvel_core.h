#ifndef MARVEL_H
#define MARVEL_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <iostream>


using namespace std;

#define PI 3.141592
#define RAD2DEG(x) ((x)*180./PI)
#define DEG2RAD(X) ((X)*PI/180.)

enum _tag
{
	tag0,
	tag1,
	tag2,
	tag3,
	tag4,
	tag5,
	tag6,
	tag7,
	tag8,
	tag9,
	
};

string tagFrame[10] = {"tag_0","tag1","tag2","tag3","tag4",
						"tag5","tag6","tag7","tag8","tag9"};

class Marvel
{
public:
	Marvel();
	void Start();
	bool DetectTag(int tagNum);
	//note : Target x ,y ,th on "Tag Frame"
	bool TagLocation(int tagNum, float x, float y, float th); //input x,y : m /th : degree

private:
	void Parameter();
	ros::NodeHandle node_;
	string parentFrame_;
	string childFrame_;
	double angleTolerance_;
	double distanceTolerance_;
	
};

#endif 