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
/* 
    1. Initial : navigation movement always start at Initial State.
 				localization based on tag0(intial position)
				and wait until arrive navigation topic.

	2. Moving : move when arrive navigation topic. 
				navigate to targetPosition

	3. AtTargetPosition : when robot arrived target position, wait until someone UI 'ok' touch or countdown.

	4. Back : navigate to initial position

    5. Aborted : when robot is aborted, stop 
   				and publish topic to notify aborted state.
   				alive when robot detect tag then, navigate to Initial position 


*/
enum _working
{
	Initial,
	Moving,
	AtTargetPosition,
	Back,
	Aborted
}
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



string tagFrame[10] = {"tag_0","tag_1","tag_2","tag_3","tag_4",
						"tag_5","tag_6","tag_7","tag_8","tag_9"};

class Marvel
{
public:
	Marvel();
	void Start();
	bool DetectTag(int tagNum);
	//note : Target x ,y ,th on "Tag Frame"
	bool TagLocation(int tagNum, float x, float y, float th); //input x,y : m /th : degree

private:
	void ParameterSet();
	void WaitGoal();
	ros::NodeHandle node_;
	string parentFrame_;
	string childFrame_;
	double angleTolerance_;
	double distanceTolerance_;
	tf2_ros::Buffer tfBuffer_;
	geometry_msgs::TransformStamped transformStamped_;
	int flag_ ;
};

#endif 