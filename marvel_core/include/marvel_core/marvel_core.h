#ifndef MARVEL_H
#define MARVEL_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h> 
#include <boost/thread.hpp>
#include <tf2/LinearMath/Quaternion.h>

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
enum map_pose
{
	X,
	Y,
	TH
};

enum _working
{
	Initial,
	Moving,
	AtTargetPosition,
	Back,
	Aborted
};

enum _tag
{
	tag0,
	tag1,
	tag2,
	tag3,
	tag4,
	tag5,
	
};
int tagSize = 6;
string tagFrame[] = {"tag_0","tag_1","tag_2","tag_3","tag_4","tag_5"};

enum _wayPoint
{
	InitWayPoint,
	OneWayPoint,
	TwoWayPoint,
	ThreeWayPoint

};
int wayPointSize = 4;



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Marvel
{
public:
	Marvel();
	void Start();
	bool DetectTag(int tagNum);
	//note : Target x ,y ,th on "Tag Frame"
	bool TagLocation(int tagNum); //input x,y : m /th : degree

private:
	void ParameterSet();
	void LossLocalization();
	void WaitGoal();
	void SendGoal(int target_tag);
	void doneCB();
	void activeCB();
	void feedbackCB();
	void PrintActionState();
	ros::NodeHandle node_;
	string parentFrame_;
	string childFrame_;
	MoveBaseClient ac_ ;
	double angleTolerance_;
	double distanceTolerance_;
	tf2_ros::Buffer tfBuffer_;
	geometry_msgs::TransformStamped transformStamped_;
	int flag_ ;
	float target_tag_poseX_,target_tag_poseY_,target_tag_poseTH_;
	int target_tag_;
	float tag_location[tagSize][3]_;
	float loss_tag_location[tagSize][3]_;
	float wayPoint[wayPointSize][3]_;

};

#endif 