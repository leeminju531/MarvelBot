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

string tagFrame[10] = {"base_footprint","tag1","tag2","tag3","tag4",
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


Marvel::Marvel()
{
	Parameter();
	while(node_.ok())
	DetectTag(tag0);
}
void Marvel::Start()
{

}

void Marvel::Parameter()
{
	if(!node_.getParam("parentFrame",parentFrame_))
		parentFrame_ = "base_footprint";
	if(!node_.getParam("angleTolerance",angleTolerance_))
		angleTolerance_ = DEG2RAD(5);
	if(!node_.getParam("distanceTolerance",distanceTolerance_))
		distanceTolerance_ = 0.1;
	
}

bool Marvel::DetectTag(int tagNum)
{	
	ROS_INFO("tag Num : %d",tagNum);
	cout<<tagFrame[0]<<endl;
	printf("string : %s",tagFrame[0]);
	
	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tfListener(tfBuffer_);
	geometry_msgs::TransformStamped transformStamped_;
	try
	{
		transformStamped_ = tfBuffer_.lookupTransform("odom",tagFrame[tagNum],ros::Time(0),ros::Duration(0.3));
		
	}
	catch(tf2::TransformException &ex)
	{
		ROS_INFO(" Frame not exist ! ");
		return false;
	}
	ROS_INFO(" Frame is exist ! ");
	return true;

	
}

bool Marvel::TagLocation(int tagNum, float x, float y, float th)
{

	/*while loof
	pd control 
	if in tolerance , return true. 
	can't find detection Area -> return false
	*/

	return true;
}

