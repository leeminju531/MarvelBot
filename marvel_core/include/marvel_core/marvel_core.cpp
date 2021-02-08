#include "marvel_core.h"

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
	
	// tf2_ros::Buffer tfBuffer_;
	// tf2_ros::TransformListener tfListener(tfBuffer_);
	// geometry_msgs::TransformStamped transformStamped_;
	// try
	// {
	// 	transformStamped_ = tfBuffer_.lookupTransform("odom","base_footprint",ros::Time(0));
		
	// }
	// catch(tf2::TransformException &ex)
	// {
	// 	ROS_INFO(" Frame not exist ! ");
	// 	return false;
	// }
	// ROS_INFO(" Frame is exist ! ");
	// return true;

	tf2_ros::Buffer tfBuffer_;
	geometry_msgs::TransformStamped transformStamped_;
	tf2_ros::TransformListener tfListener(tfBuffer_);
	while(node_.ok()){
		try
		{
			transformStamped_ = tfBuffer_.lookupTransform("odom","base_footprint",ros::Time(0));
			
		}
		catch(tf2::TransformException &ex)
		{
			ROS_INFO("Frame not exist ! ");
			
		}
		ROS_INFO(" Frame is exist ! ");
	}
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

