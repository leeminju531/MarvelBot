#include "april_slam.cpp"
#include <ros/ros.h>
int main(int argc,char** argv)
{
	ros::init(argc, argv, "marvel_core");
	// ros::NodeHandle node;
	TagSlam tag;
	// Marvel marvel;
	// marvel.Start();

	// while(ros::ok())
	// {
		tag.tempPub();
		
	
	
	return 0;
}