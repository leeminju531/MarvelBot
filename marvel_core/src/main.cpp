#include "april_slam.cpp"
#include <ros/ros.h>
int main(int argc,char** argv)
{

	ros::init(argc, argv, "marvel_core");
	
	ros::NodeHandle nh;
	TagSlam tag(nh);
	// Marvel marvel;
	// marvel.Start();

	// while(ros::ok())
	// {
		
		
	
	
	return 0;
}