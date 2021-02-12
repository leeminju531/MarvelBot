#include "april_slam.h"
#include <ros/ros.h>
int main(int argc,char** argv)
{
	ros::init(argc, argv, "marvel_core");
	// ros::NodeHandle node;
	TagSlam tag(0.1,5);
	// Marvel marvel;
	// marvel.Start();

	while(ros::ok())
	{
		tag.TagLocation(tag0,0);
		ros::spinOnce();
	}
	
	return 0;
}