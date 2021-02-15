#include "april_slam.h"
#include <ros/ros.h>
int main(int argc,char** argv)
{
	ros::init(argc, argv, "marvel_core");
	// ros::NodeHandle node;
	TagSlam tag;
	// Marvel marvel;
	// marvel.Start();

	while(ros::ok())
	{
		if(tag.TagLocation(tag0,0))	break;
		
	}
	
	return 0;
}