#include "marvel_core.h"

int main(int argc,char** argv)
{
	ros::init(argc, argv, "marvel_core");
	Marvel marvel;
	marvel.Start();
	// tf2_ros::Buffer tfBuffer_;
	// geometry_msgs::TransformStamped transformStamped_;
	// tf2_ros::TransformListener tfListener(tfBuffer_);
	// while(n.ok()){
	// 	try
	// 	{
	// 		transformStamped_ = tfBuffer_.lookupTransform("odom","base_footprint",ros::Time(0));
			
	// 	}
	// 	catch(tf2::TransformException &ex)
	// 	{
	// 		ROS_INFO("Frame not exist ! ");
			
	// 	}
	// 	ROS_INFO(" Frame is exist ! ");
	// }
	
	

	return 0;
}