#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

// #include <move_base_msgs/MoveBaseActionGoal.h>
// #include <move_base_msgs/MoveBaseActionFeedback.h>
// #include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseAction.h> 
#include <boost/thread.hpp>

//#include <actionlib_msgs/GoalID.h>
//#include <actionlib_msgs/GoalStatusArray>

#include <tf2/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


//input parameter
float x = 1,y = 0, th = 0; // th : rad

int main(int argc,char **argv)
{
	ros::init(argc,argv,"action_nav");
	// sending goal
	ROS_INFO("Wait for action server");
	MoveBaseClient ac("move_base",true);
	ac.waitForServer();
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;

	tf2::Quaternion q;
	q.setRPY(0,0,th);
	goal.target_pose.pose.orientation.x = q.x();
	goal.target_pose.pose.orientation.y = q.y();
	goal.target_pose.pose.orientation.z = q.z();
	goal.target_pose.pose.orientation.w = q.w();

	ac.sendGoal(goal);
	ROS_INFO("Send goal to action server");

}