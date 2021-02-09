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




class MoveAction
{
public:
	MoveAction() : ac_("move_base",true)
	{
		
	}
	~MoveAction()
	{
		
	}
	void SendGoal(float x, float y, float th);
	void doneCB();
	void activeCB();
	void feedbackCB();
private:
	MoveBaseClient ac_ ;

	
};
//&MyNode::activeCB ,&MyNode::feedbackCB );
		//when i used result + active func
//ac_.sendGoal(goal , boost::bind(&MyNode::doneCB, this, _1, _2), Client::SimpleActiveCallback(),Client::SimpleFeedbackCallback());
//&MyNode::activeCB ,&MyNode::feedbackCB );
// ac_->sendGoal(goal , boost::bind(&MoveAction::doneCB, this ), boost::bind(&MoveAction::activeCB,this),boost::bind(&MoveAction::feedbackCB,this ));

void MoveAction::SendGoal(float x, float y, float th)
{
	ROS_INFO("Wait for action server");
	while(!ac_.waitForServer(ros::Duration(3.0)))	ROS_INFO("Waiting for the move_base Action Server");
	

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
	ac_.sendGoal(goal , boost::bind(&MoveAction::doneCB, this ), 
					boost::bind(&MoveAction::activeCB,this),boost::bind(&MoveAction::feedbackCB,this ));

	//ac_.sendGoal(goal);
	ROS_INFO("Sending goal to action server");
	ROS_INFO(" ");
	//ac_.waitForResult(ros::Duration(10.0));
}
int cnt=0;
int temp = 20;

void MoveAction::activeCB()
{
	ROS_INFO("Active Function Called");
 	ROS_INFO("Goal reached to Action Server");
 	ROS_INFO("Client node is Active State ");
 	if(ac_.getState() == actionlib::SimpleClientGoalState::PENDING){ 
		ROS_INFO("PENDING State");
	}
	//The goal is currently being processed by the action server
	else if(ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE){
		ROS_INFO("ACTIVE State");
	}
	//the goal received a cancel request after it started executing
	// 실행을 시작한 후 취소 요청을 받았고 이후 실행을 완료 (Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
		ROS_INFO("PREEMPTED State");
	}
	//  The goal was achieved successfully by the action server (Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("SUCCEEDED State");
	}
	// The goal was aborted during execution by the action server (Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::ABORTED){
		ROS_INFO("ABORTED State");
	}
	//목표를 달성할 수 없거나 잘못되었기 때문에 수행 서버에서 목표를 처리하지 않고 거부(Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::REJECTED){
		ROS_INFO("REJECTED State");
	}
	// The goal received a cancel request before it started executing 
	// and was successfully cancelled (Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::RECALLED){
		ROS_INFO("RECALLED State");
	}
	// An action client can determine that a goal is LOST.
	// The goal's state. Returns LOST if this SimpleActionClient isn't tracking a goal. 
	else if(ac_.getState() == actionlib::SimpleClientGoalState::LOST){
		ROS_INFO("LOST State");
	}
 	ROS_INFO("==============================");
 	cnt++;
 	ac_.cancelGoal();
 	ROS_INFO("cancelGoal Call");
 	if(cnt > temp)	{
		ac_.cancelGoal();
		ROS_INFO("cancelGoal Call");
	}
}
void MoveAction::feedbackCB()
{
	ROS_INFO("feedbackCB function called");
	// The goal has yet to be processed by the action server
	if(ac_.getState() == actionlib::SimpleClientGoalState::PENDING){ 
		ROS_INFO("PENDING State");
	}
	//The goal is currently being processed by the action server
	else if(ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE){
		ROS_INFO("ACTIVE State");
	}
	//the goal received a cancel request after it started executing
	// 실행을 시작한 후 취소 요청을 받았고 이후 실행을 완료 (Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
		ROS_INFO("PREEMPTED State");
	}
	//  The goal was achieved successfully by the action server (Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("SUCCEEDED State");
	}
	// The goal was aborted during execution by the action server (Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::ABORTED){
		ROS_INFO("ABORTED State");
	}
	//목표를 달성할 수 없거나 잘못되었기 때문에 수행 서버에서 목표를 처리하지 않고 거부(Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::REJECTED){
		ROS_INFO("REJECTED State");
	}
	// The goal received a cancel request before it started executing 
	// and was successfully cancelled (Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::RECALLED){
		ROS_INFO("RECALLED State");
	}
	// An action client can determine that a goal is LOST.
	// The goal's state. Returns LOST if this SimpleActionClient isn't tracking a goal. 
	else if(ac_.getState() == actionlib::SimpleClientGoalState::LOST){
		ROS_INFO("LOST State");
	}

	ROS_INFO("--------------------------");
	cnt++;
	if(cnt > temp)	{
		ac_.cancelGoal();
		ROS_INFO("cancelGoal Call");
	}
}
void MoveAction::doneCB()
{
	ROS_INFO("doneCB function called");
	// The goal has yet to be processed by the action server
	if(ac_.getState() == actionlib::SimpleClientGoalState::PENDING){ 
		ROS_INFO("PENDING State");
	}
	//The goal is currently being processed by the action server
	else if(ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE){
		ROS_INFO("ACTIVE State");
	}
	//  The goal was achieved successfully by the action server (Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("SUCCEEDED State");
	}
	// The goal was aborted during execution by the action server (Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::ABORTED){
		ROS_INFO("ABORTED State");
	}
	//the goal received a cancel request after it started executing
	// 실행을 시작한 후 취소 요청을 받았고 이후 실행을 완료 (Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
		ROS_INFO("PREEMPTED State");
	}
	//목표를 달성할 수 없거나 잘못되었기 때문에 수행 서버에서 목표를 처리하지 않고 거부(Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::REJECTED){
		ROS_INFO("REJECTED State");
	}
	// The goal received a cancel request before it started executing 
	// and was successfully cancelled (Terminal State)
	else if(ac_.getState() == actionlib::SimpleClientGoalState::RECALLED){
		ROS_INFO("RECALLED State");
	}
	// An action client can determine that a goal is LOST.
	// The goal's state. Returns LOST if this SimpleActionClient isn't tracking a goal. 
	else if(ac_.getState() == actionlib::SimpleClientGoalState::LOST){
		ROS_INFO("LOST State");
	}
	cnt++;
	if(cnt > temp)	{
		ac_.cancelGoal();
		ROS_INFO("cancelGoal Call");
	}
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"action_nav");
	MoveAction move_action;
	move_action.SendGoal(2,0,0);
	ros::spin();
	
}