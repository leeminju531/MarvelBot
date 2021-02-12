// #include "marvel_core.h"

// Marvel::Marvel() : ac_("move_base",true)
// {
// 	ParameterSet();
// 	flag_ = Initial;
// }


// void Marvel::Start()
// {
	
// 	LossLocalization();
		

	



// 	// while(node_.ok())
// 	// {
// 	// 	switch(flag_){
// 	// 	case Initial:
// 	// 		if(DetectTag(tag0))
// 	// 		{
// 	// 			if(TagLocation(tag0,0,0,0))
// 	// 			{
// 	// 				// initial pose topic to amcl
// 	// 			}
// 	// 		}
// 	// 		break;
// 	// 	case Moving:


// 	// 		break;
// 	// 	case AtTargetPosition:


// 	// 		break;
// 	// 	case Back:


// 	// 		break;
// 	// 	case Aborted:


// 	// 		break;
// 	// 	}

// 	// }
	
// }

// void Marvel::LossLocalization()
// {
// 	do{
// 		if(DetectTag(tag0)){
// 			TagLocation(tag0,loss_tag_location[tag0][X],
// 				loss_tag_location[tag0][Y],loss_tag_location[tag0][TH]);
// 			break;
// 		}else if(DetectTag(tag1)){
// 			TagLocation(tag1,loss_tag_location[tag1][X],
// 				loss_tag_location[tag1][Y],loss_tag_location[tag1][TH]);
// 			break;
// 		}else if(DetectTag(tag2)){
// 			TagLocation(tag2,loss_tag_location[tag2][X],
// 				loss_tag_location[tag2][Y],loss_tag_location[tag2][TH]);
// 			break;
// 		}else if(DetectTag(tag3)){
// 			TagLocation(tag3,loss_tag_location[tag3][X],
// 				loss_tag_location[tag3][Y],loss_tag_location[tag3][TH]);
// 			break;
// 		}else if(DetectTag(tag4)){
// 			TagLocation(tag4,loss_tag_location[tag4][X],
// 				loss_tag_location[tag4][Y],loss_tag_location[tag4][TH]);
// 			break;
// 		}else if(DetectTag(tag5)){
// 			TagLocation(tag5,loss_tag_location[tag5][X],
// 				loss_tag_location[tag5][Y],loss_tag_location[tag5][TH]);
// 			break;
// 		}else{
// 			ROS_INFO("Robot Loss Localization, Drag To Tag Detection Area ");
// 			/*
// 			publish topic to UI to notify localization fail
// 			*/
// 			ros::Duration(3).sleep();
// 		}

// 	} while(true);

// 	SendGoal(tag0,InitWayPoint);
// }






// void Marvel::WaitGoal()
// {

// }
// void Marvel::SendGoal(int target_tag,int wayPointNum)
// {
// 	this->target_tag_ = target_tag;
// 	ROS_INFO("Wait for action server");
// 	while(!ac_.waitForServer(ros::Duration(3.0)))	ROS_INFO("Waiting for the move_base Action Server");
	
// 	move_base_msgs::MoveBaseGoal goal;
// 	goal.target_pose.header.frame_id = "map";
// 	goal.target_pose.header.stamp = ros::Time::now();
// 	goal.target_pose.pose.position.x = wayPoint[wayPointNum][X]_;
// 	goal.target_pose.pose.position.y = wayPoint[wayPointNum][Y]_;

// 	tf2::Quaternion q;
// 	q.setRPY(0,0,DEG2RAD(wayPoint[wayPointNum][TH]_));
// 	goal.target_pose.pose.orientation.x = q.x();
// 	goal.target_pose.pose.orientation.y = q.y();
// 	goal.target_pose.pose.orientation.z = q.z();
// 	goal.target_pose.pose.orientation.w = q.w();
// 	ac_.sendGoal(goal , boost::bind(&MoveAction::doneCB, this ), 
// 					boost::bind(&MoveAction::activeCB,this),boost::bind(&MoveAction::feedbackCB,this ));

// 	ROS_INFO("Sending goal to action server");
// 	ROS_INFO(" ");
// }

// void Marvel::activeCB()
// {
// 	ROS_INFO("Goal reached to Action Server");
//  	ROS_INFO("Client node is Active State ");
// }

// void Marvel::feedbackCB()
// {
// 	PrintActionState();
// 	if(DetectTag(target_tag_))

// }


// void Marvel::doneCB()
// {
// 	ROS_INFO("doneCB function Called");
// 	PrintActionState();

// 	LossLocalization();

// 	// if( DetectTag(target_tag_) ){
// 	// 	if(!TagLocation(target_tag_))
// 	// 		ROS_WARN("Can't Localization Based on Tag !! ");
// 	// }
// 	// else	// lost localization.
// 	// {
// 	// 	ROS_WARN("Nivigation Ended, But Can't Find Target Tag !! ");
// 	// 	/*
// 	// 		topic to UI to notify
// 	// 	*/
// 	// }

// }

// void Marvel::PrintActionState()
// {
// 	if(ac_.getState() == actionlib::SimpleClientGoalState::PENDING){ 
// 		ROS_INFO("PENDING State");
// 	}
// 	//The goal is currently being processed by the action server
// 	else if(ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE){
// 		ROS_INFO("ACTIVE State");
// 	}
// 	//  The goal was achieved successfully by the action server (Terminal State)
// 	else if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
// 		ROS_INFO("SUCCEEDED State");
// 	}
// 	// The goal was aborted during execution by the action server (Terminal State)
// 	else if(ac_.getState() == actionlib::SimpleClientGoalState::ABORTED){
// 		ROS_INFO("ABORTED State");
// 	}
// 	//the goal received a cancel request after it started executing
// 	// 실행을 시작한 후 취소 요청을 받았고 이후 실행을 완료 (Terminal State)
// 	else if(ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
// 		ROS_INFO("PREEMPTED State");
// 	}
// 	//목표를 달성할 수 없거나 잘못되었기 때문에 수행 서버에서 목표를 처리하지 않고 거부(Terminal State)
// 	else if(ac_.getState() == actionlib::SimpleClientGoalState::REJECTED){
// 		ROS_INFO("REJECTED State");
// 	}
// 	// The goal received a cancel request before it started executing 
// 	// and was successfully cancelled (Terminal State)
// 	else if(ac_.getState() == actionlib::SimpleClientGoalState::RECALLED){
// 		ROS_INFO("RECALLED State");
// 	}
// 	// An action client can determine that a goal is LOST.
// 	// The goal's state. Returns LOST if this SimpleActionClient isn't tracking a goal. 
// 	else if(ac_.getState() == actionlib::SimpleClientGoalState::LOST){
// 		ROS_INFO("LOST State");
// 	}
// }
// void Marvel::ParameterSet()
// {
// 	if(!node_.getParam("parentFrame",parentFrame_))
// 		parentFrame_ = "base_footprint";
// 	if(!node_.getParam("angleTolerance",angleTolerance_))
// 		angleTolerance_ = DEG2RAD(5);
// 	if(!node_.getParam("distanceTolerance",distanceTolerance_))
// 		distanceTolerance_ = 0.1;

// 	if(!node_.getParam("map_position_x0",wayPoint[INIT][X]_))
// 		wayPoint[INIT][X]_ = 0;
// 	if(!node_.getParam("map_position_y0",wayPoint[INIT][Y]_))
// 		wayPoint[INIT][Y]_ = 0;
// 	if(!node_.getParam("map_position_th0",wayPoint[INIT][TH]_))
// 		wayPoint[INIT][TH]_ = 0;

// 	if(!node_.getParam("map_position_x1",wayPoint[ONE][X]_))
// 		wayPoint[ONE][X]_ = 0;
// 	if(!node_.getParam("map_position_y1",wayPoint[ONE][Y]_))
// 		wayPoint[ONE][Y]_ = 0;
// 	if(!node_.getParam("map_position_th1",wayPoint[ONE][TH]_))
// 		wayPoint[ONE][TH]_ = 0;

// 	if(!node_.getParam("map_position_x2",wayPoint[TWO][X]_))
// 		wayPoint[TWO][X]_ = 0;
// 	if(!node_.getParam("map_position_y2",wayPoint[TWO][Y]_))
// 		wayPoint[TWO][Y]_ = 0;
// 	if(!node_.getParam("map_position_th2",wayPoint[TWO][TH]_))
// 		wayPoint[TWO][TH]_ = 0;

// 	if(!node_.getParam("map_position_x3",wayPoint[THREE][X]_))
// 		wayPoint[THREE][X]_ = 0;
// 	if(!node_.getParam("map_position_y3",wayPoint[THREE][Y]_))
// 		wayPoint[THREE][Y]_ = 0;
// 	if(!node_.getParam("map_position_th3",wayPoint[THREE][TH]_))
// 		wayPoint[THREE][TH]_ = 0;


// 	if(!node_.getParam("tag0_position_x",tag_location[tag0][X]_))
// 		tag_location[tag0][X]_ = 0;
// 	if(!node_.getParam("tag0_position_y",tag_location[tag0][Y]_))
// 		tag_location[tag0][Y]_ = 0;
// 	if(!node_.getParam("tag0_position_th",tag_location[tag0][TH]_))
// 		tag_location[tag0][TH]_ = 0;

// 	if(!node_.getParam("tag1_position_x",tag_location[tag1][X]_))
// 		tag_location[tag1][X]_ = 0;
// 	if(!node_.getParam("tag1_position_y",tag_location[tag1][Y]_ ))
// 		tag_location[tag1][Y]_ = 0;
// 	if(!node_.getParam("tag1_position_th",tag_location[tag1][TH]_))
// 		tag_location[tag1][TH]_ = 0;

// 	if(!node_.getParam("tag2_position_x",tag_location[tag2][X]_))
// 		tag_location[tag2][X]_ = 0;
// 	if(!node_.getParam("tag2_position_y",tag_location[tag2][Y]_))
// 		tag_location[tag2][Y]_ = 0;
// 	if(!node_.getParam("tag2_position_th",tag_location[tag2][TH]_))
// 		tag_location[tag2][TH]_ = 0;

// 	if(!node_.getParam("tag3_position_x",tag_location[tag3][X]_))
// 		tag_location[tag3][X]_ = 0;
// 	if(!node_.getParam("tag3_position_y",tag_location[tag3][Y]_))
// 		tag_location[tag3][Y]_ = 0;
// 	if(!node_.getParam("tag3_position_th",tag_location[tag3][TH]_))
// 		tag_location[tag3][TH]_ = 0;

// 	if(!node_.getParam("tag4_position_x",tag_location[tag4][X]_))
// 		tag_location[tag4][X]_ = 0;
// 	if(!node_.getParam("tag4_position_y",tag_location[tag4][Y]_))
// 		tag_location[tag4][Y]_ = 0;
// 	if(!node_.getParam("tag4_position_th",tag_location[tag4][TH]_))
// 		tag_location[tag4][TH]_ = 0;

// 	if(!node_.getParam("tag5_position_x",tag_location[tag5][X]_))
// 		tag_location[tag5][X]_ = 0;
// 	if(!node_.getParam("tag5_position_y",tag_location[tag5][Y]_))
// 		tag_location[tag5][Y]_ = 0;
// 	if(!node_.getParam("tag5_position_th",tag_location[tag5][TH]_))
// 		tag_location[tag5][TH]_ = 0;




// }

// bool Marvel::DetectTag(int tagNum)
// {	
// 	tf2_ros::TransformListener tfListener(tfBuffer_);
// 	try
// 	{
// 		transformStamped_ = tfBuffer_.lookupTransform(parentFrame_,tagFrame[tagNum],ros::Time(0),ros::Duration(0.3));
		
// 	}
// 	catch(tf2::TransformException &ex)
// 	{
// 		ROS_INFO(" Frame not exist ! ");
// 		return false;
// 	}
// 	ROS_INFO(" Frame is exist ! ");
// 	return true;	
// }

// bool Marvel::DetectTag(int tagNum,float Area)
// {	
// 	tf2_ros::TransformListener tfListener(tfBuffer_);
// 	try
// 	{
// 		transformStamped_ = tfBuffer_.lookupTransform(parentFrame_,tagFrame[tagNum],ros::Time(0),ros::Duration(0.3));
		
// 	}
// 	catch(tf2::TransformException &ex)
// 	{
// 		ROS_INFO(" Frame not exist ! ");
// 		return false;
// 	}
// 	ROS_INFO(" Frame is exist ! ");
// 	return true;	
// }

// bool Marvel::TagLocation(int tagNum,float x, float y, float th)
// {
// 	this -> target_tag_poseX_ = x;
// 	this -> target_tag_poseY_ = y;
// 	this -> target_tag_poseTH_ = th;
// 	/*while loof
// 	pd control 
// 	if in tolerance , return true. 
// 	can't find detection Area -> return false
// 	*/

// 	return true;
// }