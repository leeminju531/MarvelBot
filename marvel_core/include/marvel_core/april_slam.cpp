#include "april_slam.h"

TagSlam::TagSlam(double angleTolerance,double distanceTolerance) 
:	tfListener_(tfBuffer_),target_pose_x_(0.0),target_pose_y_(0.0),target_pose_th_(0.0),
	vel_X_(0),vel_Y_(0),vel_Th_(0),Th_(0)
{
	angleTolerance_ = angleTolerance;
	distanceTolerance_ = distanceTolerance;
	parentFrame_ = "base_footprint";

	cmd_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel",10);

}
TagSlam::TagSlam(double angleTolerance,double distanceTolerance,string parentFrame)
:	tfListener_(tfBuffer_)
{
	angleTolerance_ = angleTolerance;
	distanceTolerance_ = distanceTolerance;
	parentFrame_ = parentFrame;

	cmd_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel",10);
}


bool TagSlam::TagDetection(int tagNum)
{
	if(tagNum >= tagSize)
	{
		ROS_WARN("tagNum Out of Index !! ");
		return false;
	}
	try
	{	// the TF broadcaster hz have to pass up at least 3.3hz(1/0.3)
		transformStamped_ = tfBuffer_.lookupTransform(parentFrame_,tagFrame[tagNum],
			ros::Time::now(),ros::Duration(0.3));
	}
	catch(tf2::TransformException &ex)
	{
		ROS_INFO(" Frame not exist ! ");
		return false;
	}
	ROS_INFO(" Frame is exist ! ");
	return true;	
}

bool TagSlam::TagLocation(int tagNum, float tag_pose_x,float tag_pose_y, float tag_pose_th)
{
	if (!TagDetection(tagNum))	return false;

	float detect_pose_x=0,detect_pose_y=0,detect_pose_th=0;
	
	while(ros::ok())
	{
		try
		{
			transformStamped_ = tfBuffer_.lookupTransform(parentFrame_,tagFrame[tagNum],
			ros::Time::now(),ros::Duration(0.3));

		}catch(tf2::TransformException &ex)
		{
			ROS_INFO(" Frame cant leading ! ");
			// target_pose_x = detect_pose_x + tag_pose_x;
			// target_pose_y = detect_pose_y + tag_pose_y;
			// target_pose_th = detect_pose_th + tag_pose_th;
			PredictUpdate();
			PDControl(target_pose_x_,target_pose_y_,target_pose_th_);
			continue;
		}
		PredicInit();
		detect_pose_x = transformStamped_.transform.translation.x;
		detect_pose_y = transformStamped_.transform.translation.y;

		tf::Quaternion quat_temp;
		double hole_r,hole_p,hole_y;
		tf::quaternionMsgToTF(transformStamped_.transform.rotation,quat_temp);
		tf::Matrix3x3(quat_temp).getRPY(hole_r,hole_p,hole_y);
		printf("x : %f || y : %f\n",detect_pose_x,detect_pose_y);
		printf("roll :%f || pitch :%f || yaw :%f\n",hole_r,hole_p,hole_y);
		detect_pose_th = hole_y;

		target_pose_x_ = detect_pose_x + tag_pose_x;
		target_pose_y_ = detect_pose_y + tag_pose_y;
		target_pose_th_ = detect_pose_th + tag_pose_th;
		if(PDControl(target_pose_x_,target_pose_y_,target_pose_th_))	break;
		ros::Duration(0.1).sleep();

	}

	return true;

}

bool TagSlam::TagLocation(int tagNum, float tag_pose_th)
{
	return TagLocation(tagNum, 0.0, 0.0, tag_pose_th);
}

void TagSlam::PredictUpdate()
{
	current_time_ = ros::Time::now();
	double dt = (current_time_ - last_time_).toSec();
	double delta_x = (vel_X_ * cos(Th_) - vel_Y_ * sin(Th_)) * dt;
	double delta_y = (vel_X_ * sin(Th_) + vel_Y_ * cos(Th_)) * dt;
	double delta_th = vel_Th_ * dt;

	Th_ += delta_th;
	target_pose_x_ -= delta_x;
	target_pose_y_ -= delta_y;
	target_pose_th_ -= delta_th;
}

void TagSlam::PredicInit()
{
	vel_X_ = 0;
	vel_Y_ = 0;
	vel_Th_ = 0;
}

//return true In Tolerance Area.
bool TagSlam::PDControl(float target_pose_x,float target_pose_y, float target_pose_th)
{

    vel_.linear.x = 0;
	vel_.angular.z = 0;

    cmd_pub_.publish(vel_);
    //for prediction case
    last_time_ = ros::Time::now();
    vel_X_ = vel_.linear.x;
    vel_Th_ = vel_.angular.z;

    return false;
}

