#include "april_slam.h"

TagSlam::TagSlam(double angleTolerance,double distanceTolerance) 
:	tfListener_(tfBuffer_)
{
	angleTolerance_ = angleTolerance;
	distanceTolerance_ = distanceTolerance;
	parentFrame_ = "base_footprint";

	cmd_pub = node_.advertise<geometry_msgs::Twist>("cmd_vel",10);

}
TagSlam::TagSlam(double angleTolerance,double distanceTolerance,string parentFrame)
:	tfListener_(tfBuffer_)
{
	angleTolerance_ = angleTolerance;
	distanceTolerance_ = distanceTolerance;
	parentFrame_ = parentFrame;

	cmd_pub = node_.advertise<geometry_msgs::Twist>("cmd_vel",10);
}

bool TagSlam::TagDetection(int tagNum)
{
	if(tagNum >= tagSize)
	{
		ROS_WARN("tagNum Out of Index !! ");
		return false;
	}
	// tf2_ros::Buffer tfBuffer_;
	// geometry_msgs::TransformStamped transformStamped_;
	 //tf2_ros::TransformListener tfListener(tfBuffer_);
	try
	{
		// the TF broadcaster hz have to pass up at least 3.3hz(1/0.3)
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
	float target_pose_x,target_pose_y,target_pose_th;
	while(ros::ok())
	{
		// tf2_ros::Buffer tfBuffer_;
		// geometry_msgs::TransformStamped transformStamped_;
		// tf2_ros::TransformListener tfListener(tfBuffer_);
		try
		{
			transformStamped_ = tfBuffer_.lookupTransform(parentFrame_,tagFrame[tagNum],
			ros::Time::now(),ros::Duration(0.3));

		}catch(tf2::TransformException &ex)
		{
			ROS_INFO(" Frame cant leading ! ");
			target_pose_x = detect_pose_x + tag_pose_x;
			target_pose_y = detect_pose_y + tag_pose_y;
			target_pose_th = detect_pose_th + tag_pose_th;
			PDControl(target_pose_x,target_pose_y,target_pose_th);
			continue;
		}
		detect_pose_x = transformStamped_.transform.translation.x;
		detect_pose_y = transformStamped_.transform.translation.y;

		tf::Quaternion quat_temp;
		double hole_r,hole_p,hole_y;
		tf::quaternionMsgToTF(transformStamped_.transform.rotation,quat_temp);
		tf::Matrix3x3(quat_temp).getRPY(hole_r,hole_p,hole_y);
		printf("x : %f || y : %f\n",detect_pose_x,detect_pose_y);
		printf("roll :%f || pitch :%f || yaw :%f\n",hole_r,hole_p,hole_y);
		detect_pose_th = hole_y;

		target_pose_x = detect_pose_x + tag_pose_x;
		target_pose_y = detect_pose_y + tag_pose_y;
		target_pose_th = detect_pose_th + tag_pose_th;
		if(PDControl(target_pose_x,target_pose_y,target_pose_th))	break;
		ros::Duration(0.1).sleep();


	}

	return true;

}

bool TagSlam::TagLocation(int tagNum, float tag_pose_th)
{
	return TagLocation(tagNum, 0.0, 0.0, tag_pose_th);
}

//return true In Tolerance Area.
bool TagSlam::PDControl(float target_pose_x,float target_pose_y, float target_pose_th)
{
    vel.linear.x = 0;
	vel.angular.z = 0;

    cmd_pub.publish(vel);
    return false;
}