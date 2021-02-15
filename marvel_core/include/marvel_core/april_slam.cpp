#include "april_slam.h"

TagSlam::TagSlam() 
:	tfListener_(tfBuffer_),target_pose_x_(0.0),target_pose_y_(0.0),target_pose_th_(0.0),
	vel_X_(0),vel_Y_(0),vel_Th_(0),Th_(0)
{
	parentFrame_ = "base_footprint";
	PdParamSet();
	
	cmd_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel",10);
}

void TagSlam::PdParamSet()
{
	distance_Tolerance_ = 0.1;
	angle_Tolerance_ = DEG2RAD(5);
	p_distance_gain_ = 0.1;
	d_distance_gain_ = 0.1;
	p_angle_gain_ = 0.1;
	d_angle_gain_ = 0.1;
	pd_flag_ = Point_Angular;
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
			ROS_INFO(" Frame can't leading ! ");
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
		// printf("x : %f || y : %f\n",detect_pose_x,detect_pose_y);
		// printf("roll :%f || pitch :%f || yaw :%f\n",hole_r,hole_p,hole_y);
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
	//target_pose_th_ -= delta_th;
}

void TagSlam::PredicInit()
{
	vel_X_ = 0;
	vel_Y_ = 0;
	vel_Th_ = 0;
	Th_=0;
}

//return true In Tolerance Area.
bool TagSlam::PDControl(float target_pose_x,float target_pose_y, float target_pose_th)
{

    float x = target_pose_x;
    float y = target_pose_y;
	switch(pd_flag_)
	{
		case Point_Angular:
			// only angular Vel
			cur_angle_ = atan2(y,x);
			cur_distance_ = 0;

			vel_.linear.x = p_distance_gain_ * cur_distance_ + d_distance_gain_ * before_distance_;
			vel_.angular.z = p_angle_gain_ * cur_angle_ + d_angle_gain_ * before_angle_;
			before_distance_ = cur_distance_;
			before_angle_ = cur_angle_;

			if(abs(cur_angle_) < angle_Tolerance_)
			{
				vel_.angular.z = 0;
				cmd_pub_.publish(vel_);
				pd_flag_ = Point_Linear_Angular;
				PrintPD_Var();
			}else{
				cmd_pub_.publish(vel_);
				PrintPD_Var();
			}

			break;
		case Point_Linear_Angular:

			cur_angle_ = atan2(y,x);
			cur_distance_ = sqrt(x*x+y*y);

			vel_.linear.x = p_distance_gain_ * cur_distance_ + d_distance_gain_ * before_distance_;
			vel_.angular.z = p_angle_gain_ * cur_angle_ + d_angle_gain_ * before_angle_;
			before_distance_ = cur_distance_;
			before_angle_ = cur_angle_;

			if( abs(cur_angle_) < angle_Tolerance_ && abs(cur_distance_) < distance_Tolerance_)
			{
				vel_.linear.x = 0;
				vel_.angular.z = 0;
				cmd_pub_.publish(vel_);
				pd_flag_ = Target_Angular;
				PrintPD_Var();
			}else{
				cmd_pub_.publish(vel_);
				PrintPD_Var();
			}
			break;
		case Target_Angular:

			cur_angle_ = target_pose_th;
			cur_distance_ = 0;

			vel_.linear.x = p_distance_gain_ * cur_distance_ + d_distance_gain_ * (cur_distance_ - before_distance_);
			vel_.angular.z = p_angle_gain_ * cur_angle_ + d_angle_gain_ * (cur_angle_ - before_angle_);
			before_distance_ = cur_distance_;
			before_angle_ = cur_angle_;

			if(abs(cur_angle_) < angle_Tolerance_)
			{
				vel_.angular.z = 0;
				cmd_pub_.publish(vel_);
				pd_flag_ = Point_Angular;
				PrintPD_Var();
				return true;
			}else{
				cmd_pub_.publish(vel_);
				PrintPD_Var();
			}
			
	}

    last_time_ = ros::Time::now();
    vel_X_ = vel_.linear.x;
    vel_Th_ = vel_.angular.z;

    return false;
}

void TagSlam::PrintPD_Var()
{
	printf("flag : %d \n",pd_flag_);
	printf("linearVel : %.3f\n",vel_.linear.x);
	printf("angularVel : %.3f\n",vel_.angular.z);
	printf("cur_angle_ : %.3f || angle_Tolerance_ : %.3f \n",RAD2DEG(cur_angle_),RAD2DEG(angle_Tolerance_));
	//printf("linear_Vel_ : %.3f || distance_Tolerance : %.3f\n")
	printf("target_pose_x_ : %.3f || target_pose_y_ : %.3f || target_pose_th_ : %.3f \n",
		target_pose_x_,target_pose_y_,target_pose_th_);
	printf("Th_ : %.3f || vel_X_: %.3f || vel_Y_ : %.3f\n",Th_,vel_X_,vel_Y_);
}