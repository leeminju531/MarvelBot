#include "april_slam.h"

TagSlam::TagSlam() 
:	tfListener_(tfBuffer_),
	vel_X_(0),vel_Y_(0),vel_Th_(0),Th_(0)
{
	parentFrame_ = "base_footprint";
	PDParamSet();
	
	cmd_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel",10);
}

void TagSlam::PDParamSet()
{
	distance_Tolerance_ = 0.1;
	angle_Tolerance_ = DEG2RAD(5);
	p_distance_gain_ = 0.1;
	d_distance_gain_ = 0.1;
	p_angle_gain_ = 0.1;
	d_angle_gain_ = 0.1;
	pd_flag_ = Turn2TargetPoint;
	imagine_target_pose_x_=0;
	imagine_target_pose_y_=0;
	imagine_th_=0;
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
			ros::Time::now(),ros::Duration(0.2));
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

	last_time_ = ros::Time::now();
	current_time_ = ros::Time::now();
	imagine_th_ = 0;
	while(ros::ok())
	{
		try
		{
			transformStamped_ = tfBuffer_.lookupTransform(parentFrame_,tagFrame[tagNum],
			ros::Time::now(),ros::Duration(0.2));

		}catch(tf2::TransformException &ex)
		{
			ROS_INFO(" Frame can't leading ! ");
			
			// PredictUpdate();
			if(PDControl(imagine_target_pose_x_,imagine_target_pose_y_,0))
			{
				ROS_WARN("Completed Pose Update But, Frame Not Exist. ");
				ROS_WARN("Please Put On April Tag Detection Area. ");
			}
			continue;
		}
		ImagineInit();
		detect_pose_x = transformStamped_.transform.translation.x;
		detect_pose_y = transformStamped_.transform.translation.y;

		tf::Quaternion quat_temp;
		double hole_r,hole_p,hole_y;
		tf::quaternionMsgToTF(transformStamped_.transform.rotation,quat_temp);
		tf::Matrix3x3(quat_temp).getRPY(hole_r,hole_p,hole_y);
		// printf("x : %f || y : %f\n",detect_pose_x,detect_pose_y);
		// printf("roll :%f || pitch :%f || yaw :%f\n",hole_r,hole_p,hole_y);
		detect_pose_th = hole_y;

		float target_pose_x = detect_pose_x + tag_pose_x;
		float target_pose_y = detect_pose_y + tag_pose_y;
		float target_pose_th = detect_pose_th + tag_pose_th;
		if(PDControl(target_pose_x,target_pose_y,target_pose_th)){
			ROS_INFO("Completed To Target Position.");
			break;
		}	
		
		// ros::Duration(0.1).sleep();

	}

	return true;

}

bool TagSlam::TagLocation(int tagNum, float tag_pose_th)
{
	return TagLocation(tagNum, 0.0, 0.0, tag_pose_th);
}

// Predict Target Position Update Based On Velocity and Pose Data.
void TagSlam::ImagineUpdate(float cur_pose_x,float cur_pose_y,float vel_X,float vel_Th)
{
	current_time_ = ros::Time::now();
	double dt = (current_time_ - last_time_).toSec();
	double delta_th = vel_Th * dt;
	// double delta_x = ( vel_X_ * cos(delta_th) ) * dt;
	// double delta_y = ( vel_X_ * sin(delta_th) ) * dt;
	float x = cur_pose_x;
	float y = cur_pose_y;
	printf("delta_th : %.3f\n",RAD2DEG(delta_th));
	printf("dt : %.3f\n",dt);
	printf("vel_Th : %.3f\n",vel_Th);
	imagine_th_ += delta_th;
	// float x = cur_pose_x - delta_x;
	// float y = cur_pose_y - delta_y;
	imagine_target_pose_x_ = cos(delta_th)*x - sin(delta_th)*y;
	imagine_target_pose_y_ = sin(delta_th)*x + cos(delta_th)*y;
	last_time_ = ros::Time::now();
}

void TagSlam::ImagineInit()
{
	imagine_target_pose_x_ = 0;
	imagine_target_pose_y_ = 0;
	imagine_th_ = 0;
	current_time_ = last_time_ = ros::Time::now();
}

// target x,y,th On Tag Frame
bool TagSlam::PDControl(float target_pose_x,float target_pose_y, float target_pose_th)
{
	float x = target_pose_x;
    float y = target_pose_y;
    cur_distance_ = sqrt(x*x + y*y);
	
	// if(cur_distance_ < distance_Tolerance_)
	// 	pd_flag_ = Turn2TargetOrientation;

	// if(cur_distance_ >= distance_Tolerance_)
	// 	if(pd_flag_ == Turn2TargetOrientation || pd_flag_ == CorrenctInTolerance)
	// 		pd_flag_ = CorrenctInTolerance;
	// 	else
	// 		pd_flag_ = Turn2TargetPoint;

	switch(pd_flag_)
	{
		// only angular Vel to Target Point
		case Turn2TargetPoint:
			if(x == 0)
			{
				if(y>0)			cur_angle_ = DEG2RAD(90);
				else if(y<0)	cur_angle_ = DEG2RAD(-90);
				else if(y==0)	cur_angle_ = 0;

			}else	cur_angle_ = atan2(y,x);
				
			
			cur_distance_ = 0;

			vel_.linear.x = p_distance_gain_ * cur_distance_ + d_distance_gain_ * (cur_distance_ - before_distance_);
			vel_.angular.z = p_angle_gain_ * cur_angle_ + d_angle_gain_ * (cur_angle_ - before_angle_);
			before_distance_ = cur_distance_;
			before_angle_ = cur_angle_;

			if(abs(cur_angle_) < angle_Tolerance_)
			{
				vel_.angular.z = 0;
				//pd_flag_ = Moving2TargetPoint;
			}
			cmd_pub_.publish(vel_);

			break;
		case Moving2TargetPoint:

			if(x == 0)
			{
				if(y>0)			cur_angle_ = DEG2RAD(90);
				else if(y<0)	cur_angle_ = DEG2RAD(-90);
				else if(y==0)	cur_angle_ = 0;

			}else	cur_angle_ = atan2(y,x);
			
			cur_distance_ = sqrt(x*x+y*y);

			vel_.linear.x = p_distance_gain_ * cur_distance_ + d_distance_gain_ * (cur_distance_ - before_distance_);
			vel_.angular.z = p_angle_gain_ * cur_angle_ + d_angle_gain_ * (cur_angle_ - before_angle_);
			before_distance_ = cur_distance_;
			before_angle_ = cur_angle_;

			if( cur_distance_ < distance_Tolerance_)
			{
				vel_.linear.x = 0;
				vel_.angular.z = 0;
				pd_flag_ = Turn2TargetOrientation;
			}	
			cmd_pub_.publish(vel_);

			break;

		

		case Turn2TargetOrientation:

			cur_angle_ = target_pose_th;
			cur_distance_ = 0;

			vel_.linear.x = p_distance_gain_ * cur_distance_ + d_distance_gain_ * (cur_distance_ - before_distance_);
			vel_.angular.z = p_angle_gain_ * cur_angle_ + d_angle_gain_ * (cur_angle_ - before_angle_);
			before_distance_ = cur_distance_;
			before_angle_ = cur_angle_;

			if(abs(cur_angle_) < angle_Tolerance_)
			{
				vel_.angular.z = 0;
				pd_flag_ = CorrenctInTolerance;
				
				return true;
			}
			cmd_pub_.publish(vel_);
			
			break;

		case CorrenctInTolerance:

			/* 
			control 
			*/

			if(abs(cur_angle_) < angle_Tolerance_ && cur_distance_ < distance_Tolerance_)
			{
				vel_.linear.x = 0;
				vel_.angular.z = 0;
				pd_flag_ = Turn2TargetOrientation;
			}
			cmd_pub_.publish(vel_);
			break;
			
	}
	target_pose_x_ = x;
	target_pose_y_ = y;
	vel_X_ = vel_.linear.x;
	vel_Th_ = vel_.angular.z;
	ImagineUpdate(target_pose_x, target_pose_y, vel_.linear.x , vel_.angular.z);
	printf("vel_.angular.z : %.3f\n",vel_.angular.z);
	PrintPD_Var();
	
    

    return false;
}

void TagSlam::PrintPD_Var()
{
	// printf("flag : %d \n",pd_flag_);
	// printf("linearVel : %.3f\n",vel_.linear.x);
	// printf("angularVel : %.3f\n",vel_.angular.z);
	// printf("cur_angle_ : %.3f || angle_Tolerance_ : %.3f \n",RAD2DEG(cur_angle_),RAD2DEG(angle_Tolerance_));
	// //printf("linear_Vel_ : %.3f || distance_Tolerance : %.3f\n")
	
	// // printf("Th_ : %.3f || vel_X_: %.3f || vel_Y_ : %.3f\n",Th_,vel_X_,vel_Y_);
	// printf("imagine_target_pose_x_ : %.3f || imagine_target_pose_y_ : %.3f\n",imagine_target_pose_x_,imagine_target_pose_y_);
	// printf("detected target_pose_x_ : %.3f || target_pose_y_ : %.3f\n",
	// 	target_pose_x_,target_pose_y_);
	// printf("Predicted imagine_pose_x_:%.3f || imagine_pose_y_ : %.3f\n ",
	// 	imagine_target_pose_x_,imagine_target_pose_y_);
	// printf("predicted imagine_th_ : %.3f\n",RAD2DEG(imagine_th_));
}