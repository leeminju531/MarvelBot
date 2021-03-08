#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <math.h>
#include <boost/thread/thread.hpp>
#include "marvel_core/DetectionTag.h"
#include <queue>
#include <boost/thread/mutex.hpp>

#define PI 3.141592
#define RAD2DEG(x) ((x)*180./PI)
#define DEG2RAD(X) ((X)*PI/180.)

using namespace std;

#define PI 3.141592
#define RAD2DEG(x) ((x)*180./PI)
#define DEG2RAD(X) ((X)*PI/180.)

//ros::NodeHandle node_;

std::queue<string> tagQue; // store detecting avable tag repeatly
boost::mutex que_mutex; // for worker thread checking Queue mutex

// publish detection tag using thread pool
marvel_core::DetectionTag tag_msgs;
ros::Publisher tag_pub;
boost::mutex msg_mutex;

boost::mutex rc_mutex;
int r_count = 0;
boost::mutex rw_mutex;


tf2_ros::Buffer tfBuffer; // threads use tfBuffer Only Reader. not Necessary mutex 

enum _PD_Control_Order
{
	Turn2TargetPoint,
	Moving2TargetPoint,
	Turn2TagOrientation
};

class TagSlam
{
public:
	TagSlam(ros::NodeHandle& nh);
	~TagSlam();

private:
	void ParamGet();
	void FrameParamGet();
	void PDParamGet();
	void TagParamGet();
	void ParamPrint();
	
	ros::NodeHandle node_;
	geometry_msgs::TransformStamped transformStamped_;
	tf2_ros::TransformListener tfListener_;
	ros::Publisher cmd_pub_;
	geometry_msgs::Twist vel_;
	// Parameter Concerned with Frame
	string camFrame_,baseFrame_;
	double base_cam_x_,base_cam_y_,base_cam_z_,base_cam_yaw_;
	float tf_cycle_;
	// Parameter Concerned with PD Control
	double distance_Tolerance_,angle_Tolerance_;
	double p_distance_gain_,d_distance_gain_;
	double p_angle_gain_,d_angle_gain_;
	// Parameter Concerned with AprilTag
	std::vector<string> tag_;
	string* Available_Tag_;
	int tag_num_;
	// PD Control Order Flag
	int pd_flag_;

	// base - camera tf relation Broadcaster Thread.
	boost::thread tTFB_;
	static void CamTFB(string baseFrame,string childFrame,double cam_x_from_base,
		double cam_y_from_base,double cam_z_from_base, double cam_yaw_from_base, float cycle);

	// while diff control, diff robot temporary couldn't detect tag
	// at this time, imaginary update target Frame based on cmd vel
	void ImagineInit();
	double imagine_target_pose_x_,imagine_target_pose_y_,imagine_th_;
	ros::Time last_time_,current_time_;

	// tag detector thread ( managing thread pool)
	boost::thread tag_thr_;
	int worker_num_;
	static void TagPublisher(string* Available_Tag_,string baseFrame,int tag_num_,const int worker_num);
	static void WorkerThread(string baseFrame);
};

TagSlam::TagSlam(ros::NodeHandle& nh)
: tfListener_(tfBuffer), node_(nh)
{
	ParamGet();
	ParamPrint();
	
	cmd_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel",10);
	tag_pub = node_.advertise<marvel_core::DetectionTag>("tag_detector",10);

	// publish TF relation base_footprintf and camera  
	tTFB_ = boost::thread(&TagSlam::CamTFB,baseFrame_,camFrame_,
			base_cam_x_,base_cam_y_, base_cam_z_, base_cam_yaw_,tf_cycle_);
	tag_thr_ = boost::thread(&TagSlam::TagPublisher, Available_Tag_,baseFrame_, tag_num_,worker_num_);

}

TagSlam::~TagSlam()
{
	tTFB_.join();
	tag_thr_.join();
	delete[] Available_Tag_;
}
void TagSlam::TagPublisher(string* Available_Tag_,string baseFrame,int tag_num,const int worker_num)
{
	// make worker thread
	boost::thread worker[worker_num];
	for(int i=0; i<worker_num ; i++)
		worker[i] = boost::thread(&TagSlam::WorkerThread,baseFrame);

	que_mutex.lock();
	for(int i=0;i<tag_num;i++)
		tagQue.push(Available_Tag_[i]);
	que_mutex.unlock();

	ros::Rate rate(10.0);
	while(ros::ok())
	{
		rw_mutex.lock();
		que_mutex.lock();
		if(tagQue.empty())
		{
			cout << "tagQueue : ";
			for(int i=0;i<tag_num;i++)
			{
				//temp code for checking 
				cout <<Available_Tag_[i];
				cout << " pushed " ;
				// end of temp
				tagQue.push(Available_Tag_[i]);
			}
			cout << "parent thread queue fulling" << endl;
			// que_mutex.unlock();

			// publish detection tag msg
			msg_mutex.lock();
			tag_pub.publish(tag_msgs);
			tag_msgs.tag.clear();
			msg_mutex.unlock();
		}
		que_mutex.unlock();
		rw_mutex.unlock();

		rate.sleep();
	}
	for(int i=0; i<worker_num ; i++)
		worker[i].join();

	printf(" end of parent thread \n");
}

void TagSlam::WorkerThread(string baseFrame)
{
	ros::Rate rate(10);
	while(ros::ok())
	{
		// geometry_msgs::TransformStamped transformStamped;

		rc_mutex.lock();
		r_count++;
		if(r_count == 1)
			rw_mutex.lock();
		rc_mutex.unlock();

		que_mutex.lock();
		if(!tagQue.empty())
		{
			string tagFrame = tagQue.front();
			tagQue.pop();
			que_mutex.unlock();
			cout << "checking frame : "<< tagFrame << endl;
			try
			{
				tfBuffer.lookupTransform(baseFrame,tagFrame,
				ros::Time::now(),ros::Duration(0.2));
			}
			catch(tf2::TransformException &ex)
			{
				goto skip;
			}
			msg_mutex.lock();
			tag_msgs.tag.push_back(tagFrame);
			msg_mutex.unlock();	
		}
		else
		{
			que_mutex.unlock();
		}
		skip:
		rc_mutex.lock();
		r_count--;
		if(r_count == 0)
			rw_mutex.unlock();
		rc_mutex.unlock();
		
		
		rate.sleep();
	}
	printf(" end of worker Thread\n");
	return ;
}

void TagSlam::ParamGet()
{
	FrameParamGet();
	PDParamGet();
	TagParamGet();
}

void TagSlam::FrameParamGet()
{
	PDParamGet();
	if(!node_.getParam("BaseFrame",baseFrame_))
		baseFrame_ = "base_footprint";
	if(!node_.getParam("CamFrame",camFrame_))
		camFrame_ = "camera";
	if(!node_.getParam("Camera_X_From_BaseFrame",base_cam_x_))
		base_cam_x_ = 0;
	if(!node_.getParam("Camera_Y_From_BaseFrame",base_cam_y_))
		base_cam_y_ = 0;
	if(!node_.getParam("Camera_Z_From_BaseFrame",base_cam_z_))
		base_cam_z_ = 0.3;
	if(!node_.getParam("Camera_Yaw_From_BaseFrame",base_cam_yaw_)){
		base_cam_yaw_ = DEG2RAD(90);
	}else{
		base_cam_yaw_ = DEG2RAD(base_cam_yaw_); // input th unit : degree
	}
	if(!node_.getParam("TFControlCycle",tf_cycle_)) // hz
		tf_cycle_ = 10.0;
}

void TagSlam::PDParamGet()
{
	if(!node_.getParam("distance_Tolerance",distance_Tolerance_))
		distance_Tolerance_ = 0.02;
	if(!node_.getParam("angle_Tolerance",angle_Tolerance_)){
		angle_Tolerance_ = DEG2RAD(2);
	}else{// input angle_Tolerance_ unit : degree
		angle_Tolerance_ = DEG2RAD(angle_Tolerance_);
	}
	if(!node_.getParam("p_distance_gain",p_distance_gain_))
		p_distance_gain_ = 0.3;
	if(!node_.getParam("d_distance_gain",d_distance_gain_))
		d_distance_gain_ = 0.1;
	if(!node_.getParam("p_angle_gain",p_angle_gain_))
		p_angle_gain_ = 0.6;
	if(!node_.getParam("d_angle_gain",d_angle_gain_))
		d_angle_gain_ = 0.1;

	pd_flag_ = Turn2TargetPoint;
}

void TagSlam::TagParamGet() // Detection Available Tag
{
	if(!node_.getParam("Tag",tag_)){
		tag_.assign(1,"tag_0");
	}
	tag_num_ = (int)tag_.size();

	Available_Tag_ = new string[tag_num_];
	for(int i=0;i<tag_num_;i++)
		Available_Tag_[i] = tag_[i].c_str();

	if(!node_.getParam("WorkerThread",worker_num_))
		worker_num_ = 4;
}
void TagSlam::ParamPrint()
{
	printf("===============PD Parameter Check=============\n");
	printf("distance_Tolerance : %.3f (m)\n",distance_Tolerance_);
	printf("angle_Tolerance : %.3f (Degree)\n",RAD2DEG(angle_Tolerance_));
	printf("p_distance_gain : %.3f\n",p_distance_gain_);
	printf("d_distance_gain : %.3f\n",d_distance_gain_);
	printf("p_angle_gain : %.3f\n",p_angle_gain_);
	printf("d_angle_gain : %.3f\n",d_angle_gain_);
	printf("==============================================\n\n");
	printf("===============Frame Parameter Check=============\n");
	printf("BaseFrame : %s \n",baseFrame_.c_str());
	printf("Camera Frame : %s \n",camFrame_.c_str());
	printf("Camera_X_From_BaseFrame : %.2f (m)\n",base_cam_x_);
	printf("Camera_Y_From_BaseFrame : %.2f (m)\n",base_cam_y_);
	printf("Camera_Z_From_BaseFrame : %.2f (m)\n",base_cam_z_);
	printf("Camera_Yaw_From_BaseFrame : %.2f (Degree)\n",RAD2DEG(base_cam_yaw_));
	printf("=====================================================\n");
	printf("==============Tag Parameter Check=====================\n");
	printf("Detection Available tag_number : %d\n",tag_num_);
	printf("Detection Available Tag : ");
	for(int i=0;i<tag_num_;i++)
		printf("%s ",tag_[i].c_str());
	printf("\n");
}
void TagSlam::CamTFB(string baseFrame,string childFrame,double cam_x_from_base,
		double cam_y_from_base,double cam_z_from_base, double cam_yaw_from_base,
		float cycle)
{
	
	tf2_ros::TransformBroadcaster tfb;
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.frame_id = baseFrame;
	transformStamped.child_frame_id = childFrame;
	transformStamped.transform.translation.x = cam_x_from_base;
	transformStamped.transform.translation.y = cam_y_from_base;
	transformStamped.transform.translation.z = cam_z_from_base;
	tf2::Quaternion q;
	q.setRPY(0,0,cam_yaw_from_base);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();
	ros::Rate rate(cycle);
	while(ros::ok())
	{
		transformStamped.header.stamp = ros::Time::now();
		tfb.sendTransform(transformStamped);
		rate.sleep();
	}
	
}
void TagSlam::ImagineInit()
{
	imagine_target_pose_x_ = 0;
	imagine_target_pose_y_ = 0;
	imagine_th_ = 0;
	current_time_ = last_time_ = ros::Time::now();
}


// bool TagSlam::TagLocation(int tagNum, float tag_pose_x,float tag_pose_y, float tag_pose_th)
// {
	
// 	if (!TagDetection(tagNum))	return false;

// 	float detect_pose_x=0,detect_pose_y=0,detect_pose_th=0;

// 	last_time_ = ros::Time::now();
// 	current_time_ = ros::Time::now();
// 	imagine_th_ = 0;
// 	while(ros::ok())
// 	{
// 		try
// 		{
// 			transformStamped_ = tfBuffer.lookupTransform(baseFrame_,tagFrame[tagNum],
// 			ros::Time::now(),ros::Duration(0.2));

// 		}catch(tf2::TransformException &ex)
// 		{
// 			ROS_INFO(" Frame can't leading ! ");
			
// 			// PredictUpdate();
// 			if(PDControl(imagine_target_pose_x_,imagine_target_pose_y_,0))
// 			{
// 				ROS_WARN("Completed Pose Update But, Frame Not Exist. ");
// 				ROS_WARN("Please Put On April Tag Detection Area. ");
// 			}
// 			continue;
// 		}
// 		ImagineInit();
// 		detect_pose_x = transformStamped_.transform.translation.x;
// 		detect_pose_y = transformStamped_.transform.translation.y;
// 		// tfBuffer._frameExists(tagFrame[tagNum]);
// 		tf::Quaternion quat_temp;
// 		double hole_r,hole_p,hole_y;
// 		tf::quaternionMsgToTF(transformStamped_.transform.rotation,quat_temp);
// 		tf::Matrix3x3(quat_temp).getRPY(hole_r,hole_p,hole_y);
// 		// printf("x : %f || y : %f\n",detect_pose_x,detect_pose_y);
// 		// printf("roll :%f || pitch :%f || yaw :%f\n",hole_r,hole_p,hole_y);
// 		detect_pose_th = hole_y;

// 		float target_pose_x = detect_pose_x + tag_pose_x;
// 		float target_pose_y = detect_pose_y + tag_pose_y;
// 		float target_pose_th = detect_pose_th + tag_pose_th;
// 		if(PDControl(target_pose_x,target_pose_y,target_pose_th)){
// 			ROS_INFO("Completed To Target Position.");
// 			break;
// 		}	
		
// 		// ros::Duration(0.1).sleep();

// 	}

// 	return true;

// }

// bool TagSlam::TagLocation(int tagNum, float tag_pose_th)
// {
// 	return TagLocation(tagNum, 0.0, 0.0, tag_pose_th);
// }

// // Predict Target Position Update Based On Velocity and Pose Data.
// void TagSlam::ImagineUpdate(float cur_pose_x,float cur_pose_y,float vel_X,float vel_Th)
// {
// 	current_time_ = ros::Time::now();
// 	double dt = (current_time_ - last_time_).toSec();
// 	double delta_th = vel_Th * dt;
// 	double delta_x = ( vel_X_ * cos(delta_th) ) * dt;
// 	double delta_y = ( vel_X_ * sin(delta_th) ) * dt;
// 	float x = cur_pose_x;
// 	float y = cur_pose_y;
// 	printf("delta_th : %.3f\n",RAD2DEG(delta_th));
// 	// printf("dt : %.3f\n",dt);
// 	printf("vel_Th : %.3f\n",vel_Th);
// 	imagine_th_ += delta_th;
// 	x = cur_pose_x - delta_x;
// 	y = cur_pose_y - delta_y;
// 	imagine_target_pose_x_ = cos(delta_th)*x + sin(delta_th)*y;
// 	imagine_target_pose_y_ = -sin(delta_th)*x + cos(delta_th)*y;
// 	printf("atan2 based imagine : %.3f\n",RAD2DEG(atan2(imagine_target_pose_y_,imagine_target_pose_x_)));
// 	last_time_ = ros::Time::now();
// }


// // target x,y,th On Tag Frame
// bool TagSlam::PDControl(float target_pose_x,float target_pose_y, float target_pose_th)
// {
// 	float x = target_pose_x;
//     float y = target_pose_y;
//     cur_distance_ = sqrt(x*x + y*y);
	
// 	// if(cur_distance_ < distance_Tolerance_)
// 	// 	pd_flag_ = Turn2TargetOrientation;

// 	// if(cur_distance_ >= distance_Tolerance_)
// 	// 	if(pd_flag_ == Turn2TargetOrientation || pd_flag_ == CorrenctInTolerance)
// 	// 		pd_flag_ = CorrenctInTolerance;
// 	// 	else
// 	// 		pd_flag_ = Turn2TargetPoint;

// 	switch(pd_flag_)
// 	{
// 		// only angular Vel to Target Point
// 		case Turn2TargetPoint:
// 			if(x == 0)
// 			{
// 				if(y>0)			cur_angle_ = DEG2RAD(90);
// 				else if(y<0)	cur_angle_ = DEG2RAD(-90);
// 				else if(y==0)	cur_angle_ = 0;

// 			}else	cur_angle_ = atan2(y,x);
				
			
// 			cur_distance_ = 0;

// 			vel_.linear.x = p_distance_gain_ * cur_distance_ + d_distance_gain_ * (cur_distance_ - before_distance_);
// 			vel_.angular.z = p_angle_gain_ * cur_angle_ + d_angle_gain_ * (cur_angle_ - before_angle_);
			

// 			before_distance_ = cur_distance_;
// 			before_angle_ = cur_angle_;
// 			if(abs(cur_angle_) < angle_Tolerance_)
// 			{
// 				vel_.angular.z = 0;
// 				pd_flag_ = Moving2TargetPoint;
// 			}
// 			cmd_pub_.publish(vel_);

// 			break;
// 		case Moving2TargetPoint:

// 			if(x == 0)
// 			{
// 				if(y>0)			cur_angle_ = DEG2RAD(90);
// 				else if(y<0)	cur_angle_ = DEG2RAD(-90);
// 				else if(y==0)	cur_angle_ = 0;

// 			}else	cur_angle_ = atan2(y,x);
			
// 			cur_distance_ = sqrt(x*x+y*y);

// 			vel_.linear.x = p_distance_gain_ * cur_distance_ + d_distance_gain_ * (cur_distance_ - before_distance_);
// 			vel_.angular.z = p_angle_gain_ * cur_angle_ + d_angle_gain_ * (cur_angle_ - before_angle_);
// 			before_distance_ = cur_distance_;
// 			before_angle_ = cur_angle_;


// 			if(cur_angle_ > angle_Tolerance_ )
// 			{
// 				vel_.linear.x = 0;
// 				vel_.angular.z = 0;
// 				pd_flag_ = Turn2TargetPoint;
// 			}
// 			if( cur_distance_ < distance_Tolerance_)
// 			{
// 				vel_.linear.x = 0;
// 				vel_.angular.z = 0;
// 				pd_flag_ = Turn2TagOrientation;
// 			}

// 			cmd_pub_.publish(vel_);

// 			break;

		

// 		case Turn2TagOrientation:

// 			cur_angle_ = target_pose_th;
// 			cur_distance_ = 0;

// 			vel_.linear.x = p_distance_gain_ * cur_distance_ + d_distance_gain_ * (cur_distance_ - before_distance_);
// 			vel_.angular.z = p_angle_gain_ * cur_angle_ + d_angle_gain_ * (cur_angle_ - before_angle_);
// 			before_distance_ = cur_distance_;
// 			before_angle_ = cur_angle_;

// 			if(abs(cur_angle_) < angle_Tolerance_)
// 			{
// 				vel_.angular.z = 0;
// 				pd_flag_ = Turn2TargetPoint;
				
// 				return true;
// 			}
// 			cmd_pub_.publish(vel_);
			
// 			break;

		
			
// 	}
// 	target_pose_x_ = x;
// 	target_pose_y_ = y;
// 	vel_X_ = vel_.linear.x;
// 	vel_Th_ = vel_.angular.z;
// 	ImagineUpdate(target_pose_x, target_pose_y, vel_.linear.x , vel_.angular.z);
	
	
    

//     return false;
// }