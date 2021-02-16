#ifndef APRIL_H
#define APRIL_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <math.h>

#define PI 3.141592
#define RAD2DEG(x) ((x)*180./PI)
#define DEG2RAD(X) ((X)*PI/180.)

using namespace std;

enum _tag
{
	tag0,
	tag1,
	tag2,
	tag3,
	tag4,
	tag5,
	
};
enum _PD
{
	Turn2TargetPoint,
	Moving2TargetPoint,
	CorrenctInTolerance,
	Turn2TargetOrientation
};

// enum _PD
// {
// 	Point_Angular,
// 	Point_Linear_Angular,
// 	Target_Angular
// };

int tagSize =6;
string tagFrame[] = {"tag_0","tag_1","tag_2","tag_3","tag_4","tag_5"};

class TagSlam
{
public:
	TagSlam();
	bool TagDetection(int tagNum);
	bool TagLocation(int tagNum, float tag_pose_x,float tag_pose_y, float tag_pose_th);
	bool TagLocation(int tagNum, float tag_pose_th);
	bool PDControl(float target_pose_x,float target_pose_y, float target_pose_th);

private:
	void PredicInit();
	void PDParamSet();
	void PrintPD_Var();
	void ImagineUpdate(float cur_pose_x,float cur_pose_y,float vel_X,float vel_Th);
	void ImagineInit();
	
	ros::NodeHandle node_;
	string parentFrame_;

	tf2_ros::Buffer tfBuffer_; // why have to define here??
	geometry_msgs::TransformStamped transformStamped_;
	tf2_ros::TransformListener tfListener_;

	ros::Publisher cmd_pub_;
	geometry_msgs::Twist vel_;
	

	//for predict updating
	ros::Time last_time_,current_time_;
	
	double before_distance_,before_angle_;
	double cur_distance_,cur_angle_;
	double p_distance_gain_,d_distance_gain_;
	double p_angle_gain_,d_angle_gain_;
	double distance_Tolerance_,angle_Tolerance_;
	int pd_flag_;
	double Th_;
	float imagine_target_pose_x_,imagine_target_pose_y_,imagine_th_;
	float target_pose_x_,target_pose_y_;
	double vel_X_,vel_Y_,vel_Th_;
};


#endif