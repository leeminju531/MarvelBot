#include "ros/ros.h"
#include <std_srvs/Empty.h>

bool flag = false;

void timerCallback(const ros::TimerEvent& event){
	flag = true;
}

int main(int argc, char**argv){

	ros::init(argc, argv, "global_localization_based_lidar");
	ros::NodeHandle n;

	ros::ServiceClient spread_particle_client = n.serviceClient<std_srvs::Empty>("global_localization");
	std_srvs::Empty msg;

	ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);
	
/*
Initiate global localization, 
wherein all particles are dispersed randomly through the free space in the map
*/
	if( spread_particle_client.call(msg) ){	// spread particle 
		ROS_INFO("Spreaded localization called");

		ros::ServiceClient particle_localization_client = n.serviceClient<std_srvs::Empty>("request_nomotion_update");
		std_srvs::Empty msg;

		bool lidar_based_localization_loof = true;
		int cnt = 0;
		while(lidar_based_localization_loof){
			if(flag){
				flag=false;
				cnt++;
				if(particle_localization_client.call(msg)){
					ROS_INFO("particle update localization called");
				}else{
					ROS_ERROR("particle update localization Failed");
				}
			}
			ros::spinOnce();
			if(cnt > 20){
				timer.stop();
				lidar_based_localization_loof = false;
			}
		}


	}else{
		ROS_ERROR("Spreaded localization Failed");
	}
	
	return 0;
}