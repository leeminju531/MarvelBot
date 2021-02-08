#include "marvel_core.h"

Marvel::Marvel() 
{
	ParameterSet();
	flag_ = Initial;
}


void Marvel::Start()
{
	while(node_.ok())
	{
		switch{
		case Initial:
			if(DetectTag(tag0))
			{
				if(TagLocation(tag0,x,y,th))
				{
					WaitGoal();
				}
			}
			break;
		case Moving:


			break;
		case AtTargetPosition:


			break;
		case Back:


			break;
		case Aborted:


			break;
		}

	}
	
}

void Marvel::WaitGoal()
{

}

void Marvel::ParameterSet()
{
	if(!node_.getParam("parentFrame",parentFrame_))
		parentFrame_ = "base_footprint";
	if(!node_.getParam("angleTolerance",angleTolerance_))
		angleTolerance_ = DEG2RAD(5);
	if(!node_.getParam("distanceTolerance",distanceTolerance_))
		distanceTolerance_ = 0.1;
	
}

bool Marvel::DetectTag(int tagNum)
{	
	tf2_ros::TransformListener tfListener(tfBuffer_);
	try
	{
		transformStamped_ = tfBuffer_.lookupTransform(parentFrame_,tagFrame[tagNum],ros::Time(0),ros::Duration(0.3));
		
	}
	catch(tf2::TransformException &ex)
	{
		ROS_INFO(" Frame not exist ! ");
		return false;
	}
	ROS_INFO(" Frame is exist ! ");
	return true;	
}

bool Marvel::TagLocation(int tagNum, float x, float y, float th)
{

	/*while loof
	pd control 
	if in tolerance , return true. 
	can't find detection Area -> return false
	*/

	return true;
}

