/*
 * look_exekutor.cpp
 *
 *  Created on: Sep 2, 2014
 *      Author: ace
 */

#include "exekutor/look_exekutor.h"

namespace exekutor {

LookExekutor::LookExekutor (std::string robot_name, std::string action_name) :
		ActionExekutor (robot_name, action_name),
		ptu_client_ ("ptu/SetPTUState", true)
{
}

LookExekutor::~LookExekutor()
{
}

void LookExekutor::actionThread()
{
	PeisTuple paramTuple = getParamTuple();
	ROS_INFO("LookExekutor has started.");

	geometry_msgs::PointStamped obj_position = cam_interface::getObjectPositionFromCAM(paramTuple.data, this->tf_listener_);

	if(obj_position.header.frame_id.compare("para-universe") == 0)
	{
		ROS_INFO("There was a problem retriving the position of object from CAM. Action FAILED.");
		setState(FAILED);
		return;
	}

	else if(obj_position.header.frame_id.compare("cocked-up") == 0)
	{
		ROS_INFO("Transform Failure. Action FAILED.");
		setState(FAILED);
		return;
	}

	ptu_control::PtuGotoGoal ptu_goal;

	float x__ =  obj_position.point.x + 0.17;

	double reqd_tilt = asin((1.40 - obj_position.point.z)/ sqrt((x__*x__) + (obj_position.point.y*obj_position.point.y)));
	double reqd_pan = atan2(obj_position.point.y, x__);

	ROS_INFO("Required pan = %lf", reqd_pan);
	ROS_INFO("Required tilt = %lf", reqd_tilt);

	ptu_goal.pan = reqd_pan * 180 / PI;
	ptu_goal.pan_vel = 0.5 * 180 / PI;
	ptu_goal.tilt = -1 * reqd_tilt * 180 / PI;
	ptu_goal.tilt_vel = 0.5 * 180 / PI;;

	ptu_client_.waitForServer();
	ROS_INFO("Found the server. Sending PTU goal.");

	ptu_client_.sendGoal(ptu_goal);
	ptu_client_.waitForResult(ros::Duration(10.0));

	if(ptu_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Action succeeded. Bye.");
		setState(COMPLETED);
		return;
	}
	else
	{
		ROS_INFO("Look failed. Bye.");
		setState(FAILED);
		return;
	}
}

} /* namespace exekutor */