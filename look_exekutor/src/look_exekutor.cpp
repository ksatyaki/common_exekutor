/*
 * look_exekutor.cpp
 *
 *  Created on: Sep 2, 2014
 *      Author: ace
 */

#include "exekutor/look_exekutor.h"

namespace exekutor {

LookExekutor::LookExekutor (std::string robot_name, std::string action_name) :
		ActionExekutor (robot_name, action_name)
{
	ptu_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>("/ptu/cmd", 1);
}

LookExekutor::~LookExekutor()
{
}

void LookExekutor::actionThread()
{
	PeisTuple paramTuple = getParamTuple();
	ROS_INFO("LookExekutor has started.");

	bool named_look = false;
	geometry_msgs::PointStamped obj_position;
	if(std::string("ahead").compare(paramTuple.data) == 0)
	{
	  named_look = true;
	}
	else 
	  obj_position = cam_interface::getObjectPositionFromCAM(paramTuple.data, this->tf_listener_);

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



	float x__ =  obj_position.point.x + 0.17;

	if(named_look) {
	  reqd_tilt = 0.0;
	  reqd_pan = 0.0;
	}
	else {
	  reqd_tilt = asin((1.405 - obj_position.point.z)/ sqrt((x__*x__) + (obj_position.point.y*obj_position.point.y)));
	  reqd_pan = atan2(obj_position.point.y, x__);
	}

	if(reqd_tilt >= 0.79)
	  reqd_tilt = 0.79;

	sensor_msgs::JointState ptu_cmd_msg_;
	ptu_cmd_msg_.position.push_back(reqd_pan);
	reqd_tilt = -1*reqd_tilt;
	ptu_cmd_msg_.position.push_back(reqd_tilt);

	ptu_cmd_msg_.velocity.push_back(0.5);
	ptu_cmd_msg_.velocity.push_back(0.5);

	ROS_INFO("Required pan = %lf", reqd_pan);
	ROS_INFO("Required tilt = %lf", reqd_tilt);

	ptu_cmd_pub_.publish(ptu_cmd_msg_);

	ptu_js_sub = nh_.subscribe ("ptu/joint_states", 1, &LookExekutor::ptujsCB, this);

	ptu_action_done = false;
	bool timeout = false;
	int countdown = 50;
	ros::Rate spinRate(5);
	while(!ptu_action_done)
	{
	  if(countdown == 0)
	  {
	    timeout = true;
	    break;
	  }
	  spinRate.sleep();
	  ros::spinOnce();
	  countdown--;
	}

	if(timeout == true)
	{
	  ROS_INFO("Look FAILED.");
	  setState(FAILED);
	}
	else
	{
	  ROS_INFO("Action succeeded. Bye.");
	  setState(COMPLETED);
	}

	ptu_js_sub.shutdown();
}

void LookExekutor::ptujsCB(const sensor_msgs::JointState::ConstPtr& _msg)
{
  ROS_INFO("PAN: %lf, TILT: %lf", _msg->position[0] - reqd_pan, _msg->position[1] - reqd_tilt);
  if(fabs(_msg->position[0] - reqd_pan) < 0.01 && fabs(_msg->position[1] - reqd_tilt) < 0.01)
     ptu_action_done = true;
}


} /* namespace exekutor */
