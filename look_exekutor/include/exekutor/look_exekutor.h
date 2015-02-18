/*
 * look_exekutor.h
 *
 *  Created on: Sep 2, 2014
 *      Author: ace
 */

#ifndef LOOK_EXEKUTOR_H_
#define LOOK_EXEKUTOR_H_

#include <exekutor/action_exekutor.h>
#include <cam_interface/cam_interface.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PointStamped.h>
#include <ptu_control/PtuGotoAction.h>
#include <math.h>
#include <sensor_msgs/JointState.h>

#define PI 3.14159265359

namespace exekutor {

class LookExekutor: public ActionExekutor
{

	ros::Publisher ptu_cmd_pub_;
	/**
	 * Action client to talk to the ptu action server.
	 */
	actionlib::SimpleActionClient <ptu_control::PtuGotoAction> ptu_client_;

	/**
	 * The implementation of the actionThread() function.
	 * This creates a client to the Ptu action server.
	 */
	void actionThread();


public:

	/**
	 * Constructor takes name of robot and action as parameters.
	 */
	LookExekutor(std::string robot_name, std::string action_name);

	virtual ~LookExekutor();
};

} /* namespace exekutor */

#endif /* LOOK_EXEKUTOR_H_ */
