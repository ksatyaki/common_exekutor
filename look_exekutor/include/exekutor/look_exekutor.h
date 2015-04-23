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
#include <math.h>
#include <sensor_msgs/JointState.h>

#define PI 3.14159265359

namespace exekutor {

class LookExekutor: public ActionExekutor
{

	ros::Publisher ptu_cmd_pub_;
	
	ros::Subscriber ptu_js_sub;

	void ptujsCB(const sensor_msgs::JointState::ConstPtr& _msg);

	/**
	 * The implementation of the actionThread() function.
	 * This creates a client to the Ptu action server.
	 */
	void actionThread();

	bool ptu_action_done;

	double reqd_pan, reqd_tilt;


public:

	/**
	 * Constructor takes name of robot and action as parameters.
	 */
	LookExekutor(std::string robot_name, std::string action_name);

	virtual ~LookExekutor();
};

} /* namespace exekutor */

#endif /* LOOK_EXEKUTOR_H_ */
