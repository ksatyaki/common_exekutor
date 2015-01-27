/*
 * acquire_exekutor.h
 *
 *  Created on: Aug 21, 2014
 *      Author: ace
 */

#ifndef ACQUIRE_EXEKUTOR_H_
#define ACQUIRE_EXEKUTOR_H_

#include <ros/ros.h>
#include <exekutor/action_exekutor.h>
#include <cam_interface/cam_interface.h>
#include <acquire_tabletop/AcquireTabletop.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <string>
#include <geometry_msgs/Point.h>

namespace exekutor {

class AcquireExekutor: public ActionExekutor
{
protected:

	/**
	 * A service client to the acquire_objects server.
	 */
	ros::ServiceClient client_;

	/**
	 * This function creates a client to the acquire_objects server.
	 *
	 * The implementation of actionThread() function.
	 * This is called from ActionExekutor's startActionThread().
	 * This is where we create the client to the move_to_simple action.
	 */
	void actionThread();

	/**
	 * Update the CAM tuple with the object location.
	 */
	void updateObjectDetailsInCAM(const std::string& object_name, const doro_msgs::TableObject& object);

public:
	/**
	 * Constructor.
	 */
	AcquireExekutor(std::string robot_name, std::string action_name);

	/**
	 * Destructor.
	 */
	virtual ~AcquireExekutor();
};

} /* namespace exekutor */

#endif /* ACQUIRE_EXEKUTOR_H_ */
