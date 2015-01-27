/*
 * move_to_exekutor.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Chittaranjan Srinivas
 */

#ifndef MOVE_TO_EXEKUTOR_H_
#define MOVE_TO_EXEKUTOR_H_

#include "exekutor/action_exekutor.h"

/* The actionThread function uses these headers - ROS*/
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace exekutor
{
/**
 * The Class that provides the exekutor interface for the "move to" action.
 */
class MoveToExekutor: public ActionExekutor
{
protected:
	/**
	 * The action client (simple) to a 'move_base' action.
	 * This stays alive for as long as the exekutor instance is alive.
	 */
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_client_;

	/**
	 * This function creates a client to the move_base action.
	 *
	 * The implementation of actionThread() function.
	 * This is called from ActionExekutor's startActionThread().
	 * This is where we create the client to the move_base action.
	 */
	virtual void actionThread();

public:

	/**
	 * A constructor that takes in the name of the robot and the name of the action.
	 */
	MoveToExekutor(std::string robot_name, std::string move_to_name);

	/**
	 * Destructor
	 */
	virtual ~MoveToExekutor();
};
}

#endif /* MOVE_TO_EXEKUTOR_H_ */
