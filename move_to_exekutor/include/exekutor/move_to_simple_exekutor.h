/*
 * move_to_simple_exekutor.h
 *
 *  Created on: Jul 7, 2014
 *      Author: ace
 */

#ifndef MOVE_TO_SIMPLE_EXEKUTOR_H_
#define MOVE_TO_SIMPLE_EXEKUTOR_H_

#include <exekutor/action_exekutor.h>
#include <actionlib/client/simple_action_client.h>
#include <simple_service/MoveToSimpleAction.h>

namespace exekutor {

/**
 * Simple navigation exekutor using odometry alone.
 */
class MoveToSimpleExekutor: public ActionExekutor
{
protected:

	/**
	 * A simple_action_client to the 'move_to_simple' action server.
	 */
	actionlib::SimpleActionClient <simple_service::MoveToSimpleAction> move_to_simple_client_;

	/**
	* This function creates a client to the move_to_simple action.
	*
	* The implementation of actionThread() function.
	* This is called from ActionExekutor's startActionThread().
	* This is where we create the client to the move_to_simple action.
	*/
	void actionThread();

public:
	/**
	 * Constructor.
	 */
	MoveToSimpleExekutor(std::string robot_name, std::string action_name);

	/**
	 * Destructor.
	 */
	virtual ~MoveToSimpleExekutor();
};

} /* namespace exekutor */

#endif /* MOVE_TO_SIMPLE_EXEKUTOR_H_ */
