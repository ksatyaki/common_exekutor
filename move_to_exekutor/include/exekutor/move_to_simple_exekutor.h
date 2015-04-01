/**
 * @file   move_to_simple_exekutor.h
 * @author Chittaranjan S Srinivas
 *     
 * Copyright (C) 2015  Chittaranjan Srinivas Swaminathan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 * 
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
