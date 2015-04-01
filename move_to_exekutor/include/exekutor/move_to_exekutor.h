/**
 * @file   move_to_exekutor.h
 * @author Chittaranjan S Srinivas
 * 
 * @brief  See the source file.
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
 * 
 */


#ifndef MOVE_TO_EXEKUTOR_H_
#define MOVE_TO_EXEKUTOR_H_

#include "exekutor/action_exekutor.h"
#include "cam_interface/cam_interface.h"
#include <geometry_msgs/PointStamped.h>

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
