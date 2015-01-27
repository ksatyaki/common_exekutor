/*
 * move_to_simple_exekutor.cpp
 *
 *  Created on: Jul 7, 2014
 *      Author: ace
 */

#include <exekutor/move_to_simple_exekutor.h>

namespace exekutor {

MoveToSimpleExekutor::MoveToSimpleExekutor(std::string robot_name, std::string action_name):
	ActionExekutor(robot_name, action_name),
	move_to_simple_client_("move_to_simple", true)
{

}

MoveToSimpleExekutor::~MoveToSimpleExekutor()
{
}

void MoveToSimpleExekutor::actionThread()
{
	PeisTuple paramTuple = getParamTuple();
	ROS_INFO("MoveToSimpleExekutor has started.");

	simple_service::MoveToSimpleGoal our_goal;

	std::vector <double> the_values = extractParams(paramTuple.data);

	int cmd_args = the_values.size();

	// cmd_args has the number of arguments and the arguments are in the_values.

	// Two arguments are x and y.
	if(cmd_args == 2)
	{
		// Default tolerance on the xy position.
		// Server sets default tolerance to 0.075 when this value is left at 0.0.

		// Don't care about theta (yaw).
		our_goal.yaw_tolerance = 3.14;
	}
	// Three arguments are x, y and theta.
	else if(cmd_args == 3)
	{
		// Server sets default tolerances to 0.075 for xy and 0.085 for yaw.
	}
	else if(cmd_args == 4)
	{
		our_goal.xy_tolerance = the_values[3];
		// Server sets default tolerance to 0.085 for yaw when left at 0.0.
	}
	else if(cmd_args == 5)
	{
		our_goal.xy_tolerance = the_values[3];
		our_goal.yaw_tolerance = the_values[4];
	}
	else if(cmd_args < 2)
	{
		ROS_ERROR("The parameters are too few. At least x and y are required. Aborting action...");
		setState(FAILED);
	}
	else
	{
		ROS_WARN("Too many parameters. This can lead to unpredictable results.");
	}

	our_goal.goal_pose.header.frame_id = "base_link"; // The whole point is relative motion.
	our_goal.goal_pose.header.stamp = ros::Time::now();

	our_goal.goal_pose.pose.position.x = the_values[0];
	our_goal.goal_pose.pose.position.y = the_values[1];

	if(cmd_args > 2)
	{
		our_goal.goal_pose.pose.orientation.w = cos(the_values[2]/2);
		our_goal.goal_pose.pose.orientation.z = sin(the_values[2]/2);
	}
	else
	{
		our_goal.goal_pose.pose.orientation.w = 1.0;
		our_goal.goal_pose.pose.orientation.z = 0.0;
	}

	move_to_simple_client_.waitForServer();
	ROS_INFO("Sending goal.");

	move_to_simple_client_.sendGoal(our_goal);

	move_to_simple_client_.waitForResult();

	if(move_to_simple_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Successfully executed the action on the robot.");
		setState(COMPLETED);
		sleep(1);
	}
	else
	{
		ROS_INFO("\n\nAttempted! But, something cocked-up and we failed!");
		setState(FAILED);
		sleep(1);
	}

}


} /* namespace exekutor */
