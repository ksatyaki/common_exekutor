/*
 * move_to_executor.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: ace
 */

#include "exekutor/move_to_exekutor.h"

namespace exekutor
{

MoveToExekutor::MoveToExekutor(std::string robot_name, std::string move_to_name) :
		ActionExekutor::ActionExekutor(robot_name, move_to_name),
		mb_client_("move_base", true)
{

}

MoveToExekutor::~MoveToExekutor()
{

}

void MoveToExekutor::actionThread()
{
	ROS_INFO("MoveToExekutor has started!");
	setState(RUNNING);

	PeisTuple paramTuple = getParamTuple();

	std::vector<double> the_values = extractParams(paramTuple.data);

	int cmd_args = the_values.size();

	/* Small notifier in case the parameters are scanty or superfluous. */
	if(cmd_args < 2)
	{
		ROS_ERROR("The parameters are too few. At least x and y are required. Aborting action...");
		setState(FAILED);
		return;
	}

	if(cmd_args > 5)
		ROS_WARN("Too many parameters. This can lead to unpredictable results.");

	/* *******************************************************************************************************
	 * /move_base/TrajectoryPlannerROS/xy_goal_tolerance ---> This controls the tolerance in the position
	 * ( i.e. x and y co-ordinates )
	 *
	 * /move_base/TrajectoryPlannerROS/yaw_goal_tolerance ---> This controls the tolerance in the orientation.
	 * ******************************************************************************************************* */
	if(cmd_args == 4)
	{
		if (nh_.hasParam("/move_base/TrajectoryPlannerROS/xy_goal_tolerance"))
		{
			ROS_INFO("\n The required parameter for position tolerance was found and is being set.");
			/* The fourth (optional) argument is the tolerance on the position.
			 * We are setting it here */
			nh_.setParam("/move_base/TrajectoryPlannerROS/xy_goal_tolerance", the_values[3]);
		}
	}
	if(cmd_args == 2)
	{
		if(nh_.hasParam("/move_base/TrajectoryPlannerROS/yaw_goal_tolerance"))
		{
			ROS_INFO("\n The required parameter for orientation tolerance was found and is being set. Note: This means a goal yaw was not specified.");
			/* The third (optional) argument is the required orientation
			 * In case it is not provided, we assume that we have no tolerance on the orientation.*/
			nh_.setParam("/move_base/TrajectoryPlannerROS/yaw_goal_tolerance", 3.14);
		}
	}
	if(cmd_args == 5)
	{
		if (nh_.hasParam("/move_base/TrajectoryPlannerROS/xy_goal_tolerance"))
		{
			ROS_INFO("\n The required parameter for position tolerance was found and is being set.");
			/* The fourth (optional) argument is the tolerance on the position.
			 * We are setting it here */
			nh_.setParam("/move_base/TrajectoryPlannerROS/xy_goal_tolerance", the_values[3]);
		}

		if(nh_.hasParam("/move_base/TrajectoryPlannerROS/yaw_goal_tolerance"))
		{
			ROS_INFO("\n The required parameter for orientation tolerance was found and is being set. Note: This means a goal yaw was not specified.");
			/* The third (optional) argument is the required orientation
			 * In case it is not provided, we assume that we have no tolerance on the orientation.*/
			nh_.setParam("/move_base/TrajectoryPlannerROS/yaw_goal_tolerance", the_values[4]);
		}
	}

	/* CODE FOR EXECUTING THE FUNCTION SHOULD GO HERE */
	/* A variable for the goal */
	move_base_msgs::MoveBaseGoal the_goal;
	the_goal.target_pose.header.frame_id = "map";
	the_goal.target_pose.header.stamp = ros::Time::now();

	the_goal.target_pose.pose.position.x = the_values[0];
	the_goal.target_pose.pose.position.y = the_values[1];

	if(cmd_args > 2)
	{
		the_goal.target_pose.pose.orientation.w = cos(the_values[2]/2);
		the_goal.target_pose.pose.orientation.z = sin(the_values[2]/2);
	}
	else
	{
		the_goal.target_pose.pose.orientation.w = 1.0;
		the_goal.target_pose.pose.orientation.z = 0.0;
	}

	ROS_INFO("Orientation (%f,%f)", the_goal.target_pose.pose.orientation.w,
			the_goal.target_pose.pose.orientation.z);

	mb_client_.waitForServer();
	ROS_INFO("Sending goal.");

	mb_client_.sendGoal(the_goal);

	mb_client_.waitForResult();

	if(mb_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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
}
