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
	double old_xy_tolerance_max;
	double old_xy_tolerance_min;
	double old_yaw_tolerance;
	std::string old_driving_direction;

	nh_.getParam("move_base/yaw_tolerance", old_yaw_tolerance);
	nh_.getParam("move_base/xy_tolerance_max", old_xy_tolerance_max);
	nh_.getParam("move_base/xy_tolerance_min", old_xy_tolerance_min);
	nh_.getParam("move_base/driving_direction", old_driving_direction);

	ROS_INFO("MoveToExekutor has started!");
	setState(RUNNING);

	PeisTuple paramTuple = getParamTuple();

	std::vector <std::string>  params = extractParamStrings(paramTuple.data);
	std::vector<double> the_values;

	if(params.size() == 1)
	{
		geometry_msgs::PointStamped object_location = cam_interface::getObjectPositionFromCAM(params[0], tf_listener_);
		the_values.push_back(object_location.point.x);
		the_values.push_back(object_location.point.y);
		the_values.push_back(1.0);
		the_values.push_back(0.8);
		the_values.push_back(3.14);
	}

	else
		the_values = extractParams(paramTuple.data);

	int cmd_args = the_values.size();

	/* Small notifier in case the parameters are scanty or superfluous. */
	if(cmd_args < 2)
	{
		ROS_ERROR("The parameters are too few. At least x and y are required. Aborting action...");
		setState(FAILED);
		return;
	}

	if(cmd_args == 2)
	{
		if(nh_.hasParam("move_base/yaw_tolerance"))
		{
			ROS_INFO("\nA goal yaw was not specified.");
			/* The third (optional) argument is the required orientation
			 * In case it is not provided, we assume that we have no tolerance on the orientation.*/
			nh_.setParam("move_base/yaw_tolerance", 3.14);
		}
	}

	/* *******************************************************************************************************
	 * /move_base/TrajectoryPlannerROS/xy_goal_tolerance ---> This controls the tolerance in the position
	 * ( i.e. x and y co-ordinates )
	 *
	 * /move_base/TrajectoryPlannerROS/yaw_goal_tolerance ---> This controls the tolerance in the orientation.
	 * ******************************************************************************************************* */
	if(cmd_args >= 4)
	{
		if (nh_.hasParam("move_base/xy_tolerance_max") && nh_.hasParam("move_base/xy_tolerance_min"))
		{

			ROS_INFO("\n The required parameter for position tolerance was found and is being set.");
			/* The fourth (optional) argument is the tolerance on the position.
			 * We are setting it here */
			nh_.setParam("move_base/xy_tolerance_min", the_values[3]);
			nh_.setParam("move_base/xy_tolerance_max", the_values[3] + 0.5);
		}
	}

	if(cmd_args >= 5)
	{
		if(nh_.hasParam("move_base/yaw_tolerance"))
		{
			ROS_INFO("(((YAW TOLERANCE))): %lf", the_values[4]);
			/* The third (optional) argument is the required orientation
			 * In case it is not provided, we assume that we have no tolerance on the orientation.*/
			nh_.setParam("move_base/yaw_tolerance", the_values[4]);
		}
	}
	if(cmd_args >= 6)
	{
		std::string driving_dir = the_values[5] == -1.0 ? "backward": "forward";
		ROS_INFO("(((DRIVING DIRECTION))): %s", driving_dir.c_str());
		nh_.getParam("move_base/driving_direction", driving_dir);
	}

	if(cmd_args > 6)
		ROS_WARN("Too many parameters. This can lead to unpredictable results.");

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

	// SET OLD PARAMETERS BACK.

	nh_.setParam("move_base/yaw_tolerance", old_yaw_tolerance);
	nh_.setParam("move_base/xy_tolerance_max", old_xy_tolerance_max);
	nh_.setParam("move_base/xy_tolerance_min", old_xy_tolerance_min);
	nh_.setParam("move_base/driving_direction", old_driving_direction);


}
}
