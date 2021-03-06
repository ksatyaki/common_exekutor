/**
 * @file   move_to_exekutor.cpp
 * @author Chittaranjan S Srinivas
 * 
 * @brief  MoveTo Using Exekutor interface.
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

	bool base_link_target = false;

	nh_.getParam("move_base/yaw_tolerance", old_yaw_tolerance);
	nh_.getParam("move_base/xy_tolerance_max", old_xy_tolerance_max);
	nh_.getParam("move_base/xy_tolerance_min", old_xy_tolerance_min);
	nh_.getParam("move_base/driving_direction", old_driving_direction);

	ROS_INFO("MoveToExekutor has started!");
	setState(RUNNING);

	PeisTuple paramTuple = getParamTuple();

	std::vector <std::string>  params = extractParamStrings(paramTuple.data);
	std::vector<double> the_values;

	if(params.size() == 1 || params.size() == 2)
	{
		if(params.size() == 2) {
			if(params[1].compare("forward") == 0 || params[1].compare("backward") == 0) {
				std::vector <double> object_location = cam_interface::getObjectReachablePositionFromCAM(params[0]);
				the_values.push_back(object_location[0]);
				the_values.push_back(object_location[1]);
				the_values.push_back(object_location[2]); // This is in fact a heading and not a z co-ordinate.
				base_link_target = false;

				nh_.setParam("move_base/driving_direction", params[1]);
				ROS_INFO("DRIVING DIRECTION: %s", params[1].c_str());
			}
			else
				the_values = extractParams(paramTuple.data);
		}
		else {
			std::vector <double> object_location = cam_interface::getObjectReachablePositionFromCAM(params[0]);
			the_values.push_back(object_location[0]);
			the_values.push_back(object_location[1]);
			the_values.push_back(object_location[2]); // This is in fact a heading and not a z co-ordinate.
			base_link_target = false;
		}
	}

	else
		the_values = extractParams(paramTuple.data);

	int cmd_args = the_values.size();

	/* Small notifier in case the parameters are scanty or superfluous. */
	if(cmd_args < 2)
	{
		//ROS_ERROR("The parameters are too few. At least x and y are required. Aborting action...");
		setState(FAILED);
		return;
	}

	if(cmd_args == 2)
	{
		if(nh_.hasParam("move_base/yaw_tolerance"))
		{
			//ROS_INFO("\nA goal yaw was not specified.");
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

			//ROS_INFO("\n The required parameter for position tolerance was found and is being set.");
			/* The fourth (optional) argument is the tolerance on the position.
			 * We are setting it here */
			ROS_INFO("XY TOLERANCE: %lf", the_values[3]);
			nh_.setParam("move_base/xy_tolerance_min", the_values[3]);
			nh_.setParam("move_base/xy_tolerance_max", the_values[3] + 0.5);
		}
	}

	if(cmd_args >= 5)
	{
		if(nh_.hasParam("move_base/yaw_tolerance"))
		{
			ROS_INFO("YAW TOLERANCE: %lf", the_values[4]);
			/* The third (optional) argument is the required orientation
			 * In case it is not provided, we assume that we have no tolerance on the orientation.*/
			nh_.setParam("move_base/yaw_tolerance", the_values[4]);
		}
	}
	if(cmd_args >= 6)
	{
		std::string driving_dir = the_values[5] == -1.0 ? "backward": "forward";
		ROS_INFO("DRIVING DIRECTION: %s", driving_dir.c_str());
		nh_.setParam("move_base/driving_direction", driving_dir);
	}

	if(cmd_args > 6)
		ROS_WARN("Too many parameters. This can lead to unpredictable results.");

	/* CODE FOR EXECUTING THE FUNCTION SHOULD GO HERE */
	/* A variable for the goal */
	move_base_msgs::MoveBaseGoal the_goal;
	the_goal.target_pose.header.frame_id = base_link_target? "base_link" : "map";
	the_goal.target_pose.header.stamp = ros::Time::now();

	the_goal.target_pose.pose.position.x = the_values[0];
	the_goal.target_pose.pose.position.y = the_values[1];

	ROS_INFO("Goal position: %lf, %lf.", the_values[0], the_values[1]);

	if(cmd_args > 2)
	{
		ROS_INFO("Goal heading: %lf.", the_values[2]);
		the_goal.target_pose.pose.orientation.w = cos(the_values[2]/2);
		the_goal.target_pose.pose.orientation.z = sin(the_values[2]/2);
	}
	else
	{
		the_goal.target_pose.pose.orientation.w = 1.0;
		the_goal.target_pose.pose.orientation.z = 0.0;
	}

	mb_client_.waitForServer(ros::Duration(30.0));

	if(!mb_client_.isServerConnected()) {
		ROS_INFO("Couldn't find the move_base action server in 30 seconds.");
		setState(FAILED);
		setResult("Server not found");
		nh_.setParam("move_base/yaw_tolerance", old_yaw_tolerance);
		nh_.setParam("move_base/xy_tolerance_max", old_xy_tolerance_max);
		nh_.setParam("move_base/xy_tolerance_min", old_xy_tolerance_min);
		nh_.setParam("move_base/driving_direction", old_driving_direction);
		
		return;
	}
	
	ROS_INFO("Sending goal.");

	mb_client_.sendGoal(the_goal);

	mb_client_.waitForResult(ros::Duration(30.0));

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
