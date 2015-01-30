/*
 * load_map_exekutor.cpp
 *
 *  Created on: Jan 30, 2015
 *      Author: ace
 */

#include "exekutor/load_map_exekutor.h"

namespace exekutor {

LoadMapExekutor::LoadMapExekutor(std::string robot_name, std::string action_name, std::string map_config_file_name):
		ActionExekutor(robot_name, action_name)
{
	load_map_client_ = nh_.serviceClient <cognidrive_ros::LoadMap> ("LoadMap");

	initial_pose_pub_ = nh_.advertise <geometry_msgs::PoseWithCovarianceStamped> ("initialpose", 2);

	std::fstream f (map_config_file_name.c_str());
	if(!f)
	{
		ROS_INFO("Couldn't open the file: %s", map_config_file_name.c_str());
		exit(-1);
	}
	char buffer[500];
	while(f.getline(buffer, 500))
	{
		if(strlen(buffer) == 0)
			break;
		std::string buf = buffer;

		ids_and_maps_.insert(std::pair<std::string, std::string> (
				buf.substr(0, buf.find_first_of(" --- ")),
				buf.substr(buf.find_first_of(" --- ") + std::string(" --- ").length()))
		);
	}

	for(std::map<std::string, std::string>::iterator iter = ids_and_maps_.begin(); iter != ids_and_maps_.end(); iter++)
	{
		ROS_INFO("ID: %s, MAP FILE LOCATION: %s",  iter->first.c_str(), iter->second.c_str());
	}

}

LoadMapExekutor::~LoadMapExekutor()
{
}

void LoadMapExekutor::actionThread()
{
	ROS_INFO("LoadMapExekutor Started.");

	std::vector <std::string> split_strs = extractParamStrings(ids_and_maps_[getParamTuple().data].c_str());

	geometry_msgs::PoseWithCovarianceStamped initial_pose;
	initial_pose.header.frame_id = "map";

	double x = std::atof(split_strs[1].c_str());
	double y = std::atof(split_strs[2].c_str());
	double theta = std::atof(split_strs[3].c_str());

	initial_pose.pose.pose.position.x = x;
	initial_pose.pose.pose.position.y = y;
	initial_pose.pose.pose.orientation.w = cos(theta/2);
	initial_pose.pose.pose.orientation.z = sin(theta/2);

	cognidrive_ros::LoadMap message;
	message.request.map = split_strs[0];

	ROS_INFO("Changing to map %s.", message.request.map.c_str());

	if(!load_map_client_.call(message))
	{
		ROS_INFO("Couldn't call the load map service. Action Failed.");
		setState(FAILED);
		return;
	}

	if(message.response.result.find("OK") != std::string::npos)
	{
		initial_pose_pub_.publish(initial_pose);
		ROS_INFO("Map change succeeded.");
		setState(COMPLETED);
	}
	else
	{
		ROS_INFO("Couldn't load the map. Action Failed.");
		setState(FAILED);
	}



}

} /* namespace exekutor */
