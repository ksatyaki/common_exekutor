/*
 * load_map_exekutor.h
 *
 *  Created on: Jan 30, 2015
 *      Author: ace
 */

#ifndef LOAD_MAP_EXEKUTOR_H_
#define LOAD_MAP_EXEKUTOR_H_

#include <exekutor/action_exekutor.h>
#include <cognidrive_ros/LoadMap.h>
#include <fstream>
#include <algorithm>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace exekutor
{

class LoadMapExekutor : ActionExekutor
{
	/**
	 * A client to change the map.
	 */
	ros::ServiceClient load_map_client_;

	/**
	 * A Hashmap to store the ids and map file locations.
	 */
	std::map <std::string, std::string> ids_and_maps_;

	/**
	 * The overridden actionThread function.
	 */
	void actionThread();

	/**
	 * A publisher to the initial pose topic.
	 */
	ros::Publisher initial_pose_pub_;
public:
	LoadMapExekutor(std::string robot_name, std::string action_name,  std::string map_config_file_name = "/localhome/demo/load_map.txt");

	virtual ~LoadMapExekutor();
};

} /* namespace exekutor */

#endif /* COMMON_EXEKUTOR_LOAD_MAP_EXEKUTOR_INCLUDE_EXEKUTOR_LOAD_MAP_EXEKUTOR_H_ */
