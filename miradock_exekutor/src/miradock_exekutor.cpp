/*
 * miradock_exekutor.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: ace
 */

#include "exekutor/miradock_exekutor.h"

namespace exekutor {


MiradockExekutor::MiradockExekutor(std::string robot_name, std::string action_name, std::string docking_stations_filename) :
			ActionExekutor (robot_name, action_name)
{
	docking_client_ = nh_.serviceClient <cognidrive_ros::Dock> ("Dock");
	docking_status_sub_ = nh_.subscribe("docking_status", 5, &MiradockExekutor::dockingStatusCallback, this);

	last_operation_ = UNDOCK;
	last_docked_station_number_ = 1;

	std::fstream f (docking_stations_filename.c_str());
	if(!f)
	{
		ROS_INFO("Couldn't open the file: %s", docking_stations_filename.c_str());
		exit(-1);
	}
	char buffer[500];
	while(f.getline(buffer, 500))
	{
		std::string buf = buffer;

		if(strlen(buffer) == 0 || buf.find_first_of(" --- ") == std::string::npos)
			break;

		station_ids_and_numbers_.insert (
				std::pair<std::string, int> (
				buf.substr(0, buf.find_first_of(" --- ")),
				atoi( buf.substr(buf.find_first_of(" --- ") + std::string(" --- ").length()).c_str() )
			)
		);
	}

	ROS_INFO("***********************************");
	for(std::map<std::string, int>::iterator iter = station_ids_and_numbers_.begin(); iter != station_ids_and_numbers_.end(); iter++)
	{
		ROS_INFO("ID: %s, STATION NUMBER: %d",  iter->first.c_str(), iter->second);
	}
	ROS_INFO("***********************************");

}

void MiradockExekutor::dockingStatusCallback(const std_msgs::String::ConstPtr& rollerStatus)
{
	docking_status_ = rollerStatus;
}

MiradockExekutor::~MiradockExekutor()
{
}

void MiradockExekutor::actionThread()
{
	ROS_INFO("DockingExekutor has started.");
	std::string stationString = getParamTuple().data;
	for(int i = 0; i<stationString.size(); i++)
		stationString[i] = toupper(stationString[i]);

	cognidrive_ros::Dock message;

	if(stationString.compare("UNDOCK") == 0)
	{
		message.request.type = cognidrive_ros::Dock::Request::UNDOCK;
		message.request.ID_station = last_docked_station_number_;
	}
	else
	{
		message.request.type = cognidrive_ros::Dock::Request::DOCK;
		ROS_INFO("Docking to station no. %d", atoi(stationString.c_str()));
		message.request.ID_station = station_ids_and_numbers_[stationString.c_str()];
	}

	if(docking_client_.call(message))
	{
		ROS_INFO("Dispatching success. Wait for result...");
		docking_status_.reset();
	}
	else
	{
		setState(FAILED);
		ROS_INFO("Failed when calling the docking server at cognidrive_ros...");
		return;
	}

	while(true)
	{
		ros::spinOnce();
		if(!docking_status_)
			continue;
		else
		{
			if(message.request.type == cognidrive_ros::Dock::Request::UNDOCK && docking_status_->data.compare("UnDocked") == 0)
			{
				last_operation_ = UNDOCK;
				break;
			}
			else if(message.request.type == cognidrive_ros::Dock::Request::DOCK && docking_status_->data.compare("Docked") == 0)
			{
				last_docked_station_number_ = message.request.ID_station;
				last_operation_ = DOCK;
				break;
			}
			else if(docking_status_->data.compare("Failure") == 0)
			{
				setState(FAILED);
				ROS_INFO("Failed when trying to dock.");
				return;
			}
		}

	}

	setState(COMPLETED);
	ROS_INFO("Successfully executed the action.");
}

} /* namespace exekutor */
