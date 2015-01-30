/*
 * miradock_exekutor.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: ace
 */

#include "exekutor/miradock_exekutor.h"

namespace exekutor {


MiradockExekutor::MiradockExekutor(std::string robot_name, std::string action_name) :
			ActionExekutor (robot_name, action_name)
{
	docking_client_ = nh_.serviceClient <cognidrive_ros::Dock> ("Dock");
	docking_status_sub_ = nh_.subscribe("docking_status", 5, &MiradockExekutor::dockingStatusCallback, this);

	last_operation_ = UNDOCK;
	last_docked_station_number_ = 1;
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
		ROS_INFO("Docking to station no. %d", std::atoi(stationString.c_str()));
		message.request.ID_station = std::atoi(stationString.c_str());
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
