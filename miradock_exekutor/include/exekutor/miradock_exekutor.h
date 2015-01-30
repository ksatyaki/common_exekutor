/*
 * miradock_exekutor.h
 *
 *  Created on: Jan 29, 2015
 *      Author: ace
 */

#ifndef MIRADOCK_EXEKUTOR_H_
#define MIRADOCK_EXEKUTOR_H_

#include <exekutor/action_exekutor.h>
#include <cognidrive_ros/Dock.h>
#include <cstdlib>
#include <std_msgs/String.h>


namespace exekutor
{

class MiradockExekutor: public ActionExekutor
{
	enum Operation {DOCK, UNDOCK};
	/**
	 * A cakkback for the docking status.
	 */
	void dockingStatusCallback(const std_msgs::String::ConstPtr& dockingStatus);

	/**
	 * A Subscriber to the message from docking status.
	 */
	ros::Subscriber docking_status_sub_;

	/**
	 * A client to the docking service.
	 */
	ros::ServiceClient docking_client_;


	/**
	 * The action thread function - overloaded.
	 */
	void actionThread();

	/**
	 * The globally maintained docking status.
	 */
	std_msgs::String::ConstPtr docking_status_;

	/**
	 * A variable that is set to the last docked station number.
	 */
	int last_docked_station_number_;

	/**
	 * A variable that keeps track of what the last operation was.
	 */
	Operation last_operation_;

public:
	MiradockExekutor(std::string robot_name, std::string action_name);
	virtual ~MiradockExekutor();
};

} /* namespace exekutor */

#endif /* MIRADOCK_EXEKUTOR_H_ */
