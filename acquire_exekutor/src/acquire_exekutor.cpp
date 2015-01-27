/*
 * acquire_exekutor.cpp
 *
 *  Created on: Aug 21, 2014
 *      Author: ace
 */

#include "exekutor/acquire_exekutor.h"

namespace exekutor {

AcquireExekutor::AcquireExekutor(std::string robot_name, std::string action_name):
		ActionExekutor(robot_name, action_name)
{
	client_ = nh_.serviceClient <acquire_tabletop::AcquireTabletop> ("acquire_tabletop");
}

void AcquireExekutor::updateObjectDetailsInCAM(const std::string& object_name, const doro_msgs::TableObject& object)
{
	//ROS_INFO("Known object. Updating CAM.");
	int CAM_peis_id = 9898;
	std::string pos_geo_key = object_name + ".pos.geo.update";
	std::string color_key = object_name + ".color.rgb.update";
	std::string size_key = object_name + ".boundingbox.update";

	tf::StampedTransform map_to_base_link;
	tf::Vector3 point_in_base_link (object.centroid.x, object.centroid.y, object.centroid.z);

	try
	{
		tf_listener_->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3));
		//tf_listener_->transformPoint("base_link", object_position, _object_position);
		tf_listener_->lookupTransform("map", "base_link", ros::Time(0), map_to_base_link);

	}
	catch(tf::TransformException& ex)
	{
		ROS_INFO("What's the problem: %s", ex.what());
		return;
	}

	tf::Vector3 point_in_map = map_to_base_link*point_in_base_link;

	//printf("Kay till here\n");

	char position_tuple_str[100];
	char color_tuple_str[100];
	char size_tuple_str[100];
	sprintf(position_tuple_str, "%0.4f %0.4f %0.4f", point_in_map.x(), point_in_map.y(), point_in_map.z());
	if(object.color.size() == 3)
		sprintf(color_tuple_str, "%u %u %u", object.color[0], object.color[1], object.color[2]);
	if(object.cluster_size.size() == 3)
		sprintf(size_tuple_str, "%0.4f %0.4f %0.4f", object.cluster_size[0], object.cluster_size[1], object.cluster_size[2]);

	peiskmt_setRemoteStringTuple(CAM_peis_id, color_key.c_str(), color_tuple_str);
	peiskmt_setRemoteStringTuple(CAM_peis_id, pos_geo_key.c_str(), position_tuple_str);
	peiskmt_setRemoteStringTuple(CAM_peis_id, size_key.c_str(), size_tuple_str);
	ROS_INFO("Updated CAM.");
}

AcquireExekutor::~AcquireExekutor()
{
}

void AcquireExekutor::actionThread()
{
	PeisTuple paramTuple = getParamTuple();
	ROS_INFO("Acquire exekutor has started.");

	std::vector <std::string> params = extractParamStrings(paramTuple.data);

	acquire_tabletop::AcquireTabletop message;

	if(params.empty())
	{
		setState(FAILED);
		ROS_INFO("Parameters were empty. Action failed.");
		return;
	}

	if(params[0].compare("all") == 0)
	{
		message.request.type = acquire_tabletop::AcquireTabletopRequest::ALL;
		if(params.size() > 1)
			message.request.tolerance = std::strtod(params[1].c_str(), NULL);
		else
			message.request.tolerance = 0.8;
	}

	else if(params[0].compare("known") == 0)
	{
		message.request.type = acquire_tabletop::AcquireTabletopRequest::KNOWN;

		if(params.size() > 1)
			message.request.tolerance = std::strtod(params[1].c_str(), NULL);
		else
			message.request.tolerance = 0.8;
	}


	else if(params[0].compare("signature") == 0)
	{
		if(params.size() != 2)
		{
			setState(FAILED);
			ROS_INFO("Signature name was not given. Action failed.");
			return;
		}

		if(params.size() > 2)
		{
			message.request.tolerance = std::strtod(params[2].c_str(), NULL);
		}
		else
			message.request.tolerance = 0.8;

		message.request.type = acquire_tabletop::AcquireTabletopRequest::SIGNATURE;

		message.request.signature = cam_interface::getObjectSignatureFromCAM(params[1], this->tf_listener_);

		/**
		 * BEGIN Naive method
		geometry_msgs::PointStamped position = cam_interface::getObjectPositionFromCAM(params[1], this->tf_listener_);
		std::vector <double> position_tolerance = cam_interface::getObjectPositionToleranceFromCAM(params[1]);
		std::vector <unsigned char> color = cam_interface::getObjectColorFromCAM(params[1]);
		std::vector <double> size = cam_interface::getObjectSizeFromCAM(params[1]);

		* END Naive method
		**/
	}
	else
	{
		message.request.type = acquire_tabletop::AcquireTabletopRequest::NAME;
		message.request.signature.id = params[0];
		if(params.size() > 1)
			message.request.tolerance = std::strtod(params[1].c_str(), NULL);
		else
			message.request.tolerance = 0.8;
	}

	ROS_INFO("Calling client...");
	if (client_.call(message))
	{
		std::cout<<message.response;
		std::string result_string = "";
		// SET SOME TUPLES WITH THE RECEIVED MESSAGE.

		for(std::vector<doro_msgs::TableObject>::iterator obj_iterator = message.response.objects.table_objects.begin();
				obj_iterator != message.response.objects.table_objects.end();
				obj_iterator++)
		{
			result_string += std::string(obj_iterator->id + " ");
			//std::cout<<"result_string at this stage: "<<result_string.c_str()<<std::endl;
			if(obj_iterator->id.find("unknown") == std::string::npos && obj_iterator->id.find("unseen") == std::string::npos)
			{
				updateObjectDetailsInCAM(obj_iterator->id, *obj_iterator);
			}
		}

		if(message.response.objects.table_objects.size() == 0)
		{
			setResult("not found");
			setState(FAILED);
			ROS_ERROR("Object not found");
			return;
		}

		else
			setResult(result_string);
	}

	else
	{
		setState(FAILED);
		ROS_ERROR("Failed to call service acquire_tabletop");
		return;
	}
	setState(COMPLETED);
}

} /* namespace exekutor */
