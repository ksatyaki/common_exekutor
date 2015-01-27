/*
 * transform_point.cc
 *
 *  Created on: Sep 3, 2014
 *      Author: Kolam-spaceman
 */

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PointStamped.h>

int main(int argn, char* args[])
{
	ros::init(argn, args, "point_transformer");

	tf::TransformListener tf_l (ros::Duration(10.0));
/**
0.981195
        y: 0.121204
        z: 1.04594*/
	tf::Vector3 my_point_base_link (0.9588, 0.0026, 0.939), my_point_map;
	tf::StampedTransform map_to_bl;

	try
	{
		tf_l.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
		//tf_l.transformPoint("map", _s, our_s);
		tf_l.lookupTransform("map", "base_link", ros::Time(0), map_to_bl);
	}
	catch(tf::TransformException& ex)
	{
		ROS_INFO("%s is the exception.", ex.what());
	}

	my_point_map = map_to_bl*my_point_base_link;

	ROS_INFO("[X: %lf], [Y: %lf], [Z: %lf]", my_point_map.x(), my_point_map.y(), my_point_map.z());

	return 0;
}






