/*
 * look_exekutor_test.cc
 *
 *  Created on: Sep 2, 2014
 *      Author: ace
 */

#include <exekutor/look_exekutor.h>


int main(int argn, char* args[])
{
	peiskmt_initialize(&argn, args);
	ros::init(argn, args, "test_look_exekutor");

	exekutor::LookExekutor l_e_ ("Doro", "Look");
	peiskmt_setStringTuple("CAM.objects.tropicana.position", "5.518331 -0.781105 1.0538");
	exekutor::ActionExekutor::waitForLink();

	return 0;
}


