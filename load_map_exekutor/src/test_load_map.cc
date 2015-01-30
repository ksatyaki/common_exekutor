#include <exekutor/load_map_exekutor.h>

int main (int argn, char* args[])
{
	peiskmt_initialize (&argn, args);
	ros::init (argn, args, "test_load_map");

	exekutor::LoadMapExekutor load_map_exek ("coro", "loadmap", "/home/ace/load_map.txt");

	exekutor::ActionExekutor::waitForLink();
	return 0;
}
