#include <exekutor/acquire_exekutor.h>
#include <ros/ros.h>
#include <signal.h>

void handler(int signal_number)
{
	if(signal_number == SIGINT)
		printf("Ctrl-C caught...\n");
	else
		printf("Ctrl-Z caught...\n");
	ros::shutdown();

	abort();
}

int main(int argn, char* args[])
{
	ros::init(argn, args, "acquire_exekutor_test_node");
	peiskmt_initialize(&argn, args);

	exekutor::AcquireExekutor Axe("doro", "acquire");

	peiskmt_setStringTuple("doro.acquire.23.command", "OFF");
	peiskmt_setStringTuple("doro.acquire.23.parameters", "-");
	peiskmt_setStringTuple("doro.acquire.23.state", "IDLE");

	peiskmt_setStringTuple("doro.acquire.23.signature.name", "-");
	peiskmt_setStringTuple("doro.acquire.23.signature.size", "-");
	peiskmt_setStringTuple("doro.acquire.23.signature.color", "-");
	peiskmt_setStringTuple("doro.acquire.23.signature.centroid", "-");

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = &handler;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTSTP, &sa, NULL);

	Axe.waitForLink();
	return 0;
}
