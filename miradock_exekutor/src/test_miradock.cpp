#include <exekutor/miradock_exekutor.h>
#include <signal.h>

void handler(int signal_number)
{
	if(signal_number == SIGINT)
		printf("Ctrl-C caught...\n");
	else
		printf("Ctrl-Z caught...\n");

	ros::shutdown();
	peiskmt_shutdown();

	abort();
}

int main(int argn, char* args[])
{
	ros::init(argn, args, "miradock_exekutor");
	peiskmt_initialize (&argn, args);

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = &handler;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTSTP, &sa, NULL);

	exekutor::MiradockExekutor miradock_exek ("coro", "miradock","/home/ace/docking_stations.txt");
	miradock_exek.waitForLink();

	return 0;
}
