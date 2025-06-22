#include <stdio.h>
#include <platforms/vxworks/apps.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/workqueue.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/init.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include "hrt_work.h"
#include <vector>

#define MAX_ARGS 8 // max number of whitespace separated args after app name
#define MODULE_NAME "px4-vxworks"

static void run_cmd(apps_map_type &apps, const std::vector<std::string> &appargs)
{
	// command is appargs[0]
	std::string command = appargs[0];

	//replaces app.find with iterator code to avoid null pointer exception
	for (apps_map_type::iterator it = apps.begin(); it != apps.end(); ++it)
		if (it->first == command) {
			// one for command name, one for null terminator
			const char *arg[MAX_ARGS + 2];

			unsigned int i = 0;

			if (appargs.size() > MAX_ARGS + 1) {
				PX4_ERR("%d too many arguments in run_cmd", appargs.size() - (MAX_ARGS + 1));
				return;
			}

			while (i < appargs.size() && appargs[i].c_str()[0] != '\0') {
				arg[i] = (char *)appargs[i].c_str();
				PX4_DEBUG("  arg%d = '%s'\n", i, arg[i]);
				++i;
			}

			arg[i] = (char *)0;

			//PX4_DEBUG_PRINTF(i);
			if (apps[command] == NULL) {
				PX4_ERR("Null function !!\n");

			} else {
				apps[command](i, (char **)arg);
				break;
			}

		}
}

void eat_whitespace(const char *&b, int &i)
{
	// Eat whitespace
	while (b[i] == ' ' || b[i] == '\t') { ++i; }

	b = &b[i];
	i = 0;
}

static void process_commands(apps_map_type &apps, const char *cmds)
{
	std::vector<std::string> appargs;
	int i = 0;
	const char *b = cmds;
	char arg[256];

	// Eat leading whitespace
	eat_whitespace(b, i);

	for (;;) {
		// End of command line
		if (b[i] == '\n' || b[i] == '\0') {
			strncpy(arg, b, i);
			arg[i] = '\0';
			appargs.push_back(arg);

			// If we have a command to run
			if (appargs.size() > 0) {
				PX4_DEBUG("Processing command: %s", appargs[0].c_str());

				for (int ai = 1; ai < (int)appargs.size(); ai++) {
					PX4_DEBUG("   > arg: %s", appargs[ai].c_str());
				}

				run_cmd(apps, appargs);
			}

			appargs.clear();

			if (b[i] == '\n') {
				eat_whitespace(b, ++i);
				continue;

			} else {
				break;
			}
		}

		// End of arg
		else if (b[i] == ' ') {
			strncpy(arg, b, i);
			arg[i] = '\0';
			appargs.push_back(arg);
			eat_whitespace(b, ++i);
			continue;
		}

		++i;
	}
}


int zynq7k_main(int argc, char * argv[]) {
#ifdef _WRS_KERNEL
	printf("\nHello world from kernel space!\n");
#else
	printf("\nHello world from user space!\n");
#endif

	printf("\n");
	printf("______  __   __    ___ \n");
	printf("| ___ \\ \\ \\ / /   /   |\n");
	printf("| |_/ /  \\ V /   / /| |\n");
	printf("|  __/   /   \\  / /_| |\n");
	printf("| |     / /^\\ \\ \\___  |\n");
	printf("\\_|     \\/   \\/     |_/\n");
	printf("\n");
	printf("%s starting.\n", "px4-vxworks");
	printf("\n");

	hrt_abstime time = hrt_absolute_time();
	apps_map_type apps;
	printf("hrt abs time=%llu\n", time);
	init_app_map(apps);
	printf("apps.size()=%d\n", apps.size());

	hrt_init();
	px4::WorkQueueManagerStart();

	work_queues_init();
	hrt_work_queue_init();
	// px4_platform_init()

	param_init();
	process_commands(apps, "uorb start\n");
	sleep(2);
	process_commands(apps, "fake_imu start\n");
	// process_commands(apps, "px4_simple_app start\n");
	process_commands(apps, "uorb status\n");
	process_commands(apps, "dataman start\n");
	process_commands(apps, "dataman help\n");
	process_commands(apps, "dataman start -r\n");
	process_commands(apps, "mavlink help\n");
	process_commands(apps, "listen help\n");
	process_commands(apps, "mavlink start -x -u 14556 -r 4000000 -f\n");
	process_commands(apps, "mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u 14556\n");
	process_commands(apps, "sensors start\n");
	process_commands(apps, "commander start\n");
	process_commands(apps, "navigator start\n");
	process_commands(apps, "ekf2 start\n");

	for (;;) {
		sleep(1);
		// process_commands(apps, "uorb status\n");
	}

	return 0;
}

static void usage()
{
	PX4_INFO("Usage: px4 {start |stop}");
}

int main(int argc, char *argv[])
{
	int ret = 0;

	if (argc == 2 && strcmp(argv[1], "start") == 0) {
		(void) px4_task_spawn_cmd("zynq7k",
			    SCHED_DEFAULT,
			    SCHED_PRIORITY_MAX - 5,
			    1500,
			    zynq7k_main,
			    argv);

	} else {
		usage();
		ret = -1;
	}


	for (;;) {
		sleep(1);
	}


	return ret;
}