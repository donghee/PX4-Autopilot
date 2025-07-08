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

namespace px4
{
void init_once();
}

int zynq7k_main(int argc, char * argv[]) {
#ifdef _WRS_KERNEL
	printf("\nHello world from kernel space!\n");
#else
	printf("\nHello world from user space!\n");
	unsigned int clock_rate = __sysClkRateGet();
	printf("sysClkRateGet=%d\n", clock_rate); // default 60Hz, should be ???hz for PX4
#endif

	px4::init_once();
	px4::init(argc, argv, MODULE_NAME);

	hrt_abstime time = hrt_absolute_time();
	printf("hrt abs time=%llu\n", time);
	apps_map_type apps;
	init_app_map(apps);
	// printf("apps.size()=%d\n", apps.size());

	process_commands(apps, "param select param\n");
	// process_commands(apps, "param set SYS_AUTOCONFIG 1\n");
	process_commands(apps, "param set SYS_AUTOCONFIG 0\n");
	process_commands(apps, "param set MAV_SYS_ID 0\n");
	//process_commands(apps, "param set SYS_AUTOSTART 10017\n"); // 1001: HIL QUADROTOR X, 4001: GENERAL QUADROTOR X 10017: JMAVSIM IRIS, 1100: SIH Quadcopter
	process_commands(apps, "param set SYS_AUTOSTART 1100\n"); // 1001: HIL QUADROTOR X, 4001: GENERAL QUADROTOR X 10017: JMAVSIM IRIS, 1100: SIH Quadcopter

	process_commands(apps, "param set CAL_ACC0_ID 1310988\n"); // DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	process_commands(apps, "param set CAL_GYRO0_ID 1310988\n"); // DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
	process_commands(apps, "param set CAL_ACC1_ID 1310996\n"); // DRV_IMU_DEVTYPE_SIM, BUS: 2, ADDR: 1, TYPE: SIMULATION
	process_commands(apps, "param set CAL_GYRO1_ID 1310996\n"); // DRV_IMU_DEVTYPE_SIM, BUS: 2, ADDR: 1, TYPE: SIMULATION
	process_commands(apps, "param set CAL_ACC2_ID 1311004\n"); // DRV_IMU_DEVTYPE_SIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
	process_commands(apps, "param set CAL_GYRO2_ID 1311004\n"); // DRV_IMU_DEVTYPE_SIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
	
	process_commands(apps, "param set CAL_MAG0_ID 197388\n");
	process_commands(apps, "param set CAL_MAG0_PRIO 50\n");
	process_commands(apps, "param set CAL_MAG1_ID 197644\n");
	process_commands(apps, "param set CAL_MAG1_PRIO 50\n");

	process_commands(apps, "param set SENS_BOARD_X_OFF 0.000001\n");
	process_commands(apps, "param set SENS_DPRES_OFF 0.001\n");

	process_commands(apps, "param set-default BAT1_N_CELLS 4\n");
	process_commands(apps, "param set-default CBRK_SUPPLY_CHK 894281\n");
	process_commands(apps, "param set-default COM_CPU_MAX -1\n");
	process_commands(apps, "param set-default COM_RC_IN_MODE 1\n");
	process_commands(apps, "param set-default EKF2_REQ_GPS_H 0.5\n");
	process_commands(apps, "param set-default IMU_GYRO_FFT_EN 1\n");
	process_commands(apps, "param set-default MAV_PROTO_VER 2\n"); // Ensures QGC does not drop the first few packets after a SITL restart due to MAVLINK 1 packets
	process_commands(apps, "param set-default -s MC_AT_EN 1\n");

	process_commands(apps, "param set-default SDLOG_MODE 1\n");
	process_commands(apps, "param set-default SDLOG_PROFILE 131\n");
	process_commands(apps, "param set-default SDLOG_DIRS_MAX 7\n");

	process_commands(apps, "param set-default TRIG_INTERFACE 3\n");
	process_commands(apps, "param set-default SYS_FAILURE_EN 1\n");
	process_commands(apps, "param set-default COM_LOW_BAT_ACT 2\n");

	// simulator
	process_commands(apps, "param set SYS_HITL 2\n"); //1: HITL, 2: SIH, 0: Disabled Default
	process_commands(apps, "param set SENS_EN_GPSSIM 1\n");
	process_commands(apps, "param set SENS_EN_BAROSIM 1\n");
	process_commands(apps, "param set SENS_EN_MAGSIM 1\n");

	process_commands(apps, "param set PWM_MAIN_FUNC1 101\n");
	process_commands(apps, "param set PWM_MAIN_FUNC2 102\n");
	process_commands(apps, "param set PWM_MAIN_FUNC3 103\n");
	process_commands(apps, "param set PWM_MAIN_FUNC4 104\n");
	process_commands(apps, "param set SIH_VEHICLE_TYPE 0\n");

	process_commands(apps, "param set IMU_INTEG_RATE 200\n"); // For jmavsim simulator
	process_commands(apps, "param set-default IMU_INTEG_RATE 200\n"); // For jmavsim simulator

	process_commands(apps, "param set COM_ARM_SDCARD 0\n");
	// process_commands(apps, "param set SYS_HAS_BARO 0\n");
	// process_commands(apps, "param set SYS_HAS_MAG 0\n");

	process_commands(apps, "param set EKF2_GPS_DELAY 10\n");
	process_commands(apps, "param set EKF2_MULTI_IMU 3\n");
	process_commands(apps, "param set SENS_IMU_MODE 0\n");

	// process_commands(apps, "logger start -t -b 200\n"); // 200 bytes buffer, timestamp enabled

	// process_commands(apps, "dataman start -f dataman\n"); // File
	process_commands(apps, "dataman start -r\n"); // RAM
	process_commands(apps, "load_mon start\n");

	// sensors
	// process_commands(apps, "fake_imu start\n");
	// process_commands(apps, "fake_gps start\n");
	// process_commands(apps, "fake_magnetometer start\n");
	
	// simulator
	process_commands(apps, "simulator_sih start\n");
	process_commands(apps, "sensor_baro_sim start\n");
	process_commands(apps, "sensor_gps_sim start\n");
	process_commands(apps, "sensor_mag_sim start\n");
	//process_commands(apps, "sensor_airspeed_sim start\n");
	//process_commands(apps, "pwm_out_sim start\n");
	process_commands(apps, "pwm_out_sim start -m hil\n");
	process_commands(apps, "simulator_mavlink start\n");

	// ground control
	// process_commands(apps, "mavlink start -x -u 14556 -r 4000000 -f\n");
	// process_commands(apps, "mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u 14556\n");

	// simulator mavlink
	process_commands(apps, "mavlink start -x -u 14560 -r 4000000 -f\n");

	// controller
	process_commands(apps, "manual_control start");
	process_commands(apps, "control_allocator start");
	process_commands(apps, "mc_rate_control start");
	process_commands(apps, "mc_att_control start");
	process_commands(apps, "mc_hover_thrust_estimator start");
	process_commands(apps, "flight_mode_manager start");
	process_commands(apps, "mc_pos_control start");
	process_commands(apps, "land_detector start multicopter");

	process_commands(apps, "sensors start\n");
	//process_commands(apps, "commander start\n");
	process_commands(apps, "commander start -h\n"); // hitl
	process_commands(apps, "navigator start\n");
	process_commands(apps, "ekf2 start\n");

	for (;;) {
		sleep(1);
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