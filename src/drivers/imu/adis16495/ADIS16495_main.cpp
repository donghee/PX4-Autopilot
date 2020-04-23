/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ADIS16495.hpp"
#include <systemlib/err.h>

#define ADIS16495_DEVICE_PATH_ACCEL		"/dev/ADIS16495_accel"
#define ADIS16495_DEVICE_PATH_GYRO		"/dev/ADIS16495_gyro"

extern "C" { __EXPORT int adis16495_main(int argc, char *argv[]); }

/**
 * Local functions in support of the shell command.
 */
namespace adis16495
{

ADIS16495	*g_dev;

void	start(enum Rotation rotation);
void	test();
void	reset();
void	info();
void	usage();
/**
 * Start the driver.
 */
void
start(enum Rotation rotation)
{
	if (g_dev != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}

	/* create the driver */
#if defined(PX4_SPIDEV_EXTERNAL1_1)
	g_dev = new ADIS16495(PX4_SPI_BUS_EXTERNAL1, ADIS16495_DEVICE_PATH_ACCEL, ADIS16495_DEVICE_PATH_GYRO,
			      PX4_SPIDEV_EXTERNAL1_1, rotation);
#else
	PX4_ERR("External SPI not available");
	exit(0);
#endif

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	exit(0);
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	sensor_accel_s a_report{};
	sensor_gyro_s g_report{};

	ssize_t sz;

	/* get the driver */
	int fd = px4_open(ADIS16495_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed", ADIS16495_DEVICE_PATH_ACCEL);
	}

	/* get the gyro driver */
	int fd_gyro = px4_open(ADIS16495_DEVICE_PATH_GYRO, O_RDONLY);

	if (fd_gyro < 0) {
		err(1, "%s open failed", ADIS16495_DEVICE_PATH_GYRO);
	}

	/* do a simple demand read */
	sz = read(fd, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		PX4_ERR("ret: %d, expected: %d", sz, sizeof(a_report));
		err(1, "immediate acc read failed");
	}

	print_message(a_report);

	/* do a simple demand read */
	sz = px4_read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(g_report));
		err(1, "immediate gyro read failed");
	}

	print_message(g_report);

	px4_close(fd_gyro);
	px4_close(fd);

	reset();
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = px4_open(ADIS16495_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed");
	}

	if (px4_ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	px4_close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'test', 'info', 'reset'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

}
// namespace

int
adis16495_main(int argc, char *argv[])
{
	enum Rotation rotation = ROTATION_NONE;
	int ch;

	/* start options */
	while ((ch = getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;

		default:
			adis16495::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		adis16495::start(rotation);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		adis16495::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		adis16495::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		adis16495::info();
	}

	adis16495::usage();
	exit(1);
}
