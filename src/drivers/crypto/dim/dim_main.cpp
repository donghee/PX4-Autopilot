/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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

#include "DIM.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

void
DIM::print_usage()
{
	PRINT_MODULE_USAGE_NAME("dim", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("crypto");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	// PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}


I2CSPIDriverBase *DIM::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
		int runtime_instance)
{
	DIM *instance = new DIM(iterator.configuredBusOption(), iterator.bus(), iterator.devid(), cli.rotation,
					    cli.bus_frequency, cli.spi_mode, iterator.DRDYGPIO());

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->init()) {
		delete instance;
		return nullptr;
	}

	return instance;
}

extern "C" int dim_main(int argc, char *argv[])
{
 	using ThisDriver = DIM;
	BusCLIArguments cli{false, true};
	cli.default_spi_frequency = 4000000;
    cli.spi_mode = SPIDEV_MODE0;

    const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, 0xA0); // TODO CHANGE driver type

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "flash")) {
        cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "sd")) {
        cli.custom1 = 2;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "random")) {
        cli.custom1 = 3;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
