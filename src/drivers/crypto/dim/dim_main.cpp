#include "DIM.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>


/**
 * DIM Driver daemon show usage
 * @return
 */
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


/**
 * DIM Driver daemon command interface construtor
 * @param cli       argument of command
 * @param iterator       bus iterator
 * @param runtime_instance       instance of Device object.
 * @return instance of DIM object
 */
I2CSPIDriverBase *DIM::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				   int runtime_instance)
{
	DIM *instance = new DIM(iterator.configuredBusOption(), iterator.bus(), iterator.devid(), cli.bus_frequency,
				cli.spi_mode, iterator.DRDYGPIO());

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

/**
 * entry point of DIM Driver. main function
 * @param argc       size of arguments
 * @param argv       arguments
 * @return          success(0) or fail(-1)
 */
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
		cli.custom2 = 16; // default random count

		if (argc >= 3) {
			uint16_t count = atoi(argv[2]);
			cli.custom2 = count;
		}

		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "test")) {
		cli.custom1 = 4;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "mavlink")) {
		typedef struct {
			const char *filepath;
			uint8_t buffer[42];
		} file_io_t;
		file_io_t io = {"/fs/microsd/global_position_int.log", {}};
		memset(&(io.buffer), 0xaa, sizeof(io.buffer));

		int fd = px4_open("/dev/dim0", O_RDWR);

		if (fd < 0) {
			PX4_ERR("can't open DIM device");
			return -1;
		}

		px4_ioctl(fd, 0, &io);
		px4_close(fd);
		return 0;
	}

	ThisDriver::print_usage();
	return -1;
}
