/**
 * @file max11615.cpp
 *
 * Driver for MAX11615 ADC: read battery cell voltages
 *
 * @author Donghee Park <dongheepark@gmail.com>
 *
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>

#include <board_config.h>
#include <drivers/device/device.h>

#include <systemlib/err.h>

#define MAX11615_I2C_BUS					4
#define MAX11615_I2C_BUS_SPEED					100000
#define MAX11615_I2C_ADDR					0x33
#define devicepath						"/dev/max11615"

//#define MAX_CELL_V_REF							3.3
//#define MAX_CELL_V_REF							4.2 // when R2 is 12.7k, R1 is 10k
#define MAX_CELL_V_REF							4.29 // when R2 is 13k, R1 is 10k

// TODO Fix Opamp circuit to read maximum cell voltage (4.2v) 2021. 4
// classic difference amplifer
// https://www.analog.com/en/analog-dialogue/articles/deeper-look-into-difference-amplifiers.html
// Vout = (R2/R1)(V2-V1)
// R2 = 12.7k, R1 = 10k

class MAX11615 : public device::I2C
{
public:
	MAX11615(int bus, const char *_device_path);
	~MAX11615();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);
	virtual int	init();

	unsigned int  max11615_check(unsigned int count, bool showOutput, bool initI2C);
	unsigned int  max11615_read(uint16_t *adc_result, uint16_t len);

private:
	static const bool		showDebug = false;

	int						_task;
	char					_device[20];

	//orb_advert_t			_t_cell_status;

	static int				task_main_trampoline(int argc, char *argv[]);
	int					task_main();

	bool		_task_should_exit;
};


namespace
{

MAX11615	*g_max11615;

} // namespace

MAX11615::MAX11615(int bus, const char *_device_path) :
	I2C("max11615", "/dev/max11615", bus, MAX11615_I2C_ADDR, MAX11615_I2C_BUS_SPEED),
	_task(-1),
	_task_should_exit(false)
{
	strncpy(_device, _device_path, sizeof(_device));
	/* enforce null termination */
	_device[sizeof(_device) - 1] = '\0';
}

MAX11615::~MAX11615()
{
	if (_task != -1) {
		/* tell the task we want it to go away */
		_task_should_exit = true;

		unsigned i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);

			/* if we have given up, kill it */
			if (--i == 0) {
				task_delete(_task);
				break;
			}

		} while (_task != -1);
	}

	g_max11615 = nullptr;
}

int
MAX11615::init()
{
	int ret;
	ASSERT(_task == -1);

	ret = I2C::init();

	if (ret != OK) {
		warnx("I2C init failed");
		return ret;
	}

	usleep(500000);

	if (sizeof(_device) > 0) {
		ret = register_driver(_device, &fops, 0666, (void *)this);

		if (ret == OK) {
			DEVICE_LOG("creating adc device");
		}

	}

	/* start the ADC task */
	_task = px4_task_spawn_cmd("max11615",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_MAX - 20,
				   1500,
				   (main_t)&MAX11615::task_main_trampoline,
				   nullptr);


	if (_task < 0) {
		DEVICE_DEBUG("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

int
MAX11615::task_main_trampoline(int argc, char *argv[])
{
	return g_max11615->task_main();
}

int
MAX11615::task_main()
{
	uint16_t buffer[8];
	uint16_t len = 8;

	/*
	 * advertise the battery cell voltage status.
	 */

	/*
	batt_cell_status_s cell_status;
	memset(&cell_status, 0, sizeof(cell_status));
	_t_cell_status = orb_advertise(ORB_ID(batt_cell_status), &cell_status);
	*/

	uint8_t setup_configure[2] = {0x80, 0x0f}; // SETUP: 0b1000 0000, CONFIGURE: 0b0000 1111 ; read 8 channels

	if (OK == transfer(&setup_configure[0], 1, NULL, 0)) {
		printf("[max11615] setup succeed \n");
	}

	if (OK == transfer(&setup_configure[1], 1, NULL, 0)) {
		printf("[max11615] configure succeed \n");
	}

	/* loop untill killed */
	while (!_task_should_exit) {

		max11615_read((uint16_t *) buffer, len);

		for (unsigned i = 0; i < 5; i++) {
			printf("[BAT CELL %d]: %.2f v \t", i + 1, (double)buffer[i] / 4095.0 * MAX_CELL_V_REF);
		}

		printf("\n");

		/*
		* publish the battery cell voltage status.

		cell_status.timestamp = hrt_absolute_time();
		cell_status.cell_count = 5;
		cell.cell1 = 3.3f;
		cell.cell2 = 3.3f;
		cell.cell3 = 3.3f;
		cell.cell4 = 3.3f;
		cell.cell5 = 3.3f;

		orb_publish(ORB_ID(cell_status), _t_cell_status, &cell_status);
		*/

		usleep(500000);
	}

	_task = -1;
	return 0;
}


unsigned int
MAX11615::max11615_read(uint16_t *adc_result, uint16_t len)
{
	uint8_t result[16];

	set_device_address(MAX11615_I2C_ADDR);

	if (OK == transfer(NULL, 0, &result[0], 16)) {
		for (unsigned i = 0; i < len; i++) {
			adc_result[i] = (uint16_t)(result[i * 2] & 0x0F) << 8 | result[i * 2 + 1];
		}
	}

	return 1;
}


unsigned int
MAX11615::max11615_check(unsigned int count, bool showOutput, bool initI2C)
{
	if (initI2C) {
		I2C::init();
	}

	uint8_t foundCount = 0;

	uint8_t msg = 0;
	uint8_t result[2];

	for (unsigned i = 0; i < count; i++) {
		set_device_address(MAX11615_I2C_ADDR);

		if (OK == transfer(&msg, 0, &result[0], 2)) {
			if (showOutput == true) {
				fprintf(stderr, "result: 0x%x, 0x%x\n", result[0], result[0]);
			}

			foundCount = 1;
		}
	}

	return foundCount;
}

int
MAX11615::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	ret = CDev::ioctl(filp, cmd, arg);

	return ret;
}


int
max11615_start(const char *device_path)
{
	int ret;

	// try i2c4 first
	g_max11615 = new MAX11615(MAX11615_I2C_BUS, device_path);

	if (!g_max11615) {
		return -ENOMEM;
	}

	if (OK == g_max11615->init()) {
		warnx("[max11615] scanning i2c4...\n");
		ret = g_max11615->max11615_check(2, true, false);

		if (ret > 0) {
			return OK;
		}
	}

	delete g_max11615;
	g_max11615 = nullptr;

	return -ENXIO;
}


extern "C" __EXPORT int max11615_main(int argc, char *argv[]);

int
max11615_main(int argc, char *argv[])
{
	if (max11615_start(devicepath) != OK) {
		errx(1, "failed to start the MAX11615 driver");
	}

	exit(0);
}
