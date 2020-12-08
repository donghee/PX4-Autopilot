/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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

/**
 * @file ipt_spi.cpp
 *
 * SPI interface for IPT
 */

/* XXX trim includes */
#include <px4_time.h>
#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>

#include <arch/board/board.h>

#include <drivers/device/spi.h>

#include "ipt.h"
#include "board_config.h"

typedef struct {
	uint32_t pressure;
	uint32_t temperature;
	uint32_t eeprom;
} ipt_cs;

enum IPT_DEVICE {
	IPT0,
	IPT1,
};

static constexpr ipt_cs ipt0_cs_gpio = {SPI6_CS1_EXTERNAL2, SPI6_CS2_EXTERNAL2, SPI6_CS3_EXTERNAL2};
static constexpr ipt_cs ipt1_cs_gpio = {SPI6_CS4_EXTERNAL2, SPI6_CS5_EXTERNAL2, SPI6_CS6_EXTERNAL2};
static constexpr ipt_cs ipt_cs_gpio[] = {ipt0_cs_gpio, ipt1_cs_gpio};

#if defined(PX4_SPIDEV_BARO) || defined(PX4_SPIDEV_EXT_BARO)

device::Device *IPT_spi_interface(ipt::prom_u &prom_buf, bool external_bus);

class IPT_SPI : public device::SPI
{
public:
	IPT_SPI(uint8_t bus, uint32_t device, ipt::prom_u &prom_buf);
	virtual ~IPT_SPI() = default;

	virtual int	init();
	virtual int	read(unsigned offset, void *data, unsigned count);
	virtual int	ioctl(unsigned operation, unsigned &arg);
private:
	ipt::prom_u	&_prom;

	/**
	 * Send a reset command to the IPT.
	 *
	 * This is required after any bus reset.
	 */
	int		_reset();

	/**
	 * Send a measure command to the IPT.
	 *
	 */
	int		_measure();

	/**
	 * Read the IPT channels
	 *
	 * @return		OK if the PROM reads successfully.
	 */
	int     _init_pressure_channel(IPT_DEVICE ipt);
	int     _init_temperature_channel(IPT_DEVICE ipt);

	int     _read_pressure_channel(IPT_DEVICE ipt);
	int     _read_temperature_channel(IPT_DEVICE ipt);
	uint8_t     _read_eeprom(IPT_DEVICE ipt, uint8_t addr);
	int     _read_correction_coef();

	int     _read_serial_number(IPT_DEVICE ipt);
	float64 _ieee754_single(uint32_t i_data);
	/**
	 * Read a 16-bit register value.
	 *
	 * @param reg		The register to read.
	 */
	uint16_t	_reg16(unsigned reg);

	/**
	 * Wrapper around transfer() that prevents interrupt-context transfers
	 * from pre-empting us. The sensor may (does) share a bus with sensors
	 * that are polled from interrupt context (or we may be pre-empted)
	 * so we need to guarantee that transfers complete without interruption.
	 */
	int		_transfer(uint8_t *send, uint8_t *recv, unsigned len);
};

device::Device *
IPT_spi_interface(ipt::prom_u &prom_buf, uint8_t busnum)
{
	return new IPT_SPI(busnum, PX4_SPIDEV_EXTERNAL2_1, prom_buf);
}

IPT_SPI::IPT_SPI(uint8_t bus, uint32_t device, ipt::prom_u &prom_buf) :
	// SPI("IPT_SPI", nullptr, bus, device, SPIDEV_MODE3, 20 * 1000 * 1000 /* will be rounded to 10 MHz */),
	SPI("IPT_SPI", nullptr, bus, device, SPIDEV_MODE3, 5 * 1000 * 100 /* will be rounded to 1000kHz */),
	_prom(prom_buf)
{
}

int
IPT_SPI::init()
{
	int ret;

	ret = SPI::init();

	if (ret != OK) {
		PX4_DEBUG("SPI init failed");
		goto out;
	}

	// printf("ipt_cs_gpio[IPT0].pressure %d\n", ipt_cs_gpio[IPT0].pressure);
	// printf("SPI6_CS1_EXTERNAL2 %d\n", SPI6_CS1_EXTERNAL2);
	// printf("ipt_cs_gpio[IPT0].temperature %d\n", ipt_cs_gpio[IPT0].temperature);
	// printf("SPI6_CS2_EXTERNAL2 %d\n", SPI6_CS2_EXTERNAL2);
	// printf("ipt_cs_gpio[IPT0].eeprom %d\n", ipt_cs_gpio[IPT0].eeprom);
	// printf("SPI6_CS3_EXTERNAL2 %d\n", SPI6_CS3_EXTERNAL2);
	// printf("ipt_cs_gpio[IPT1].pressure %d\n", ipt_cs_gpio[IPT1].pressure);
	// printf("SPI6_CS4_EXTERNAL2 %d\n", SPI6_CS4_EXTERNAL2);
	// printf("ipt_cs_gpio[IPT1].temperature %d\n", ipt_cs_gpio[IPT1].temperature);
	// printf("SPI6_CS5_EXTERNAL2 %d\n", SPI6_CS5_EXTERNAL2);
	// printf("ipt_cs_gpio[IPT1].eeprom %d\n", ipt_cs_gpio[IPT1].eeprom);
	// printf("SPI6_CS6_EXTERNAL2 %d\n", SPI6_CS6_EXTERNAL2);

	// init chip select
	// for (int i = 0; i <= IPT1; i++) {
	//     px4_arch_configgpio(ipt_cs_gpio[i].pressure);
	//     px4_arch_configgpio(ipt_cs_gpio[i].temperature);
	//     px4_arch_configgpio(ipt_cs_gpio[i].eeprom);

	//     px4_arch_gpiowrite(ipt_cs_gpio[i].pressure, 1);
	//     px4_arch_gpiowrite(ipt_cs_gpio[i].temperature, 1);
	//     px4_arch_gpiowrite(ipt_cs_gpio[i].eeprom, 1);
	// }
	px4_arch_configgpio(SPI6_CS1_EXTERNAL2);
	px4_arch_configgpio(SPI6_CS2_EXTERNAL2);
	px4_arch_configgpio(SPI6_CS3_EXTERNAL2);
	px4_arch_configgpio(SPI6_CS4_EXTERNAL2);
	px4_arch_configgpio(SPI6_CS5_EXTERNAL2);
	px4_arch_configgpio(SPI6_CS6_EXTERNAL2);

	px4_arch_gpiowrite(SPI6_CS1_EXTERNAL2, 1);
	px4_arch_gpiowrite(SPI6_CS2_EXTERNAL2, 1);
	px4_arch_gpiowrite(SPI6_CS3_EXTERNAL2, 1);
	px4_arch_gpiowrite(SPI6_CS4_EXTERNAL2, 1);
	px4_arch_gpiowrite(SPI6_CS5_EXTERNAL2, 1);
	px4_arch_gpiowrite(SPI6_CS6_EXTERNAL2, 1);

	_read_serial_number(IPT0);
	_read_serial_number(IPT1);

	ret = _init_pressure_channel(IPT0);

	if (ret != OK) {
		PX4_DEBUG("pressure channel init failed");
		goto out;
	}

	px4_usleep(1000);
	ret = _init_temperature_channel(IPT0);

	if (ret != OK) {
		PX4_DEBUG("temperature channel init failed");
		goto out;
	}

	px4_usleep(1000);

	ret = _init_pressure_channel(IPT1);

	if (ret != OK) {
		PX4_DEBUG("pressure channel init failed");
		goto out;
	}

	px4_usleep(1000);

	ret = _init_temperature_channel(IPT1);

	if (ret != OK) {
		PX4_DEBUG("temperature channel init failed");
		goto out;
	}

	px4_usleep(1000);

out:
	return ret;
}

int
IPT_SPI::read(unsigned offset, void *data, unsigned count)
{
	union _cvt {
		uint8_t	b[4];
		uint32_t w;
	} *cvt = (_cvt *)data;
	uint8_t buf[4] = { 0, 0, 0, 0 };

	/* read the most recent measurement */
	int ret = _transfer(&buf[0], &buf[0], sizeof(buf));

	if (ret == OK) {
		/* fetch the raw value */
		cvt->b[0] = buf[3];
		cvt->b[1] = buf[2];
		cvt->b[2] = buf[1];
		cvt->b[3] = 0;

		ret = count;
	}

	return ret;
}

int
IPT_SPI::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {
	case IOCTL_RESET:
		ret = _reset();
		break;

	case IOCTL_MEASURE:
		ret = _measure();
		break;

	default:
		ret = EINVAL;
	}

	if (ret != OK) {
		errno = ret;
		return -1;
	}

	return 0;
}

int
IPT_SPI::_reset()
{
	// uint8_t cmd = ADDR_RESET_CMD | DIR_WRITE;

	// return  _transfer(&cmd, nullptr, 1);
	return OK;
}

int
IPT_SPI::_measure()
{
	_read_pressure_channel(IPT0);
	_read_pressure_channel(IPT1);

	_read_temperature_channel(IPT0);
	_read_temperature_channel(IPT1);

	return OK;
}

int
IPT_SPI::_read_serial_number(IPT_DEVICE ipt)
{
	uint32_t serial_number = 0;
	uint8_t data[4] = { 0, 0, 0, 0 };

	data[0] = _read_eeprom(ipt, 0xAC);
	data[1] = _read_eeprom(ipt, 0xAD);
	data[2] = _read_eeprom(ipt, 0xAE);
	data[3] = _read_eeprom(ipt, 0xAF);

	serial_number = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
	printf("DEVICE %d Serial No: %lu\n", ipt, serial_number);

	return OK;
}

int
IPT_SPI::_read_correction_coef()
{
	// 00~93
	// coef_s ps_coef;
	// ps_coef.A = 0;

	return OK;
}

float64
IPT_SPI::_ieee754_single(uint32_t i_data)
{
	/* IEEE 754 single - Precision Floating-point format
	 *
	 * 31 bit : sign
	 * 30 - 23 bit : exponent (8 bit)
	 * 22 - 0  bit : fraction (23 bit)
	 *
	 * example
	 * 0x41 0x29 0x02 0xDE
	 * 0100 0001 0010 1001 0000 0010 1101 1110
	 * sign = 0
	 * Exponent = 10000010
	 :_read_correction_coef     * Mantissa = 1010010000001011011110
	 *
	 * V = (-1)^s * ((1.0) + M*2^-23) * 2^(E-127)
	 */

	int16_t Sign = 0, C_A = 0;;
	int16_t Exponent = 0;
	int32_t Mantissa = 0;
	float64 Exponent_result = 0.0;
	float64 Fraction_result = 0.0, Decimal_Fraction = 0.0, ret_data;

	// First Step : sign
	if (i_data & 0x80000000) {   Sign = -1;   } // Negative

	else {                      Sign =  1;   }  // Positive

	// Second step : separation Exponent / Mantissa
	Exponent = ((i_data & 0x7F800000) >> 23) & 0xFFFF;
	Mantissa = ((i_data & 0x007FFFFF)) & 0xFFFFFF;
	C_A      = Exponent - 127;

	// Third step : calculation
	Exponent_result  = (float)(::pow(2, C_A));
	Decimal_Fraction = (float64)(::pow(2, -23));
	Fraction_result  = (float64)(Mantissa * Decimal_Fraction);
	ret_data = Sign * Exponent_result * (1 + Fraction_result);

	return ret_data;
}


int
IPT_SPI::_init_pressure_channel(IPT_DEVICE ipt)
{
	// px4_arch_gpiowrite(SPI6_CS1_EXTERNAL2, 0);
	// px4_arch_gpiowrite(SPI6_CS4_EXTERNAL2, 0);
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].pressure, 0);
	px4_usleep(1000);

	// init press AD7799
	uint8_t cmd_comm = 0x10;
	_transfer(&cmd_comm, nullptr, 1);
	px4_usleep(10);

	uint8_t cmd_conf[2];
	cmd_conf[0] = 0x10;
	cmd_conf[1] = 0x20;
	_transfer(cmd_conf, nullptr, sizeof(cmd_conf));
	px4_usleep(10);

	cmd_comm = 0x08;
	_transfer(&cmd_comm, nullptr, 1);
	px4_usleep(10);

	cmd_conf[0] = 0x30;
	cmd_conf[1] = 0x01;
	_transfer(cmd_conf, nullptr, sizeof(cmd_conf));
	px4_usleep(10);

	// px4_arch_gpiowrite(SPI6_CS1_EXTERNAL2, 1);
	// px4_arch_gpiowrite(SPI6_CS4_EXTERNAL2, 1);
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].pressure, 1);

	return OK;
}

int
IPT_SPI::_init_temperature_channel(IPT_DEVICE ipt)
{
	// px4_arch_gpiowrite(SPI6_CS2_EXTERNAL2, 0);
	// px4_arch_gpiowrite(SPI6_CS5_EXTERNAL2, 0);
	//px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 0);
	px4_usleep(1000);

	// init temperature AD7799
	uint8_t cmd_comm = 0x20;
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 0);
	_transfer(&cmd_comm, nullptr, 1);
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 1);
	px4_usleep(10);

	uint8_t cmd_filt = 0x03;
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 0);
	_transfer(&cmd_filt, nullptr, 1);
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 1);
	px4_usleep(10);

	cmd_comm = 0x10;
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 0);
	_transfer(&cmd_comm, nullptr, 1);
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 1);
	px4_usleep(10);

	uint8_t cmd_mode = 0x80;
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 0);
	_transfer(&cmd_mode, nullptr, 1);
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 1);
	px4_usleep(10);

	// px4_arch_gpiowrite(SPI6_CS2_EXTERNAL2, 1);
	// px4_arch_gpiowrite(SPI6_CS5_EXTERNAL2, 1);
	//px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 1);

	return OK;
}

int
IPT_SPI::_read_pressure_channel(IPT_DEVICE ipt)
{
	// px4_arch_gpiowrite(SPI6_CS1_EXTERNAL2, 0);
	// px4_arch_gpiowrite(SPI6_CS4_EXTERNAL2, 0);
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].pressure, 0);
	px4_usleep(10000); // 10ms

	uint8_t cmd_comm = 0x58;
	_transfer(&cmd_comm, nullptr, 1);
	px4_usleep(10);

	uint8_t cmd[3] = { 0, 0, 0 };
	_transfer(nullptr, cmd, sizeof(cmd));
	px4_usleep(10);

	cmd_comm = 0x08;
	_transfer(&cmd_comm, nullptr, 1);
	px4_usleep(10);

	uint8_t cmd_conf[2];
	cmd_conf[0] = 0x30;
	cmd_conf[1] = 0x01;
	_transfer(cmd_conf, nullptr, sizeof(cmd_conf));
	px4_usleep(1000); // 1ms

	// px4_arch_gpiowrite(SPI6_CS1_EXTERNAL2, 1);
	// px4_arch_gpiowrite(SPI6_CS4_EXTERNAL2, 1);
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].pressure, 1);

	printf("DEVICE %d PRESSURE BYTES: %x %x %x\n", ipt, cmd[0], cmd[1], cmd[2]);
	uint32_t pressure = (cmd[0] << 16) | (cmd[1] << 8) | cmd[2];
	printf("DEVICE %d PRESSURE RAW: %lu\n", ipt, (unsigned long)pressure);

	return OK;
}

int
IPT_SPI::_read_temperature_channel(IPT_DEVICE ipt)
{
	// px4_arch_gpiowrite(SPI6_CS2_EXTERNAL2, 0);
	// px4_arch_gpiowrite(SPI6_CS5_EXTERNAL2, 0);
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 0);
	px4_usleep(100000); // 100ms

	uint8_t cmd_comm = 0x38;
	_transfer(&cmd_comm, nullptr, 1);

	uint8_t cmd[2] = { 0, 0 };
	_transfer(nullptr, cmd, sizeof(cmd));

	cmd_comm = 0x10;
	_transfer(&cmd_comm, nullptr, 1);

	uint8_t cmd_mode = 0x80;
	_transfer(&cmd_mode, nullptr, 1);
	px4_usleep(1000); // 1ms

	// px4_arch_gpiowrite(SPI6_CS2_EXTERNAL2, 1);
	// px4_arch_gpiowrite(SPI6_CS5_EXTERNAL2, 1);
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 1);

	printf("DEVICE %d TEMPERATURE BYTES: %x %x\n", ipt, cmd[0], cmd[1]);
	uint16_t temperature = (cmd[0] << 8) | cmd[1];
	printf("DEVICE %d TEMPERATURE RAW: %lu\n", ipt, (unsigned long)temperature);

	return OK;
}


uint8_t
IPT_SPI::_read_eeprom(IPT_DEVICE ipt, uint8_t addr)
{
	// px4_arch_gpiowrite(SPI6_CS3_EXTERNAL2, 0);
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].eeprom, 0);
	px4_usleep(1); // 1us

	uint8_t cmd_comm = 0x03;
	_transfer(&cmd_comm, nullptr, 1);

	uint8_t _addr = addr;
	_transfer(&_addr, nullptr, 1);

	uint8_t data;
	_transfer(nullptr, &data, 1);
	px4_usleep(10); // 10us

	// px4_arch_gpiowrite(SPI6_CS3_EXTERNAL2, 1);
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].eeprom, 1);
	return data;
}

uint16_t
IPT_SPI::_reg16(unsigned reg)
{
	uint8_t cmd[3] = { 0, 0, 0 };

	_transfer(cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[1] << 8) | cmd[2];
}

int
IPT_SPI::_transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	int ret;

	ret = transfer(send, recv, len);

	return ret;
}

#endif /* PX4_SPIDEV_BARO || PX4_SPIDEV_EXT_BARO */
