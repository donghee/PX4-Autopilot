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
#include <string.h>

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
	IPT0, // PS
	IPT1, // PT
};

#define PS_Coef ipt_coef[0]
#define PT_Coef ipt_coef[1]

/**
 * bswap32: convert endian. eeprom store data as big-endian. arm-cortex use little-endian.
 */
typedef union { uint8_t a[4]; uint32_t b; } __bitcast_u32;

static inline uint32_t be32(uint32_t x)
{
	__bitcast_u32 y = {
		.a = {
			(uint8_t)(x >> 24), (uint8_t)(x >> 16),
			(uint8_t)(x >> 8), (uint8_t)(x)
		}
	};
	return y.b;
}

static uint32_t bswap32(uint32_t x) { return be32(x); };

/**
 * spi cs
 */
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
	ipt::coef_s ipt_coef[2];
	ipt::ipt_s IPT_DATA;

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
	int     _reset_channel(IPT_DEVICE ipt);
	int     _init_temperature_channel(IPT_DEVICE ipt);

	int _ipt_correction(IPT_DEVICE ipt, double PT_PSI, double PS_PSI, double PT_TEMP, double PS_TEMP);
	uint32_t     _read_pressure_channel(IPT_DEVICE ipt);
	uint16_t     _read_temperature_channel(IPT_DEVICE ipt);
	uint8_t     _read_eeprom(IPT_DEVICE ipt, uint8_t addr);

	int     _read_serial_number(IPT_DEVICE ipt);
	int     _read_prom(IPT_DEVICE ipt);


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
	SPI("IPT_SPI", nullptr, bus, device, SPIDEV_MODE3, 1 * 1000 * 1000 /* will be rounded to 1 MHz */),
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

	// init chip select
	for (int i = IPT0; i <= IPT1; i++) {
		px4_arch_configgpio(ipt_cs_gpio[i].pressure);
		px4_arch_configgpio(ipt_cs_gpio[i].temperature);
		px4_arch_configgpio(ipt_cs_gpio[i].eeprom);

		px4_arch_gpiowrite(ipt_cs_gpio[i].pressure, 1);
		px4_arch_gpiowrite(ipt_cs_gpio[i].temperature, 1);
		px4_arch_gpiowrite(ipt_cs_gpio[i].eeprom, 1);
	}

	_read_serial_number(IPT0);
	_read_serial_number(IPT1);

	_read_prom(IPT0);
	_read_prom(IPT1);

	for (int i = 0 ; i < 10; i++) {
		ret = _init_pressure_channel(IPT0);
		px4_usleep(20);
		ret = _reset_channel(IPT0);
		px4_usleep(20);
		ret = _init_pressure_channel(IPT0);
		px4_usleep(20);
	}

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

	for (int i = 0 ; i < 10; i++) {
		ret = _init_pressure_channel(IPT1);
		px4_usleep(20);
		ret = _reset_channel(IPT1);
		px4_usleep(20);
		ret = _init_pressure_channel(IPT1);
		px4_usleep(20);
	}

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
	int ret;

	ipt::ipt_s *cvt = (ipt::ipt_s *)data;

	// int ret = _measure();

	// if (ret == OK) {
	cvt->PT_PSI = IPT_DATA.PT_PSI;
	cvt->PS_PSI = IPT_DATA.PS_PSI;
	cvt->PT_TEMP = IPT_DATA.PT_TEMP;
	cvt->PS_TEMP = IPT_DATA.PS_TEMP;

	cvt->PT_inHg = IPT_DATA.PT_inHg;
	cvt->PS_inHg = IPT_DATA.PS_inHg;

	cvt->BARO_ALT = IPT_DATA.BARO_ALT;
	cvt->BARO_SPD = IPT_DATA.BARO_SPD;

	ret = count;
	// }

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
	double ipt0_pressure, ipt1_pressure;
	static double ipt0_temperature, ipt1_temperature;
	double IPT_inHg;
	int tmp;

	static uint8_t read_temp_count = 0;

	ipt0_pressure = (double) _read_pressure_channel(IPT0) / (double)16777215.0;
	ipt1_pressure = (double) _read_pressure_channel(IPT1) / (double)16777215.0;

	// read temperatures every 10 measures
	if (read_temp_count % IPT_MEASUREMENT_RATIO == 0) {
		ipt0_temperature = (double) _read_temperature_channel(IPT0) / (double)65535.0;
		ipt1_temperature = (double) _read_temperature_channel(IPT1) / (double)65535.0;
		read_temp_count = 0;
	}

	read_temp_count = read_temp_count + 1;

	_ipt_correction(IPT0, ipt1_pressure, ipt0_pressure, ipt1_temperature, ipt0_temperature);
	_ipt_correction(IPT1, ipt1_pressure, ipt0_pressure, ipt1_temperature, ipt0_temperature);

	printf("PS PSI CALIBRATED: %9.6f\n", IPT_DATA.PS_PSI);
	printf("PT PSI CALIBRATED: %9.6f\n", IPT_DATA.PT_PSI);
	printf("PS TEMPERATURE: %9.6f\n", IPT_DATA.PS_TEMP);
	printf("PT TEMPERATURE: %9.6f\n", IPT_DATA.PT_TEMP);

	IPT_DATA.PS_inHg = IPT_DATA.PS_PSI * 2.03602;
	IPT_DATA.PT_inHg = IPT_DATA.PT_PSI * 2.03602;

	IPT_DATA.BARO_ALT = ((1.909029114 - pow(IPT_DATA.PS_inHg, 0.190255)) / 0.000013125214) * 0.3048;

	tmp = IPT_DATA.PT_PSI - IPT_DATA.PS_PSI;

	tmp = (tmp < 0) ? 0 : tmp;

	// PSI to inhg : 1 PSI to 2.03602 inHg
	IPT_inHg = (tmp) * 2.03602;
	// Knot to Km/h coefficient is 1.852
	IPT_DATA.BARO_SPD = (1479.1026 * sqrt(pow((IPT_inHg / 29.92126 + 1.0), 0.285714286) - 1.0)) * 1.852;

	printf("Altitude: %9.6f\n", IPT_DATA.BARO_ALT);
	printf("Velocity: %9.6f\n", IPT_DATA.BARO_SPD);

	return OK;
}

int
IPT_SPI::_read_serial_number(IPT_DEVICE ipt)
{
	union serial_number_u {
		uint8_t b[4];
		uint32_t w;
	};
	serial_number_u s;
	uint32_t serial_number;

	s.b[0] = _read_eeprom(ipt, 0xAC);
	s.b[1] = _read_eeprom(ipt, 0xAD);
	s.b[2] = _read_eeprom(ipt, 0xAE);
	s.b[3] = _read_eeprom(ipt, 0xAF);

	serial_number = bswap32(s.w);
	printf("DEVICE %d Serial No: %lu\n", ipt, serial_number);

	return OK;
}

int
IPT_SPI::_read_prom(IPT_DEVICE ipt)
{
	// read all data from eeprom 0x00 - 0xd7
	for (int i = 0; i < (54) * 4; i++) {
		_prom.b[i] = _read_eeprom(ipt, i);
	}

	// change eeprom's byte order(big endian) to cpu's byte order(little endian) for pressure coffecients (0x00-0x93)
	for (int i = 0; i < 37; i++) {
		_prom.w[i] = bswap32(_prom.w[i]);
	}

	// change eeprom's byte order for serial number
	_prom.w[43] = bswap32(_prom.w[43]);

	// change eeprom's byte order for product number
	_prom.w[44] = bswap32(_prom.w[44]);

	// change eeprom's byte order for temperature coffecients
	for (int i = 46; i < 53; i++) {
		_prom.w[i] = bswap32(_prom.w[i]);
	}

	// copy pressure coffecients (0x00-0x93)
	memcpy(&(ipt_coef[ipt].A), &(_prom.s.A), sizeof(ipt::p_coef_s));
	// printf("DEVICE %d EEPROM A: %9.6f\n", ipt, (double)(_prom.s.A));
	// printf("DEVICE %d EEPROM A: %9.6f\n", ipt, (double)(ipt_coef[ipt].A));
	// printf("DEVICE %d EEPROM FA6: %9.6f\n", ipt, (double)(_prom.s.FA6));
	// printf("DEVICE %d EEPROM FA6: %9.6f\n", ipt, (double)(ipt_coef[ipt].FA6));

	// uint32_t serial_number = _prom.s.SERIAL_NO;
	// printf("DEVICE %d Serial No: %lu\n", ipt, serial_number);

	// uint32_t product_number = _prom.s.HON_PN;
	// printf("DEVICE %d Product No: 220%05lu-0%02d-T%03d\n", ipt, ((product_number >> 8) / 100), ((product_number >> 8) % 100), product_number & 0xff );

	// copy temperature coffecients (0x46-0x52)
	memcpy(&(ipt_coef[ipt].G1), &(_prom.s.G1), sizeof(ipt::t_coef_s));
	// printf("DEVICE %d EEPROM G1: %9.6f\n", ipt, (double)(_prom.s.G1));
	// printf("DEVICE %d EEPROM G1: %9.6f\n", ipt, (double)(ipt_coef[ipt].G1));
	// printf("DEVICE %d EEPROM G4: %9.6f\n", ipt, (double)(_prom.s.G4));
	// printf("DEVICE %d EEPROM G4: %9.6f\n", ipt, (double)(ipt_coef[ipt].G4));

	// float porint conversion test
	// _prom.b[0] = 0xC1;
	// _prom.b[1] = 0x24;
	// _prom.b[2] = 0x06;
	// _prom.b[3] = 0xBD;
	// printf("DEVICE %d EEPROM A: %x, %x, %x, %x\n", ipt, _prom.b[0], _prom.b[1], _prom.b[2], _prom.b[3]);
	// _prom.w[0] = bswap32(_prom.w[0]);
	// printf("DEVICE %d EEPROM A: %9.6f\n", ipt, (double)_prom.s.A); // expect -10.251645

	// _prom.b[4] = 0xC4;
	// _prom.b[5] = 0xE0;
	// _prom.b[6] = 0x9E;
	// _prom.b[7] = 0x17;
	// printf("DEVICE %d EEPROM A: %x, %x, %x, %x\n", ipt, _prom.b[4], _prom.b[5], _prom.b[6], _prom.b[7]);
	// _prom.w[1] = bswap32(_prom.w[1]);
	// printf("DEVICE %d EEPROM A: %9.6f\n", ipt, (double)_prom.s.A1); // expect -1796.940308

	return OK;
}


int
IPT_SPI::_init_pressure_channel(IPT_DEVICE ipt)
{
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].pressure, 0);
	px4_usleep(1000);

	uint8_t cmd_conf[6];
	cmd_conf[0] = 0x10;
	cmd_conf[1] = 0x10;
	cmd_conf[2] = 0x20;
	cmd_conf[3] = 0x08;
	cmd_conf[4] = 0x30;
	cmd_conf[5] = 0x01;
	_transfer(cmd_conf, nullptr, sizeof(cmd_conf));
	px4_usleep(10);

	px4_arch_gpiowrite(ipt_cs_gpio[ipt].pressure, 1);

	return OK;
}

/*
int
IPT_SPI::_init_pressure_channel(IPT_DEVICE ipt)
{
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

	px4_arch_gpiowrite(ipt_cs_gpio[ipt].pressure, 1);

	return OK;
}
*/


int
IPT_SPI::_reset_channel(IPT_DEVICE ipt)
{
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].pressure, 0);
	px4_usleep(1000);

	uint8_t cmd_conf[4];
	cmd_conf[0] = 0xff;
	cmd_conf[1] = 0xff;
	cmd_conf[2] = 0xff;
	cmd_conf[3] = 0xff;
	_transfer(cmd_conf, nullptr, sizeof(cmd_conf));
	px4_usleep(10);

	px4_arch_gpiowrite(ipt_cs_gpio[ipt].pressure, 1);

	return OK;
}



int
IPT_SPI::_init_temperature_channel(IPT_DEVICE ipt)
{
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 0);
	px4_usleep(1000);

	// init temperature AD7799
	uint8_t cmd_comm = 0x20;
	_transfer(&cmd_comm, nullptr, 1);
	px4_usleep(10);

	uint8_t cmd_filt = 0x03;
	_transfer(&cmd_filt, nullptr, 1);
	px4_usleep(10);

	cmd_comm = 0x10;
	_transfer(&cmd_comm, nullptr, 1);
	px4_usleep(10);

	uint8_t cmd_mode = 0x80;
	_transfer(&cmd_mode, nullptr, 1);
	px4_usleep(10);

	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 1);

	return OK;
}

int
IPT_SPI::_ipt_correction(IPT_DEVICE ipt, double PT_PSI, double PS_PSI, double PT_TEMP, double PS_TEMP)
{
	// Adaptation Algorithm #1
	// Y = A + (F1 x p) + (F2 x p^2) + (F3 x p^2) (F4 x p^2) (F5 x p^2) + (F6 x p^2)

	double F1, F2, F3, F4, F5, F6;

	if (ipt) { // 1

		F1 = (double)PT_Coef.A1 + PT_TEMP * ((double)PT_Coef.B1 + PT_TEMP * ((double)PT_Coef.C1 + PT_TEMP * ((
				double)PT_Coef.D1 + PT_TEMP * ((double)PT_Coef.E1 + PT_TEMP * ((double)PT_Coef.FA1)))));
		F2 = (double)PT_Coef.A2 + PT_TEMP * ((double)PT_Coef.B2 + PT_TEMP * ((double)PT_Coef.C2 + PT_TEMP * ((
				double)PT_Coef.D2 + PT_TEMP * ((double)PT_Coef.E2 + PT_TEMP * ((double)PT_Coef.FA2)))));
		F3 = (double)PT_Coef.A3 + PT_TEMP * ((double)PT_Coef.B3 + PT_TEMP * ((double)PT_Coef.C3 + PT_TEMP * ((
				double)PT_Coef.D3 + PT_TEMP * ((double)PT_Coef.E3 + PT_TEMP * ((double)PT_Coef.FA3)))));
		F4 = (double)PT_Coef.A4 + PT_TEMP * ((double)PT_Coef.B4 + PT_TEMP * ((double)PT_Coef.C4 + PT_TEMP * ((
				double)PT_Coef.D4 + PT_TEMP * ((double)PT_Coef.E4 + PT_TEMP * ((double)PT_Coef.FA4)))));
		F5 = (double)PT_Coef.A5 + PT_TEMP * ((double)PT_Coef.B5 + PT_TEMP * ((double)PT_Coef.C5 + PT_TEMP * ((
				double)PT_Coef.D5 + PT_TEMP * ((double)PT_Coef.E5 + PT_TEMP * ((double)PT_Coef.FA5)))));
		F6 = (double)PT_Coef.A6 + PT_TEMP * ((double)PT_Coef.B6 + PT_TEMP * ((double)PT_Coef.C6 + PT_TEMP * ((
				double)PT_Coef.D6 + PT_TEMP * ((double)PT_Coef.E6 + PT_TEMP * ((double)PT_Coef.FA6)))));

		IPT_DATA.PT_PSI = (double)PT_Coef.A + PT_PSI * (F1 + PT_PSI * (F2 + PT_PSI * (F3 + PT_PSI * (F4 + PT_PSI *
				  (F5 + PT_PSI * (F6))))));
		IPT_DATA.PT_TEMP = (double)PT_Coef.G1 + PT_TEMP * ((double)PT_Coef.G2 + PT_TEMP * ((double)PT_Coef.G3 + PT_TEMP * ((
					   double)PT_Coef.G4)));

	} else { // 0

		F1 = (double)PS_Coef.A1 + PS_TEMP * ((double)PS_Coef.B1 + PS_TEMP * ((double)PS_Coef.C1 + PS_TEMP * ((
				double)PS_Coef.D1 + PS_TEMP * ((double)PS_Coef.E1 + PS_TEMP * ((double)PS_Coef.FA1)))));
		F2 = (double)PS_Coef.A2 + PS_TEMP * ((double)PS_Coef.B2 + PS_TEMP * ((double)PS_Coef.C2 + PS_TEMP * ((
				double)PS_Coef.D2 + PS_TEMP * ((double)PS_Coef.E2 + PS_TEMP * ((double)PS_Coef.FA2)))));
		F3 = (double)PS_Coef.A3 + PS_TEMP * ((double)PS_Coef.B3 + PS_TEMP * ((double)PS_Coef.C3 + PS_TEMP * ((
				double)PS_Coef.D3 + PS_TEMP * ((double)PS_Coef.E3 + PS_TEMP * ((double)PS_Coef.FA3)))));
		F4 = (double)PS_Coef.A4 + PS_TEMP * ((double)PS_Coef.B4 + PS_TEMP * ((double)PS_Coef.C4 + PS_TEMP * ((
				double)PS_Coef.D4 + PS_TEMP * ((double)PS_Coef.E4 + PS_TEMP * ((double)PS_Coef.FA4)))));
		F5 = (double)PS_Coef.A5 + PS_TEMP * ((double)PS_Coef.B5 + PS_TEMP * ((double)PS_Coef.C5 + PS_TEMP * ((
				double)PS_Coef.D5 + PS_TEMP * ((double)PS_Coef.E5 + PS_TEMP * ((double)PS_Coef.FA5)))));
		F6 = (double)PS_Coef.A6 + PS_TEMP * ((double)PS_Coef.B6 + PS_TEMP * ((double)PS_Coef.C6 + PS_TEMP * ((
				double)PS_Coef.D6 + PS_TEMP * ((double)PS_Coef.E6 + PS_TEMP * ((double)PS_Coef.FA6)))));

		IPT_DATA.PS_PSI = (double)PS_Coef.A + PS_PSI * (F1 + PS_PSI * (F2 + PS_PSI * (F3 + PS_PSI * (F4 + PS_PSI *
				  (F5 + PS_PSI * (F6))))));
		IPT_DATA.PS_TEMP = (double)PS_Coef.G1 + PS_TEMP * ((double)PS_Coef.G2 + PS_TEMP * ((double)PS_Coef.G3 + PS_TEMP * ((
					   double)PS_Coef.G4)));

	}

	return OK;
}

uint32_t
IPT_SPI::_read_pressure_channel(IPT_DEVICE ipt)
{
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].pressure, 0);
	// px4_usleep(10000); // 10ms

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
	// px4_usleep(1000); // 1ms

	px4_arch_gpiowrite(ipt_cs_gpio[ipt].pressure, 1);

	printf("DEVICE %d PRESSURE BYTES: %x %x %x\n", ipt, cmd[0], cmd[1], cmd[2]);
	uint32_t pressure = (cmd[0] << 16) | (cmd[1] << 8) | cmd[2];
	printf("DEVICE %d PRESSURE RAW: %lu\n", ipt, (unsigned long)pressure);

	return pressure;
}

uint16_t
IPT_SPI::_read_temperature_channel(IPT_DEVICE ipt)
{
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 0);
	// px4_usleep(100000); // new temperature value will be available in ~100ms

	uint8_t cmd_comm = 0x38;
	_transfer(&cmd_comm, nullptr, 1);

	uint8_t cmd[2] = { 0, 0 };
	_transfer(nullptr, cmd, sizeof(cmd));

	cmd_comm = 0x10;
	_transfer(&cmd_comm, nullptr, 1);

	uint8_t cmd_mode = 0x80;
	_transfer(&cmd_mode, nullptr, 1);
	// px4_usleep(1000); // 1ms

	px4_arch_gpiowrite(ipt_cs_gpio[ipt].temperature, 1);

	// printf("DEVICE %d TEMPERATURE BYTES: %x %x\n", ipt, cmd[0], cmd[1]);
	uint16_t temperature = (cmd[0] << 8) | cmd[1];
	// printf("DEVICE %d TEMPERATURE RAW: %lu\n", ipt, (unsigned long)temperature);

	return temperature;
}


uint8_t
IPT_SPI::_read_eeprom(IPT_DEVICE ipt, uint8_t addr)
{
	px4_arch_gpiowrite(ipt_cs_gpio[ipt].eeprom, 0);
	px4_usleep(1); // 1us

	uint8_t cmd_comm = 0x03;
	_transfer(&cmd_comm, nullptr, 1);

	uint8_t _addr = addr;
	_transfer(&_addr, nullptr, 1);

	uint8_t data;
	_transfer(nullptr, &data, 1);
	px4_usleep(10); // 10us

	px4_arch_gpiowrite(ipt_cs_gpio[ipt].eeprom, 1);
	return data;
}


int
IPT_SPI::_transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	int ret;

	ret = transfer(send, recv, len);

	return ret;
}

#endif /* PX4_SPIDEV_BARO || PX4_SPIDEV_EXT_BARO */
