/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
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
 * @file ipt.h
 *
 * Shared defines for the ipt driver.
 */

/* interface ioctls */
#define IOCTL_RESET			2
#define IOCTL_MEASURE			3

/*
 * Maximum internal conversion time for OSR 1024 is 2.28 ms. We set an update
 * rate of 100Hz which is be very safe not to read the ADC before the
 * conversion finished
 */
#define IPT_CONVERSION_INTERVAL	10000	/* microseconds */
#define IPT_MEASUREMENT_RATIO	10	/* pressure measurements per temperature measurement */
#define IPT_BARO_DEVICE_PATH_EXT	"/dev/ipt_ext"

namespace ipt
{

/**
 * Calibration PROM as reported by the device.
 */
#pragma pack(push,1)


typedef struct {
	float A;
	float A1, A2, A3, A4, A5, A6;
	float B1, B2, B3, B4, B5, B6;
	float C1, C2, C3, C4, C5, C6;
	float D1, D2, D3, D4, D5, D6;
	float E1, E2, E3, E4, E5, E6;
	float FA1, FA2, FA3, FA4, FA5, FA6;
	float G1, G2, G3, G4;
} coef_s;

typedef struct {
	float A;
	float A1, A2, A3, A4, A5, A6;
	float B1, B2, B3, B4, B5, B6;
	float C1, C2, C3, C4, C5, C6;
	float D1, D2, D3, D4, D5, D6;
	float E1, E2, E3, E4, E5, E6;
	float FA1, FA2, FA3, FA4, FA5, FA6;
} p_coef_s;

typedef struct {
	float G1, G2, G3, G4;
} t_coef_s;

typedef struct {
	float A;
	float A1, A2, A3, A4, A5, A6;
	float B1, B2, B3, B4, B5, B6;
	float C1, C2, C3, C4, C5, C6;
	float D1, D2, D3, D4, D5, D6;
	float E1, E2, E3, E4, E5, E6;
	float FA1, FA2, FA3, FA4, FA5, FA6;
	float FS;
	int8_t MIN_OPER, MAX_OPER, MIN_STOR, MAX_STOR;
	uint32_t PMIN, PMAX;
	int16_t MIN_TOUT, MAX_TOUT;
	uint8_t ALGO_TYPE, DATE_MONTH, DATE_DAY, DATE_YEAR;
	uint32_t SERIAL_NO, HON_PN;
	uint8_t CHECKSUM1[4];
	float G1, G2, G3, G4;
	uint32_t SPC;
	uint32_t STC;
	float CSP;
	uint8_t CHECKSUM2[2];
} prom_s;

typedef struct {
	double PT_PSI, PS_PSI;
	double PT_TEMP, PS_TEMP;
	double PT_inHg, PS_inHg;
	double BARO_ALT, BARO_SPD;
} ipt_s;

/**
 * Grody hack for crc4()
 */
union prom_u {
	uint8_t b[(54) * 4];
	uint32_t w[54];
	prom_s s;
};
#pragma pack(pop)

extern bool crc4(uint16_t *n_prom);

} /* namespace */

/* interface factories */
extern device::Device *IPT_spi_interface(ipt::prom_u &prom_buf, uint8_t busnum);
typedef device::Device *(*IPT_constructor)(ipt::prom_u &prom_buf, uint8_t busnum);
