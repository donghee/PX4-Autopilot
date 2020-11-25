/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
n *
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

/*
 * DIM.hpp
 *
 */

#pragma once

#include <lib/cdev/CDev.hpp>

#include <drivers/device/spi.h>
#include <perf/perf_counter.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_air_data.h>

class DIM : public device::SPI, public I2CSPIDriver<DIM>
{
public:
	DIM(I2CSPIBusOption bus_option, int bus, int32_t device, enum Rotation rotation, int bus_frequency,
		  spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio);
	virtual ~DIM();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int		init();

	void			print_status() override;

	void			RunImpl();

    int16_t _kcmvpDrbg(uint8_t *pbRandom, uint16_t usRandomSize);
    int16_t _kcmvpGcm(uint8_t *pbOutput, uint8_t *pbInput, uint16_t usInputSize,
                      uint8_t bKeyType, uint16_t usKeyIndex, uint8_t *pbIv,
                      uint16_t usIvSize, uint8_t *pbAuth, uint16_t usAuthSize,
                      uint8_t *pbTag, uint16_t usTagSize, uint8_t bEnDe,
                      uint8_t bAlg);

    void custom_method(const BusCLIArguments &cli) override;
    int open(struct file *filep);
	int		ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

protected:
	int		probe() override;
	void exit_and_cleanup() override;


private:
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;

	const spi_drdy_gpio_t _drdy_gpio;

    bool    is_power_on;

    // BITS SWAP
    typedef union { uint8_t a[2]; uint16_t b; } __bitcast_u16;

    static inline uint16_t le16(uint16_t x)
    {
        __bitcast_u16 y = {
            .a = { (uint8_t)(x), (uint8_t)(x >> 8) }
        };
        return y.b;
    }

    static inline uint16_t be16(uint16_t x)
    {
        __bitcast_u16 y = {
            .a = { (uint8_t)(x >> 8), (uint8_t)(x) }
        };
        return y.b;
    }

    // #if __BYTE_ORDER == __LITTLE_ENDIAN
    uint16_t bswap16(uint16_t x) { return be16(x); }
    // #endif

    // #if __BYTE_ORDER == __BIG_ENDIAN
    // uint16_t bswap16(uint16_t x) { return le16(x); }
    // #endif


    #pragma pack(push, 1) // Ensure proper memory alignment.
	struct DimPowerReport {
        uint8_t stx;
        uint8_t dir;
        uint8_t offset;
        uint8_t len;
		uint16_t rv;
		uint8_t propriety_data1[5];
		uint8_t version[3];
		uint8_t life_cycle;
		uint8_t serial_number[8];
		uint8_t pin_type;
		uint8_t max_pin_retry_count;
        uint16_t kcmvp_key_count;
        uint16_t cert_key_count;
        uint16_t io_data_size;
        uint16_t info_file_size;
        uint8_t padding[31];
	} dim_power_report{};
    #pragma pack(pop)

    #pragma pack(push, 1) // Ensure proper memory alignment.
	struct DimReport {
        uint8_t stx;
        uint8_t dir;
        uint8_t offset;
        uint8_t len;
        uint8_t data[60];
	} dim_report{};
    #pragma pack(pop)

#pragma pack(push, 1) // Ensure proper memory alignment.
	struct Command {
        uint8_t stx;
        uint8_t dir;
        uint8_t offset;
        uint8_t len;
        union {
            uint16_t msgid;
            uint8_t msg[2];
        };
        uint8_t data[58];
	} dim_cmd{};
#pragma pack(pop)

	/**
	 * Start automatic measurement.
	 */
	void			start();
    void            stop();
 	int			power_on();
	int			power_off();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	int			measure();

    int        encrypt_test(const char* file_name, uint8_t* _plain_text, size_t count);
    int        encrypt_self_test();

  	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
  	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _home_sub{ORB_ID(home_position)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	int			_class_instance;

    uint8_t buffer[512];

};
