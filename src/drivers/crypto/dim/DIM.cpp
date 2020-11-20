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

// Stall time between SPI transfers
static constexpr uint8_t T_STALL = 16;

static constexpr uint32_t DIM_DEFAULT_RATE = 10;

using namespace time_literals;

DIM::DIM(I2CSPIBusOption bus_option, int bus, int32_t device, enum Rotation rotation, int bus_frequency,
		     spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
	SPI(0xA0, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_bad_transfers(perf_alloc(PC_COUNT, MODULE_NAME": bad transfers")),
	_drdy_gpio(drdy_gpio)
{
}

DIM::~DIM()
{
	// delete the perf counters
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
}

int
DIM::init()
{
	int ret = SPI::init();

	if (ret != OK) {
		// if probe/setup failed, bail now
		DEVICE_DEBUG("SPI init failed (%i)", ret);
		return ret;
	}

	start();

	return PX4_OK;
}

int DIM::reset()
{
	return OK;
}

int DIM::power_on()
{
  // Power ON
  memset(&dim_cmd, 0, sizeof(dim_cmd));
  dim_cmd.stx = 0xa5;
  dim_cmd.dir = 0x05;
  dim_cmd.offset = 0;
  dim_cmd.len = 0x2;
  dim_cmd.msgid = 0;

  transfer((uint8_t*)&dim_cmd, nullptr, sizeof(dim_cmd));

  up_mdelay(1000);

  // read
  transfer(nullptr, (uint8_t*)&dim_power_report, sizeof(dim_power_report));

  dim_power_report.rv = bswap16(dim_power_report.rv);

  // read
  // printf("EXPECT 0xa5 0x06 0x00 0x1d 0x6f 0x1c 0x82 0x01..?: %x %x %x %x %d \n",
  //        dim_power_report.stx, dim_power_report.dir, dim_power_report.offset, dim_power_report.len,
  //        (int16_t)dim_power_report.rv);

  if(dim_power_report.len == 0x02 && dim_power_report.rv == 0) {
    printf("Power On\r\n");
  }

  if(dim_power_report.len == 0x1d) {
      // byte swap
      dim_power_report.kcmvp_key_count = bswap16(dim_power_report.kcmvp_key_count);
      dim_power_report.cert_key_count = bswap16(dim_power_report.cert_key_count);
      dim_power_report.io_data_size = bswap16(dim_power_report.io_data_size);
      dim_power_report.info_file_size = bswap16(dim_power_report.info_file_size);

      printf("  * Version    : %X.%02X.%02X\r\n",
             dim_power_report.version[0], dim_power_report.version[1], dim_power_report.version[2]);
      printf("  * Serial Number    : %02X%02X%02X%02X%02X%02X%02X%02X\r\n",
             dim_power_report.serial_number[0], dim_power_report.serial_number[1],  dim_power_report.serial_number[2],dim_power_report.serial_number[3], dim_power_report.serial_number[4], dim_power_report.serial_number[5],
             dim_power_report.serial_number[6], dim_power_report.serial_number[7]);
      printf("  * MaxKcmvpKeyCount : %d\r\n", dim_power_report.kcmvp_key_count);
      printf("  * MaxCertKeyCount  : %d\r\n", dim_power_report.cert_key_count);
      printf("  * MaxIoDataSize    : %d\r\n", dim_power_report.io_data_size);
      printf("  * FileSize         : %d\r\n", dim_power_report.info_file_size);
  }
  return OK;
}

int DIM::power_off()
{
  // Power OFF
  dim_cmd.stx = 0xa5;
  dim_cmd.dir = 0x05;
  dim_cmd.offset = 0;
  dim_cmd.len = 0x2;
  dim_cmd.msgid = 0x01 << 8 | 0x00; // 0x0001 lsb

  transfer((uint8_t*)&dim_cmd, nullptr, sizeof(dim_cmd));
  up_udelay(T_STALL);
  up_mdelay(1000);

  transfer(nullptr, (uint8_t*)&dim_power_report, sizeof(dim_power_report));

  // read result value
  DEVICE_DEBUG("EXPECT 0xa5 0x06 0x00 0x02 0x00 0x00: %x %x %x %x %d \n",
         dim_power_report.stx, dim_power_report.dir, dim_power_report.offset, dim_power_report.len,
         dim_power_report.rv);

  printf("Power Off\r\n");
  return OK;
}

int
DIM::probe()
{
  int ret = power_on();
  if (ret != OK) {
      DEVICE_DEBUG("DIM power on error (%i)", ret);
      return ret;
  }

  up_mdelay(1000);

  // uint16_t product_id = 0;
  // DEVICE_DEBUG("PRODUCT: %X", product_id);

  return PX4_OK;
}

void
DIM::start()
{
    int ret = probe();
    if (ret != OK) {
        DEVICE_DEBUG("DIM probe error (%i)", ret);
    }

    // start polling at the specified rate
    ScheduleOnInterval((1_s / DIM_DEFAULT_RATE), 10000);

    ret = power_off();
    if (ret != OK) {
        DEVICE_DEBUG("DIM power off error (%i)", ret);
    }
}

void
DIM::exit_and_cleanup()
{
    if (_drdy_gpio != 0) {
		// Disable data ready callback
		px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr);
	}

    int ret;
    ret = power_off();
    if (ret != OK) {
        DEVICE_DEBUG("DIM power off error (%i)", ret);
    }

	I2CSPIDriverBase::exit_and_cleanup();
}

void
DIM::RunImpl()
{
    measure();
}

int
DIM::measure()
{
    perf_begin(_sample_perf);
    // DEVICE_DEBUG("DIM measure");
    printf("DIM measure\r\n");
    perf_end(_sample_perf);

    return OK;
}

void
DIM::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
}
