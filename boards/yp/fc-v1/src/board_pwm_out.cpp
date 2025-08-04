/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
#ifndef MODULE_NAME
#define MODULE_NAME "fcs_pwm_out"
#endif

#include "board_pwm_out.h"

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <px4_platform_common/log.h>

using namespace pwm_out;

const char  DevfsPWMOut::_device[] = "/pwm";

 DevfsPWMOut:: DevfsPWMOut(int max_num_outputs)
{
	if (max_num_outputs > MAX_NUM_PWM) {
		PX4_WARN("number of outputs too large. Setting to %i", MAX_NUM_PWM);
		max_num_outputs = MAX_NUM_PWM;
	}

	for (int i = 0; i < MAX_NUM_PWM; ++i) {
		_pwm_fd[i] = -1;
	}

	_pwm_num = max_num_outputs;
}

 DevfsPWMOut::~ DevfsPWMOut()
{
	for (int i = 0; i < MAX_NUM_PWM; ++i) {
		if (_pwm_fd[i] != -1) {
			::close(_pwm_fd[i]);
		}
	}
}

int  DevfsPWMOut::init()
{
	int i;
	char path[128];

	for (i = 0; i < _pwm_num; ++i) {
		::sprintf(path, "%s/%d", _device, (i + 1));
		_pwm_fd[i] = ::open(path, O_WRONLY | O_CLOEXEC);

		if (_pwm_fd[i] == -1) {
			PX4_ERR("PWM: Failed to open duty_cycle.");
			return -errno;
		}
		else {
			::ioctl(_pwm_fd[i], PWM_PULSE_FREQ_SET, FREQUENCY_PWM);
			::ioctl(_pwm_fd[i], PWM_PULSE_START, 0);
		}
	}

	return 0;
}

int  DevfsPWMOut::send_output_pwm(const uint16_t *pwm, int num_outputs)
{
	int ret = 0;

	if (num_outputs > _pwm_num) {
		num_outputs = _pwm_num;
	}

	for (int i = 0; i < num_outputs; ++i) {
		int write_ret = ::ioctl(_pwm_fd[i], PWM_PULSE_DUTYTIME_SET, pwm[i]);
		if (write_ret == -1) {
			ret = -1;
		}
	}

	return ret;
}

#if false
int  DevfsPWMOut::pwm_write_sysfs(char *path, int value)
{
	int fd = ::open(path, O_WRONLY | O_CLOEXEC);
	int n;
	char data[16];

	if (fd == -1) {
		return -errno;
	}

	n = ::snprintf(data, sizeof(data), "%u", value);

	if (n > 0) {
		n = ::write(fd, data, n);	// This n is not used, but to avoid a compiler error.
	}

	::close(fd);

	return 0;
}
#endif

