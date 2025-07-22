/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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

#include <SerialImpl.hpp>
#include <string.h> // strncpy
#include <px4_log.h>
//#include <drivers/device/qurt/uart.h>
#include <fcntl.h>
#include <drivers/drv_hrt.h>
#include <poll.h>

namespace device
{

SerialImpl::SerialImpl(const char *port, uint32_t baudrate, ByteSize bytesize, Parity parity, StopBits stopbits,
		       FlowControl flowcontrol) :
	_baudrate(baudrate),
	_bytesize(bytesize),
	_parity(parity),
	_stopbits(stopbits),
	_flowcontrol(flowcontrol)
{
	if (validatePort(port)) {
		setPort(port);

	} else {
		_port[0] = 0;
	}
}

SerialImpl::~SerialImpl()
{
	if (isOpen()) {
		close();
	}
}

bool SerialImpl::configure()
{
	/* process baud rate */
	int speed = _baudrate;

	/* set baud rate */
	if (_open) {
		::ioctl(_serial_fd, FIOBAUDRATE, speed);
		::ioctl(_serial_fd, FIORBUFSET, R_BUFFER_SIZE);
		::ioctl(_serial_fd, FIOWBUFSET, W_BUFFER_SIZE);
		::ioctl(_serial_fd, FIOOPTIONS, OPT_RAW);
		::ioctl(_serial_fd, FIOFLUSH, 0);
	}

	return true;
}

bool SerialImpl::open()
{
	// There's no harm in calling open multiple times on the same port.
	// In fact, that's the only way to change the baudrate
	if (isOpen()) {
		return true;
	}

	if (!validatePort(_port)) {
		PX4_ERR("Invalid port %s", _port);
		return false;
	}

	int serial_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (serial_fd < 0) {
		PX4_ERR("failed to open %s ", _port);
		return false;
	}

	_serial_fd = serial_fd;

	// Configure the serial port
	if (! configure()) {
		PX4_ERR("failed to configure %s ", _port);
		return false;
	}

	_open = true;

	if (_single_wire_mode) {
		setSingleWireMode();
	}

	if (_swap_rx_tx_mode) {
		setSwapRxTxMode();
	}

	setInvertedMode(_inverted_mode);

	return _open;
}

bool SerialImpl::isOpen() const
{
	return _open;
}

bool SerialImpl::close()
{
	if (_serial_fd >= 0) {
		::close(_serial_fd);
	}

	_serial_fd = -1;
	_open = false;

	return true;
}

ssize_t SerialImpl::read(uint8_t *buffer, size_t buffer_size)
{
	if (!_open) {
		PX4_ERR("Cannot read from serial device until it has been opened");
		return -1;
	}

	// TODO: DONGHEE implement read function in vxworks
	int ret = ::read(_serial_fd, buffer, buffer_size);

	if (ret < 0) {
		PX4_DEBUG("%s read error %d", _port, ret);

	}

	return ret;
}

ssize_t SerialImpl::readAtLeast(uint8_t *buffer, size_t buffer_size, size_t character_count, uint32_t timeout_us)
{
	if (!_open) {
		PX4_ERR("Cannot readAtLeast from serial device until it has been opened");
		return -1;
	}

	if (buffer_size < character_count) {
		PX4_ERR("%s: Buffer not big enough to hold desired amount of read data", __FUNCTION__);
		return -1;
	}

	const hrt_abstime start_time_us = hrt_absolute_time();
	int total_bytes_read = 0;

	while ((total_bytes_read < (int) character_count) && (hrt_elapsed_time(&start_time_us) < timeout_us)) {
		// Poll for incoming UART data.
		pollfd fds[1];
		fds[0].fd = _serial_fd;
		fds[0].events = POLLIN;

		hrt_abstime remaining_time = timeout_us - hrt_elapsed_time(&start_time_us);

		if (remaining_time <= 0) { break; }

		int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), remaining_time);

		if (ret > 0) {
			if (fds[0].revents & POLLIN) {
				const unsigned sleeptime = character_count * 1000000 / (_baudrate / 10);
				px4_usleep(sleeptime);

				ret = read(&buffer[total_bytes_read], buffer_size - total_bytes_read);

				if (ret > 0) {
					total_bytes_read += ret;
				}

			} else {
				PX4_ERR("Got a poll error");
				return -1;
			}
		}
	}

	return total_bytes_read;
}

ssize_t SerialImpl::write(const void *buffer, size_t buffer_size)
{
	if (!_open) {
		PX4_ERR("Cannot write to serial device until it has been opened");
		return -1;
	}

	// TODO: DONGHEE implement write function in vxworks
	int written = ::write(_serial_fd, buffer, buffer_size);
	::fsync(_serial_fd);

	if (written < 0) {
		PX4_ERR("%s write error %d", _port, written);

	}

	return written;
}

void SerialImpl::flush()
{
	// TODO: Flush not implemented yet on Qurt
}

const char *SerialImpl::getPort() const
{
	return _port;
}

bool SerialImpl::validatePort(const char *port)
{
	// TODO: DONGHEE implement validatePort function in vxworks
	bool ret = false;

	if (port != nullptr) {
		ret = (bool)(access(port, F_OK/*R_OK | W_OK*/) == 0);
	}
	return ret;
	//return true;
}

bool SerialImpl::setPort(const char *port)
{
	if (_open) {
		PX4_ERR("Cannot set port after port has already been opened");
		return false;
	}

	if (validatePort(port)) {
		strncpy(_port, port, sizeof(_port) - 1);
		_port[sizeof(_port) - 1] = '\0';
		return true;
	}

	return false;
}

uint32_t SerialImpl::getBaudrate() const
{
	return _baudrate;
}

bool SerialImpl::setBaudrate(uint32_t baudrate)
{
	// check if already configured
	if ((baudrate == _baudrate) && _open) {
		return true;
	}

	_baudrate = baudrate;

	// process baud rate change now if port is already open
	if (_open) {
		::ioctl(_serial_fd, FIOBAUDRATE, _baudrate);
	}

	return true;
}

ByteSize SerialImpl::getBytesize() const
{
	return _bytesize;
}

bool SerialImpl::setBytesize(ByteSize bytesize)
{
	return bytesize == ByteSize::EightBits;
}

Parity SerialImpl::getParity() const
{
	return _parity;
}

bool SerialImpl::setParity(Parity parity)
{
	return parity == Parity::None;
}

StopBits SerialImpl::getStopbits() const
{
	return _stopbits;
}

bool SerialImpl::setStopbits(StopBits stopbits)
{
	return stopbits == StopBits::One;
}

FlowControl SerialImpl::getFlowcontrol() const
{
	return _flowcontrol;
}

bool SerialImpl::setFlowcontrol(FlowControl flowcontrol)
{
	return flowcontrol == FlowControl::Disabled;
}

bool SerialImpl::getSingleWireMode() const
{
	return _single_wire_mode;
}

bool SerialImpl::setSingleWireMode()
{
#if defined(TIOCSSINGLEWIRE)

	if (_open) {
		ioctl(_serial_fd, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED);
	}

	_single_wire_mode = true;
	return true;
#else
	return false;
#endif // TIOCSSINGLEWIRE
}

bool SerialImpl::getSwapRxTxMode() const
{
	return _swap_rx_tx_mode;
}

bool SerialImpl::setSwapRxTxMode()
{
#if defined(TIOCSSWAP)

	if (_open) {
		ioctl(_serial_fd, TIOCSSWAP, SER_SWAP_ENABLED);
	}

	_swap_rx_tx_mode = true;
	return true;
#else
	return false;
#endif // TIOCSSWAP
}

bool SerialImpl::getInvertedMode() const
{
	return _inverted_mode;
}

bool SerialImpl::setInvertedMode(bool enable)
{
#if defined(TIOCSINVERT)

	if (_open) {
		if (enable) {
			ioctl(_serial_fd, TIOCSINVERT, SER_INVERT_ENABLED_RX | SER_INVERT_ENABLED_TX);

		} else {
			ioctl(_serial_fd, TIOCSINVERT, 0);
		}
	}

	_inverted_mode = enable;
	return true;
#else
	return _inverted_mode == enable;
#endif // TIOCSINVERT
}

} // namespace device
