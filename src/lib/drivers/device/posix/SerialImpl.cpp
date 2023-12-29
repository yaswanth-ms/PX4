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

#include "SerialImpl.hpp"

#include <unistd.h>

#include <string.h> // strncpy
#include <px4_log.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <drivers/drv_hrt.h>

namespace device
{

SerialImpl::SerialImpl(const char *port, uint32_t baudrate)
{
	if (port) {
		strncpy(_port, port, sizeof(_port) - 1);
		_port[sizeof(_port) - 1] = '\0';

	} else {
		_port[0] = 0;
	}

 	if (baudrate) {
		_baudrate = baudrate;
	} else {
		// If baudrate is zero then choose a reasonable default
		_baudrate = 9600;
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
	if (_SBUSMode) {
		return _sbus.configure(_serial_fd, _baudrate);
	}

	return _standard.configure(_serial_fd, _baudrate);
}

bool SerialImpl::open()
{
	if (isOpen()) {
		return true;
	}

	// Open the serial port
	int serial_fd = ::open(_port, O_RDWR | O_NOCTTY);

	if (serial_fd < 0) {
		PX4_ERR("failed to open %s err: %d", _port, errno);
		return false;
	}

	_serial_fd = serial_fd;

	// Configure the serial port
	if (! configure()) {
		PX4_ERR("failed to configure %s err: %d", _port, errno);
		close();
		return false;
	}

	_open = true;

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

	int written = ::write(_serial_fd, buffer, buffer_size);
	::fsync(_serial_fd);

	if (written < 0) {
		PX4_ERR("%s write error %d", _port, written);

	}

	return written;
}

const char *SerialImpl::getPort() const
{
	return _port;
}

uint32_t SerialImpl::getBaudrate() const
{
	return _baudrate;
}

bool SerialImpl::setBaudrate(uint32_t baudrate)
{
	// check if already configured
	if (baudrate == _baudrate) {
		return true;
	}

	_baudrate = baudrate;

	// process baud rate change now if port is already open
	if ((_open) && (configure() != 0)) {
		// Configure failed! Close the port
		close();
		return false;
	}

	return true;
}

bool SerialImpl::getSBUSMode() const
{
	return _SBUSMode;
}

bool SerialImpl::setSBUSMode(bool enable)
{
	if (_open) {
		PX4_ERR("Cannot configure SBUS mode after port has already been opened");
		return false;
	}

	_SBUSMode = enable;
	_baudrate = SerialSBUSImpl::DEFAULT_BAUDRATE;

	return true;
}

} // namespace device