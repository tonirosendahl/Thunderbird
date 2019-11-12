/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <unistd.h>
#include "typhoon_bind.h"

#ifndef B115200
#define B115200 115200
#endif

int initialise_uart(const char *const device, int &uart_fd)
{
	// open uart
	uart_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	int termios_state = -1;

	if (uart_fd < 0) {
		PX4_ERR("failed to open uart device!");
		return -1;
	}

	// set baud rate
	int speed = B115200;
	struct termios uart_config;
	tcgetattr(uart_fd, &uart_config);

	// clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	// set baud rate
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("failed to set baudrate for %s: %d\n", device, termios_state);
		close(uart_fd);
		return -1;
	}

	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("tcsetattr failed for %s\n", device);
		close(uart_fd);
		return -1;
	}

	return 0;
}

int deinitialise_uart(int &uart_fd)
{
	int ret = close(uart_fd);

	if (ret == 0) {
		uart_fd = -1;
	}

	return ret;
}

int send_packet(int uart_fd)
{
	char txbuf[] = {0x55,0x55,0x8,0x4,0x0,0x0,0x42,0x49,0x4E,0x44,0xB0};
	int packet_len = sizeof(txbuf);

	char *txPtr;
	txPtr = txbuf;
	int ret = 0;


	for (int i = 0; i < 3; i++)
	{
		ret +=	::write(uart_fd, txPtr, packet_len);
		px4_usleep(500000);
	}
	return ret;
}

int Typhoon_bind::print_status()
{
	PX4_INFO("Running");
	return 0;
}

int Typhoon_bind::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Typhoon_bind::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}


	return 0;
}

Typhoon_bind *Typhoon_bind::instantiate(int argc, char *argv[])
{

	Typhoon_bind *instance = new Typhoon_bind(false, false);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Typhoon_bind::Typhoon_bind(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void Typhoon_bind::run()
{
	const char *device = nullptr;
	char _device[32];
	port_number = 0;
	device = "/dev/ttyS3";

	strncpy(_device, device, sizeof(_device));
	_device[sizeof(_device) - 1] = '\0';

	initialise_uart(_device, port_number);

	//33 is amount of bytes to be sent. The bind command consists of 11 bytes, sent three times with a delay
	if (send_packet(port_number) == 33)
	{
		PX4_INFO("Bind command sent.");
	}
	else
	{
		 PX4_INFO("Error sending bind command.");
	}

	PX4_INFO("Please reboot the drone before flight.");

	deinitialise_uart(port_number);
}

int Typhoon_bind::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module puts the Yuneec SR24 receiver on the Typhoon H to bind mode.

### Implementation
This module sends the bind command (0x55,0x55,0x8,0x4,0x0,0x0,0x42,0x49,0x4E,0x44,0xB0) to /dev/ttyS3 @ 115 200 bps.

### Examples

$ typhoon_bind
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("typhoon_bind", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	return 0;
}

int typhoon_bind_main(int argc, char *argv[])
{
	return Typhoon_bind::main(argc, argv);
}
