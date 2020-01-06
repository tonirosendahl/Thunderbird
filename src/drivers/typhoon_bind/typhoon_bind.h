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

#pragma once

#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>

extern "C" __EXPORT int typhoon_bind_main(int argc, char *argv[]);


int bind_port_number;

/**
 *  Opens a device for use as UART.
 *  @param device UNIX path of UART device
 *  @param uart_fd file-descriptor of UART device
 *  @return 0 on success, -1 on error
 */
int bind_initialise_uart(const char *const device, int &uart_fd);

/**
 *  Closes a device previously opened with initialise_uart().
 *  @param uart_fd file-descriptor of UART device as provided by initialise_uart()
 *  @return 0 on success, -1 on error
 */
int bind_deinitialise_uart(int &uart_fd);

/**
 *  Sends a packet to UART
 *  @param uart_fd file-descriptor of UART device
 */
int bind_send_packet(int uart_fd);



class Typhoon_bind : public ModuleBase<Typhoon_bind>, public ModuleParams
{
public:
	Typhoon_bind(int example_param, bool example_flag);

	virtual ~Typhoon_bind() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Typhoon_bind *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;
};
