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

#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>
#include <math.h>

#include "st16_telemetry.h"

#define frac(f) (f - (int)f)

struct uorb_subscription_data_s {

	uORB::SubscriptionData<battery_status_s> battery_status_sub{ORB_ID(battery_status)};
	uORB::SubscriptionData<sensor_combined_s> sensor_combined_sub{ORB_ID(sensor_combined)};
	uORB::SubscriptionData<vehicle_air_data_s> vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::SubscriptionData<vehicle_global_position_s> vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::SubscriptionData<vehicle_gps_position_s> vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
};
static struct uorb_subscription_data_s *subscription_data = nullptr;

uint8_t st24_common_crc8(uint8_t *ptr, uint8_t len)
{
  uint8_t i, crc ;
  crc = 0;

  while (len--) {
    for (i = 0x80; i != 0; i >>= 1) {
      if ((crc & 0x80) != 0) {
        crc <<= 1;
        crc ^= 0x07;

      } else {
        crc <<= 1;
      }

      if ((*ptr & i) != 0) {
        crc ^= 0x07;
      }
    }

    ptr++;
  }

  return (crc);
}

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
	int speed = 115200;
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


/**
 * Initializes the uORB subscriptions.
 */
bool uorb_init()
{
	subscription_data = new uorb_subscription_data_s();

	if (!subscription_data) {
		return false;
	}

	return true;
}

void uorb_deinit()
{
	if (subscription_data) {
		delete subscription_data;
		subscription_data = nullptr;
	}
}

void uorb_update_topics()
{
	subscription_data->battery_status_sub.update();
	subscription_data->sensor_combined_sub.update();
	subscription_data->vehicle_air_data_sub.update();
	subscription_data->vehicle_global_position_sub.update();
	subscription_data->vehicle_gps_position_sub.update();
	subscription_data->vehicle_status_sub.update();
}

int send_packet(int uart_fd)
{
	ST16_telemetry::telemPayload telemetryPayload;
	ST16_telemetry::telemData telemetryData;

	while (true)
	{
		uorb_update_topics();

		if (counter >= 65535) counter = 0;
		counter ++;

		const battery_status_s &bat = subscription_data->battery_status_sub.get();
		const vehicle_gps_position_s &gps = subscription_data->vehicle_gps_position_sub.get();
		const vehicle_global_position_s &gpos = subscription_data->vehicle_global_position_sub.get();
		//const vehicle_air_data_s &air_data = subscription_data->vehicle_air_data_sub.get();

		if (gps.fix_type > 1)
		{
			gps_status = 0b10000000; //GPS lock OK
		}
		else
		{
			gps_status = 0b00000000; //No GPS lock
		}

		telemetryPayload.length = 0x26;
		telemetryPayload.type = 0x02;
		telemetryPayload.t = counter;

		telemetryPayload.lat = gps.lat;
		telemetryPayload.lon = gps.lon;
		//telemetryPayload.alt = roundf(frac(air_data.baro_alt_meter) * 100.0f);
		telemetryPayload.alt = roundf(frac(gpos.alt) * 100.0f);
		telemetryPayload.vx = (int16_t)gpos.vel_n;
		telemetryPayload.vy = (int16_t)gpos.vel_e;
		telemetryPayload.vz = (int16_t)gpos.vel_d;
		telemetryPayload.nsat = gps_status + (uint8_t)(gps.satellites_used);
		telemetryPayload.voltage = (uint8_t)roundf((bat.voltage_v-5) * 10.0f);//    0x63; //hex
		telemetryPayload.current = 0;
		telemetryPayload.roll = 0;
		telemetryPayload.pitch = 0;
		telemetryPayload.yaw = 0;
		telemetryPayload.motorStatus = 0xFF; //OK
		telemetryPayload.gpsStatus = 0x61;
		telemetryPayload.obsStatus = 0x55;
		telemetryPayload.optionbytes = 0x0510;
		telemetryPayload.alarmbytes = 0x0000;

		telemetryData.header1 =  0x55; //Header 1
		telemetryData.header2 =  0x55; //Header 2
		telemetryData.payload = telemetryPayload;
		telemetryData.crc8 = st24_common_crc8((uint8_t*)&telemetryPayload, sizeof(telemetryPayload));

		int packet_len = sizeof(telemetryData);

		::write(uart_fd, (uint8_t*)&telemetryData, packet_len);
		px4_usleep(100000);

	}
	return 1;
}

int ST16_telemetry::print_status()
{
	PX4_INFO("Running");
	return 0;
}

int ST16_telemetry::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ST16_telemetry::task_spawn(int argc, char *argv[])
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

ST16_telemetry *ST16_telemetry::instantiate(int argc, char *argv[])
{

	ST16_telemetry *instance = new ST16_telemetry(false, false);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

ST16_telemetry::ST16_telemetry(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void ST16_telemetry::run()
{
	const char *device = nullptr;
	char _device[32];
	port_number = 0;
	counter = 0;
	device = "/dev/ttyS3";

	strncpy(_device, device, sizeof(_device));
	_device[sizeof(_device) - 1] = '\0';

	initialise_uart(_device, port_number);
	uorb_init();

	{
		PX4_INFO("Sent %d bytes", send_packet(port_number));
	}


	deinitialise_uart(port_number);
	uorb_deinit();
}

int ST16_telemetry::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module sends telemetry data to Yuneec ST16 transmitter

### Implementation
The module receives data from uORB topics and sends it to the SR24 module over serial.

### Examples

$ ST16_telemetry
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("st16_telemetry", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	return 0;
}

int st16_telemetry_main(int argc, char *argv[])
{
	return ST16_telemetry::main(argc, argv);
}
