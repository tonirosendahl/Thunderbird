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

extern "C" __EXPORT int st16_telemetry_main(int argc, char *argv[]);

int port_number;
int counter;
uint8_t gps_status;

/**
 *  Opens a device for use as UART.
 *  @param device UNIX path of UART device
 *  @param uart_fd file-descriptor of UART device
 *  @return 0 on success, -1 on error
 */
int initialise_uart(const char *const device, int &uart_fd);

/**
 *  Closes a device previously opened with initialise_uart().
 *  @param uart_fd file-descriptor of UART device as provided by initialise_uart()
 *  @return 0 on success, -1 on error
 */
int deinitialise_uart(int &uart_fd);

/**
 *  Sends a packet to UART
 *  @param uart_fd file-descriptor of UART device
 */
int send_packet(int uart_fd);



class ST16_telemetry : public ModuleBase<ST16_telemetry>, public ModuleParams
{
public:
	ST16_telemetry(int example_param, bool example_flag);

	virtual ~ST16_telemetry() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static ST16_telemetry *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	//Alignment/padding
	#pragma pack(push, 1)

	typedef struct {
		uint8_t length;       ///< length includes type, data, and crc = sizeof(type)+sizeof(data[payload_len])+sizeof(crc8)
		uint8_t type;       ///< from enum ST24_PACKET_TYPE
		uint16_t t;     ///< packet counter or clock
		int32_t lat;      ///< latitude (degrees)  +/- 90 deg
		int32_t lon;      ///< longitude (degrees)  +/- 180 deg
		int32_t alt;      ///< 0.01m resolution, altitude (meters)
		int16_t vx, vy, vz;     ///< velocity 0.01m res, +/-320.00 North-East- Down

		uint8_t nsat;     ///< Sum of GPS status and number of satellites.
		//GPS status is bit 8 of the byte. Number of satellites is defined using the first 5 bits.
		//0b10011111 = GPS Ready, 31 satellites.
  		//0b00011111 = GPS Acguiring, 31 satellites.

		uint8_t voltage;    ///< 25.4V  voltage = 5 + 255*0.1 = 30.5V, min=5V
		uint8_t current;    ///< 0.5A resolution
		int16_t roll, pitch, yaw; ///< 0.01 degree resolution
		uint8_t motorStatus;    ///< 1 bit per motor for status 1=good, 0= fail

		uint8_t gpsStatus;    ///< gps and obs status
		/* Example: 0x61
		* 0x[X]Y
		* 0001 yyyy = gps disabled, obs rdy
		* 0010 yyyy = gps acquiring, obs disabled
		* 0100 yyyy = gps disabled, obs disabled
		* 1000 yyyy = gps disabled, obs disabled
		*
		* 0xX[Y]
		* xxxx 0001 = none
		* xxxx 0010 = none
		* xxxx 0100 = none
		* xxxx 1000 = none
		*/

		uint8_t obsStatus; ///< obs_avoidance | unknown
		/* Example: 0x55
		* 0x[X]Y
		* 0001 yyyy = obs avoidance fail
		* 0010 yyyy = obs avoidance fail
		* 0100 yyyy = obs avoidance disabled
		* 1000 yyyy = obs avoidance fail
		*
		* 0xX[Y]
		* xxxx 0001 = none
		* xxxx 0010 = none
		* xxxx 0100 = none
		* xxxx 1000 = none
		*/

		uint16_t optionbytes; ///< drone model | flight mode
		/* Example: 0x0510
		*
		* 0x00XY (Bytes 0-1)
		*
		* 0x[X]Y
		* 0001 yyyy = READY
		* 0010 yyyy = IPS
		* 0100 yyyy = "N/A"
		* 1000 yyyy = THR
		*
		* 0x1[Y]
		* 0001 1000 = WATCH
		* 0001 0100 = RATE
		* 0001 0010 = MAG CALI
		* 0001 0001 = NO RC
		*
		*0xXY00 (Bytes 2-3)
		*
		* 0x[X]Y
		* 0001 yyyy = nothing
		* 0010 yyyy = nothing
		* 0100 yyyy = nothing
		* 1000 yyyy = nothing
		*
		* 0xX[Y]
		* //xxxx 0001 = BATT RED, drone model: hex
		* //xxxx 0010 = BATT FULL, drone model: quad
		* //xxxx 0100 = BATT RED, drone model: hex
		* //xxxx 1000 = BATT RED, drone model: hex
		* 0x03xx = drone model: quad, batt red
		* 0x05xx = drone model: hex, batt ok <---- Typhoon H (captured)
		* 0x06xx = drone model: hex, batt red
		* 0x07xx = drone model: hex, batt ok <---- ?
		*/

		uint16_t alarmbytes; ///< unknown | alarms
		/* Example: 0x5820
		*
		* 0x00XY (Bytes 0-1)
		*
		* 0x[X]Y
		* 0001 yyyy = imu temperature warning
		* 0010 yyyy = compass warning (captured, uncalibrated compass attached)
		* 0100 yyyy = ?
		* 1000 yyyy = no fly zone warning
		*
		* 0xX[Y]
		* xxxx 0001 = battery voltage low
		* xxxx 0010 = battery voltage critically low
		* xxxx 0100 = imu temperature warning (again?)
		* xxxx 1000 = ?
		*
		* 0xXY00 (Bytes 2-3)
		*
		* 0001 yyyy = ?
		* 0010 yyyy = ?
		* 0100 yyyy = ?
		* 1000 yyyy = ?
		*
		* xxxx 0001 = ?
		* xxxx 0010 = ?
		* xxxx 0100 = ?
		* xxxx 1000 = ?
		*/
	} telemPayload;

	typedef struct {
		uint8_t header1;      ///< 0x55 for a valid packet
		uint8_t header2;      ///< 0x55 for a valid packet
		telemPayload payload;
		uint8_t crc8;       ///< crc8 checksum, calculated by st24_common_crc8 and including fields length, type and st24_data
	} telemData;

	#pragma pack(pop)

	/**
	 * CRC8 implementation for ST24 protocol
	 *
	 * @param prt Pointer to the data to CRC
	 * @param len number of bytes to accumulate in the checksum
	 * @return the checksum of these bytes over len
	 */
	uint8_t st24_common_crc8(uint8_t *ptr, uint8_t len);

};
