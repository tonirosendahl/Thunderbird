/****************************************************************************
 *
 *   Copyright (c) 2018, 2014 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * omnibusf4sd internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* Typhoon pin configuration */

/* Intel aero gpio's
 * GPIO
 *
 * Port     Signal Name               CONN
 * PA4     POWER                     JP1-23,            - Must be held High to run w/o USB
 * PB4     TEMP_CONT                 J2-2,11,14,23      - Gyro Heater
 * PC0     VOLTAGE                   JP2-13,14          - 1.84 @16.66  1.67 @15.12 Scale 0.1105
 * PC1     KEY_AD                    JP1-31,32          - Low when Power button is depressed
 * PC2     SD_SW                     SD-9 SW            - Card Present
 * PC3     PCON_RADIO                JP1-29,30
 * PC13    S2                        U8-9 74HCT151
 * PC14    S1                        U8-10 74HCT151
 * PC15    S0                        U8-11 74HCT151
 */

//Typhoon leds

//PA6 = red
#define GPIO_LED_RED       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN6)
//PA7 = green
#define GPIO_LED_GREEN       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN7)
//PB0 = blue
#define GPIO_LED_BLUE       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)
/*
#define GPIO_LED_RED   GPIO_LED1
#define GPIO_LED_GREEN   GPIO_LED2
#define GPIO_LED_BLUE   GPIO_LED3*/

//Power on
#define GPIO_POWERLATCH (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN4)

//Motor selector pins that drive inputs of the SN54HC151 8-to-1 line data selector
#define GPIO_S0     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)
#define GPIO_S1     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN14)
#define GPIO_S2     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)

//Battery voltage input
#define GPIO_LED15       (GPIO_INPUT|GPIO_SPEED_50MHz|GPIO_PORTC|GPIO_PIN0) //pc0

//IMU heater
#define GPIO_HEATER     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN4)

//Turn off to power on the RX
#define GPIO_RX_ON      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN3)

//Uncomment and initialize the pin in init.c if the CGO3+ is driven by cgo3uorb driver
//#define GCO3PWM (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN2)

#define BOARD_OVERLOAD_LED     LED_RED

//Save parameters to MCU flash
#define FLASH_BASED_PARAMS

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 0) | (1 << 10) | (1 << 11)
#define ADC_BATTERY_VOLTAGE_CHANNEL  10
#define ADC_BATTERY_CURRENT_CHANNEL  11
#define ADC_RC_RSSI_CHANNEL          0

/* Define Battery 1 Voltage Divider and A per V
 */
#define BOARD_BATTERY1_V_DIV         (11.12f)
#define BOARD_BATTERY1_A_PER_V       (31.f)

/*pb1*/
//#define GPIO_LED4       (GPIO_INPUT|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN1) //STILL 4.5 AFTER TURNING THIS TO INPUT, WHAT AFTER THIS

/* USB OTG FS*/
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTC|GPIO_PIN5)

/* I2C buses */

//Bus #1 is onboard bus for MPU6050 and MS5611
#define PX4_I2C_BUS_ONBOARD	1
//Bus #3 is expansion bus for the HMC5883L magnetometer
#define PX4_I2C_BUS_EXPANSION	3

#define PX4_I2C_BUS_ONBOARD_HZ      400000
#define PX4_I2C_BUS_EXPANSION_HZ      400000

//MPU6050 driver needs its address definition
#define PX4_I2C_MPU6050_ADDR 0x68

#define BOARD_TAP_ESC_MODE 1

#define MEMORY_CONSTRAINED_SYSTEM

/* PWM */

#define DIRECT_PWM_OUTPUT_CHANNELS      8
#define DIRECT_INPUT_TIMER_CHANNELS  0
#define BOARD_HAS_PWM    DIRECT_PWM_OUTPUT_CHANNELS

//Landing gear PWM
#define GPIO_TIM3_CH4OUT        GPIO_TIM3_CH4OUT_1
#define GPIO_TIM3_CH4IN        GPIO_TIM3_CH4IN_1

//Gimbal tilt PWM
#define GPIO_TIM2_CH3OUT        GPIO_TIM2_CH3OUT_1
#define GPIO_TIM2_CH3IN        GPIO_TIM2_CH3IN_1

/* High-resolution timer */
#define HRT_TIMER                    4 // T4C1
#define HRT_TIMER_CHANNEL            1 // use capture/compare channel 1

#define RC_SERIAL_PORT               "/dev/ttyS0"


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE    5120

#define BOARD_HAS_ON_RESET 1

#define BOARD_ENABLE_CONSOLE_BUFFER
#define BOARD_CONSOLE_BUFFER_SIZE (1024*3)

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/



extern void stm32_spiinitialize(void);
void board_spi_reset(int ms);

extern void stm32_usbinitialize(void);
extern void board_peripheral_reset(int ms);

#include <drivers/boards/common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
