/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * Agam FloRange v3 internal definitions.
 *
 * Pin assignments below are taken from the AFV3 schematic netlist and
 * cross-checked against the MCXA156 pinout table; the alternate-function
 * number for each is noted where it is not GPIO.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "mcxa1xx_port.h"

/* LEDs
 *
 * Two discrete LEDs, both active low.
 */

#define GPIO_nLED_RED     PORT_CFG(MCXA1XX_PORT3,  0, PORT_PCR_MUX_GPIO)
#define GPIO_nLED_BLUE    PORT_CFG(MCXA1XX_PORT3, 14, PORT_PCR_MUX_GPIO)

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED            LED_RED
#define BOARD_ARMED_STATE_LED         LED_BLUE

/* CAN
 *
 * FlexCAN0 on P1_10/P1_11 (ALT11), with software control of the
 * transceiver silent mode and the bus termination.
 */

#define GPIO_CAN0_TX          PORT_CFG(MCXA1XX_PORT1, 10, PORT_PCR_MUX_ALT11)
#define GPIO_CAN0_RX          PORT_CFG(MCXA1XX_PORT1, 11, PORT_PCR_MUX_ALT11 | \
                                                          PORT_PCR_IBE)
#define GPIO_CAN1_SILENT_S0   PORT_CFG(MCXA1XX_PORT3, 27, PORT_PCR_MUX_GPIO)
#define GPIO_CAN1_TERMINATION PORT_CFG(MCXA1XX_PORT3, 30, PORT_PCR_MUX_GPIO)
#define GPIO_CAN_TERM         GPIO_CAN1_TERMINATION

/* SPI1 == LPSPI1: ICM-42688-P IMU and PAA3905 optical flow share the bus */

#define GPIO_SPI1_SCK         PORT_CFG(MCXA1XX_PORT2, 12, PORT_PCR_MUX_ALT2)
#define GPIO_SPI1_MISO        PORT_CFG(MCXA1XX_PORT2, 15, PORT_PCR_MUX_ALT2 | \
                                                          PORT_PCR_IBE)
#define GPIO_SPI1_MOSI        PORT_CFG(MCXA1XX_PORT3,  8, PORT_PCR_MUX_ALT2)

#define GPIO_SPI1_CS_ICM42688P  PORT_CFG(MCXA1XX_PORT2,  6, PORT_PCR_MUX_GPIO)
#define GPIO_SPI1_CS_PAA3905    PORT_CFG(MCXA1XX_PORT3, 11, PORT_PCR_MUX_GPIO)

#define GPIO_SPI1_DRDY_ICM42688P  PORT_CFG(MCXA1XX_PORT2, 3, PORT_PCR_MUX_GPIO | \
                                                             PORT_PCR_IBE)
#define GPIO_SPI1_MOTION_PAA3905  PORT_CFG(MCXA1XX_PORT2, 7, PORT_PCR_MUX_GPIO | \
                                                             PORT_PCR_IBE)

/* IMU FSYNC.
 *
 * On P0_16, which is UTICK_CAP2 (ALT5) as well as GPIO, so the line can be
 * used either as a plain input or as a timer capture. It is left as GPIO
 * here; switch the mux to ALT5 if capture timestamping is wanted.
 */

#define GPIO_SPI1_FSYNC_ICM42688P PORT_CFG(MCXA1XX_PORT0, 16, PORT_PCR_MUX_GPIO | \
                                                              PORT_PCR_IBE)

/* SPI2 == LPSPI0: Broadcom AFBR-S50 time-of-flight rangefinder */

#define GPIO_SPI2_SCK         PORT_CFG(MCXA1XX_PORT1,  1, PORT_PCR_MUX_ALT2)
#define GPIO_SPI2_MISO        PORT_CFG(MCXA1XX_PORT1,  2, PORT_PCR_MUX_ALT2 | \
                                                          PORT_PCR_IBE)
#define GPIO_SPI2_MOSI        PORT_CFG(MCXA1XX_PORT1,  0, PORT_PCR_MUX_ALT2)

#define BROADCOM_AFBR_S50_S2PI_SPI_BUS 0    /* LPSPI0 */
#define BROADCOM_AFBR_S50_S2PI_CS   PORT_CFG(MCXA1XX_PORT0,  6, PORT_PCR_MUX_GPIO)
#define BROADCOM_AFBR_S50_S2PI_IRQ  PORT_CFG(MCXA1XX_PORT3, 31, PORT_PCR_MUX_GPIO | \
                                                                PORT_PCR_IBE)
#define BROADCOM_AFBR_S50_S2PI_CLK  GPIO_SPI2_SCK
#define BROADCOM_AFBR_S50_S2PI_MOSI GPIO_SPI2_MOSI
#define BROADCOM_AFBR_S50_S2PI_MISO GPIO_SPI2_MISO

/* Console / ROM ISP
 *
 * The serial header is on P0_2/P0_3 (LPUART0, ALT2). These are also the
 * pins the MCXA156 ROM ISP bootloader listens on, so the same header
 * serves both the NuttX console and ROM-ISP recovery; ISPMODE_N is on
 * P3_29.
 */

#define GPIO_LPUART0_RX       PORT_CFG(MCXA1XX_PORT0,  2, PORT_PCR_MUX_ALT2 | \
                                                          PORT_PCR_IBE)
#define GPIO_LPUART0_TX       PORT_CFG(MCXA1XX_PORT0,  3, PORT_PCR_MUX_ALT2)

/* Boot configuration: ISPMODE_N, driven by the bootloader switch */

#define GPIO_BOOT_CONFIG      PORT_CFG(MCXA1XX_PORT3, 29, PORT_PCR_MUX_GPIO | \
                                                          PORT_PCR_IBE | \
                                                          PORT_PCR_PE | \
                                                          PORT_PCR_PULLUP)

#define FLASH_BASED_PARAMS

/* High-resolution timer */

#define HRT_TIMER             1
#define HRT_TIMER_CHANNEL     1

__BEGIN_DECLS

#define BOARD_HAS_N_S_RGB_LED  0
#define BOARD_MAX_LEDS         2

#ifndef __ASSEMBLY__

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
