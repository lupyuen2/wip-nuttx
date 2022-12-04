/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_H
#define __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "a64_pio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* LEDs *********************************************************************/

/* The pcDuino v1 has four green LEDs; three can be controlled from software.
 * Two are tied to ground and, hence, illuminated by driving the output pins
 * to a high value:
 *
 *  1. LED1 SPI0_CLK  SPI0_CLK/UART5_RX/EINT23/PI11
 *  2. LED5 IPSOUT    From the PMU (not controllable by software)
 */

#define PIO_LED1 (PIO_OUTPUT | PIO_PULL_NONE | \
                  PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
                  PIO_OUTPUT_CLEAR | PIO_PORT_PIOB | PIO_PIN11)  //// TODO

/* And two are pull high and, hence, illuminated by grounding the output:
 *
 *   3. LED3 RX_LED    LCD1_D16/ATAD12/KP_IN6/SMC_DET/EINT16/CSI1_D16/PH16
 *   4. LED4 TX_LED    LCD1_D15/ATAD11/KP_IN5/SMC_VPPPP/EINT15/CSI1_D15/PH15
 */

#define PIO_LED3 (PIO_OUTPUT | PIO_PULL_NONE | \
                  PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
                  PIO_OUTPUT_SET | PIO_PORT_PIOH | PIO_PIN16)  //// TODO

#define PIO_LED4 (PIO_OUTPUT | PIO_PULL_NONE | \
                  PIO_DRIVE_MEDLOW | PIO_INT_NONE | \
                  PIO_OUTPUT_SET | PIO_PORT_PIOH | PIO_PIN15)  //// TODO

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int pinephone_bringup(void);
#endif

/****************************************************************************
 * Name: pinephone_led_initialize
 *
 * Description:
 *   Configure LEDs.  LEDs are left in the OFF state.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void pinephone_led_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM64_A64_PINEPHONE_SRC_PINEPHONE_H */
