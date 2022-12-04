/****************************************************************************
 * boards/arm/a1x/pcduino-a10/src/a1x_leds.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm64_internal.h"
#include "pinephone.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_led_initialize
 *
 * Description:
 *   Configure LEDs.  LEDs are left in the OFF state.
 *
 ****************************************************************************/

void pinephone_led_initialize(void)
{
  a1x_pio_config(PIO_LED1);
  a1x_pio_config(PIO_LED2);
  a1x_pio_config(PIO_LED3);
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Select the "logical" ON state.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_on(int led)
{
  //// TODO
}
#endif

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Select the "logical" OFF state.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_off(int led)
{
  //// TODO
}
#endif

/****************************************************************************
 * Name:  board_userled_initialize, board_userled, and board_userled_all
 *
 * Description:
 *   These interfaces allow user control of the board LEDs.
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control both on-board
 *   LEDs up until the completion of boot.
 *   The it will continue to control LED2; LED1 is available for application
 *   use.
 *
 *   If CONFIG_ARCH_LEDS is not defined, then both LEDs are available for
 *   application use.
 *
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Initialization already performed in a64_led_initialize */

  return BOARD_NLEDS;
}

void board_userled(int led, bool ledon)
{
  switch (led)
    {
    case BOARD_LED1:
      a1x_pio_write(PIO_LED1, ledon);
      break;

    case BOARD_LED2:
      a1x_pio_write(PIO_LED2, ledon);
      break;

    case BOARD_LED3:
      a1x_pio_write(PIO_LED3, ledon);
      break;
    }
}

void board_userled_all(uint32_t ledset)
{
  board_userled(BOARD_LED1, (ledset & BOARD_LED1) != 0);
  board_userled(BOARD_LED2, (ledset & BOARD_LED2) != 0);
  board_userled(BOARD_LED3, (ledset & BOARD_LED3) != 0);
}
