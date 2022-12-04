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
  a1x_pio_config(PIO_LED3);
  a1x_pio_config(PIO_LED4);
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Select the "logical" ON state:
 *
 *   SYMBOL            Value Meaning                    LED state
 *                                                    LED1 LED3 LED4
 *   ----------------- ----- -----------------------  ---- ---- ------------
 *   LED_STARTED         0   NuttX has been started   ON   OFF  OFF
 *   LED_HEAPALLOCATE    1   Heap has been allocated  OFF  ON   OFF
 *   LED_IRQSENABLED     2   Interrupts enabled       ON   ON   OFF
 *   LED_STACKCREATED    2   Idle stack created       ON   ON   OFF
 *   LED_INIRQ           3   In an interrupt          N/C  N/C  Soft glow
 *   LED_SIGNAL          3   In a signal handler      N/C  N/C  Soft glow
 *   LED_ASSERTION       3   An assertion failed      N/C  N/C  Soft glow
 *   LED_PANIC           3   The system has crashed   N/C  N/C  2Hz Flashing
 *   LED_IDLE           ---  MCU is is sleep mode         Not used
 *
 *   LED1 is illuminated by driving the output pins to a high value
 *   LED3 and LED 4 are illuminated by taking the output to ground.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_on(int led)
{
  switch (led)
    {
    case 0:
      a1x_pio_write(PIO_LED1, true);
      a1x_pio_write(PIO_LED3, true);
      a1x_pio_write(PIO_LED4, true);
      break;

    case 1:
      a1x_pio_write(PIO_LED1, false);
      a1x_pio_write(PIO_LED3, false);
      a1x_pio_write(PIO_LED4, true);
      break;

    case 2:
      a1x_pio_write(PIO_LED1, false);
      a1x_pio_write(PIO_LED3, true);
      a1x_pio_write(PIO_LED4, true);
      break;

    case 3:
      a1x_pio_write(PIO_LED4, false);
      break;
    }
}
#endif

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Select the "logical" OFF state:
 *
 *   SYMBOL            Value Meaning                    LED state
 *                                                    LED1 LED3 LED4
 *   ----------------- ----- -----------------------  ---- ---- ------------
 *   LED_STARTED         0   NuttX has been started   ON   OFF  OFF
 *   LED_HEAPALLOCATE    1   Heap has been allocated  OFF  ON   OFF
 *   LED_IRQSENABLED     2   Interrupts enabled       ON   ON   OFF
 *   LED_STACKCREATED    2   Idle stack created       ON   ON   OFF
 *   LED_INIRQ           3   In an interrupt          N/C  N/C  Soft glow
 *   LED_SIGNAL          3   In a signal handler      N/C  N/C  Soft glow
 *   LED_ASSERTION       3   An assertion failed      N/C  N/C  Soft glow
 *   LED_PANIC           3   The system has crashed   N/C  N/C  2Hz Flashing
 *   LED_IDLE           ---  MCU is is sleep mode         Not used
 *
 *   LED1 is illuminated by driving the output pins to a high value
 *   LED3 and LED 4 are illuminated by taking the output to ground.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_off(int led)
{
  switch (led)
    {
    case 0:
    case 1:
    case 2:
      break;

    case 3:
      a1x_pio_write(PIO_LED4, true);
      break;
    }
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

    case BOARD_LED3:
      a1x_pio_write(PIO_LED3, !ledon);
      break;

#ifndef CONFIG_ARCH_LEDS
    case BOARD_LED4:
      a1x_pio_write(PIO_LED4, !ledon);
      break;
#endif
    }
}

void board_userled_all(uint32_t ledset)
{
  board_userled(BOARD_LED1, (ledset & BOARD_LED1) != 0);
  board_userled(BOARD_LED3, (ledset & BOARD_LED3) != 0);
#ifndef CONFIG_ARCH_LEDS
  board_userled(BOARD_LED4, (ledset & BOARD_LED4) != 0);
#endif
}
