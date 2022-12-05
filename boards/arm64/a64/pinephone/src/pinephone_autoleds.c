/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_autoleds.c
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

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAYSIZE(x) (sizeof((x)) / sizeof((x)[0]))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* LED index */

static const uint32_t g_led_map[BOARD_LEDS] =
{
  LED1,
  LED2,
  LED3
};

static bool g_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Turn on selected led */

static void pinephone_led_on(led_typedef_enum led_num)
{
  _info("autoled=0x%x\n", led_num);////
  a64_pio_write(g_led_map[led_num], true);
}

/* Turn off selected led */

static void pinephone_led_off(led_typedef_enum led_num)
{
  _info("autoled=0x%x\n", led_num);////
  a64_pio_write(g_led_map[led_num], false);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  int i;
  int ret;

  /* Configure the LED GPIO for output. */

  for (i = 0; i < ARRAYSIZE(g_led_map); i++)
    {
      ret = a64_pio_config(g_led_map[i]);
      DEBUGASSERT(ret == OK);
    }
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  _info("led=0x%x\n", led);////
  switch (led)
    {
    default:
      break;

    case LED_HEAPALLOCATE:
      pinephone_led_on(BOARD_LED1);
      break;

    case LED_IRQSENABLED:
      pinephone_led_on(BOARD_LED2);
      break;

    case LED_STACKCREATED:
      pinephone_led_on(BOARD_LED3);
      g_initialized = true;
      break;

    case LED_INIRQ:
      pinephone_led_on(BOARD_LED1);
      pinephone_led_on(BOARD_LED2);
      break;

    case LED_SIGNAL:
      pinephone_led_on(BOARD_LED1);
      pinephone_led_on(BOARD_LED3);
      break;

    case LED_ASSERTION:
      pinephone_led_on(BOARD_LED2);
      pinephone_led_on(BOARD_LED3);
      break;

    case LED_PANIC:
      pinephone_led_on(BOARD_LED1);
      break;

    case LED_IDLE : /* IDLE */
      pinephone_led_on(BOARD_LED2);
    break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  _info("led=0x%x\n", led);////
  switch (led)
    {
    default:
      break;

    case LED_SIGNAL:
      pinephone_led_off(BOARD_LED1);
      pinephone_led_off(BOARD_LED3);
      break;

    case LED_INIRQ:
      pinephone_led_off(BOARD_LED1);
      pinephone_led_off(BOARD_LED2);
      break;

    case LED_ASSERTION:
      pinephone_led_off(BOARD_LED2);
      pinephone_led_off(BOARD_LED3);
      break;

    case LED_PANIC:
      pinephone_led_off(BOARD_LED1);
      break;

    case LED_IDLE : /* IDLE */
      pinephone_led_off(BOARD_LED2);
    break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
