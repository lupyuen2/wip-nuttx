/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_touch.c
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

/* Reference:
 * "NuttX RTOS for PinePhone: Touch Panel"
 * https://lupyuen.github.io/articles/touch2
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/gt9xx.h>
#include "arm64_arch.h"
#include "arm64_gic.h"
#include "a64_pio.h"
#include "pinephone_touch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

// Test Touch Panel Interrupt by Polling as GPIO Input.
// Touch Panel Interrupt (CTP-INT) is at PH4.
// Configure for GPIO Input
#define CTP_INT (PIO_INPUT | PIO_PORT_PIOH | PIO_PIN4)

// TODO
#define CTP_I2C_ADDR 0x5d  // Default I2C Address for Goodix GT917S

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

// TODO
static int pinephone_gt9xx_irq_attach(const struct gt9xx_board_s *state,
                                      xcpt_t isr,
                                      FAR void *arg);
static void pinephone_gt9xx_irq_enable(const struct gt9xx_board_s *state,
                                       bool enable);
static int pinephone_gt9xx_set_power(const struct gt9xx_board_s *state,
                                     bool on);

/****************************************************************************
 * Private Data
 ****************************************************************************/

// TODO
static const struct gt9xx_board_s g_pinephone_gt9xx =
{
  .irq_attach = pinephone_gt9xx_irq_attach,
  .irq_enable = pinephone_gt9xx_irq_enable,
  .set_power  = pinephone_gt9xx_set_power
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

// TODO
static int pinephone_gt9xx_irq_attach(const struct gt9xx_board_s *state,
                                      xcpt_t isr,
                                      FAR void *arg)
{
  iinfo("\n");

  // Attach the PIO Interrupt Handler
  if (irq_attach(A64_IRQ_PH_EINT, isr, arg) < 0)
    {
      _err("irq_attach failed\n");
      return ERROR;
    }

  // Set Interrupt Priority in Generic Interrupt Controller v2
  // TODO: Why 2?
  arm64_gic_irq_set_priority(A64_IRQ_PH_EINT, 2, IRQ_TYPE_EDGE);

  // Enable the PIO Interrupt
  up_enable_irq(A64_IRQ_PH_EINT);

  return OK;
}

// Enable or disable interrupts
static void pinephone_gt9xx_irq_enable(const struct gt9xx_board_s *state,
                                       bool enable)
{
  int ret;

  iinfo("enable=%d\n", enable);
  if (enable)
    {
      // Configure the Touch Panel Interrupt
      ret = a64_pio_config(CTP_INT);
      DEBUGASSERT(ret == 0);

      // Enable the Touch Panel Interrupt
      ret = a64_pio_irqenable(CTP_INT);
      DEBUGASSERT(ret == 0);
    }
  else
    {
      // Disable the Touch Panel Interrupt
      ret = a64_pio_irqdisable(CTP_INT);
      DEBUGASSERT(ret == 0);
    }
}

static int pinephone_gt9xx_set_power(const struct gt9xx_board_s *state,
                                     bool on)
{
  iinfo("on=%d\n", on);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

// TODO
int pinephone_touch_panel_register(
  struct i2c_master_s *i2c  // I2C Bus
)
{
  int ret;

  DEBUGASSERT(i2c != NULL);

  ret = gt9xx_register("/dev/input0", i2c, CTP_I2C_ADDR, &g_pinephone_gt9xx);
  if (ret < 0)
    {
      ierr("Register Touch Input GT9xx failed: %d\n", ret);
      return ret;
    }  

  return OK;
}
