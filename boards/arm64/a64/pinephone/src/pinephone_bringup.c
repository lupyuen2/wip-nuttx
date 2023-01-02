/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_bringup.c
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
#include <sys/types.h>
#include <syslog.h>

#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/fs.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#  include "pinephone_display.h"
#endif

#include "pinephone.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int pinephone_bringup(void)
{
  int ret;

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }

  /* Render the Test Pattern */

  pinephone_display_test_pattern();
#endif

  //
  void touch_panel_initialize(void);
  touch_panel_initialize();
  //

  UNUSED(ret);
  return OK;
}

// Testing Touch Panel
#include <nuttx/arch.h>
#include <debug.h>
#include "arm64_arch.h"

// Touch Panel Interrupt (CTP-INT) is at PH4
#define CTP_INT (PIO_EINT | PIO_PORT_PIOH | PIO_PIN4)
#define CTP_INT_PIN 4

// IRQ for Touch Panel Interrupt (PH)
#define PH_EINT 53

// Interrupt Handler for Touch Panel
static int touch_panel_interrupt(int irq, void *context, void *arg)
{
  // Print something
  up_putc('.');
  return OK;
}

// Register the Interrupt Handler for Touch Panel
void touch_panel_initialize(void)
{
  int ret;

  // Configure the Touch Panel Interrupt
  ret = a64_pio_config(CTP_INT);
  DEBUGASSERT(ret == 0);

  // Un-mask the interrupt by setting the corresponding bit in the PIO INT CTL register.
  int pin = CTP_INT_PIN;

  // PH_EINT_CTL_REG (Interrupt Control Register for PH4) at Offset 0x250
  #define PH_EINT_CTL_REG (0x1c20800 + 0x250)
  _info("v=0x%x, m=0x%x, a=0x%x\n", PIO_INT_CTL(pin), PIO_INT_CTL(pin), PH_EINT_CTL_REG);
  // Shows touch_panel_initialize: v=0x10, m=0x10, a=0x1c20a50

  // Enter Critical Section
  irqstate_t flags;
  flags = enter_critical_section();

  // Enable the Touch Panel Interrupt
  modreg32(
    PIO_INT_CTL(pin),
    PIO_INT_CTL(pin),
    PH_EINT_CTL_REG
  );

  // Leave Critical Section
  leave_critical_section(flags);

  // TODO: Disable all external PIO interrupts
  // putreg32(0, A1X_PIO_INT_CTL);

  // Attach the PIO interrupt handler
  if (irq_attach(PH_EINT, touch_panel_interrupt, NULL) < 0)
    {
      _err("irq_attach failed\n");
      return;
    }

  // And enable the PIO interrupt
  up_enable_irq(PH_EINT);
}
