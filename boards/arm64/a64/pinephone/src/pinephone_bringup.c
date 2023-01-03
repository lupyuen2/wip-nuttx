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
#ifdef CONFIG_I2C
#  include <nuttx/i2c/i2c_master.h>
#endif
#include <nuttx/kmalloc.h>
#ifdef CONFIG_MPU60X0_I2C
#  include <nuttx/sensors/mpu60x0.h>
#endif

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

#include "a64_twi.h"
#include "pinephone.h"
#include "pinephone_pmic.h"

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
#ifdef CONFIG_I2C
  int i2c_bus;
  struct i2c_master_s *i2c;
#ifdef CONFIG_MPU60X0_I2C
  struct mpu_config_s *mpu_config;
#endif
#endif

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

#if defined(CONFIG_I2C) && defined(CONFIG_A64_TWI1)
  i2c_bus = 1;
  i2c = a64_i2cbus_initialize(i2c_bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", i2c_bus);
    }
  else
    {
#if defined(CONFIG_SYSTEM_I2CTOOL)
      ret = i2c_register(i2c, i2c_bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 i2c_bus, ret);
        }
#endif

#ifdef CONFIG_MPU60X0_I2C
      /* Init PMIC */

      ret = pinephone_pmic_init();
      if (ret < 0)
        {
          syslog(LOG_ERR, "Init PMIC failed: %d\n", ret);
          return ret;
        }

      /* Wait 15 milliseconds for power supply and power-on init */

      up_mdelay(15);

      mpu_config = kmm_zalloc(sizeof(struct mpu_config_s));
      if (mpu_config == NULL)
        {
          syslog(LOG_ERR, "ERROR: Failed to allocate mpu60x0 driver\n");
        }
      else
        {
          mpu_config->i2c = i2c;
          mpu_config->addr = 0x68;
          mpu60x0_register("/dev/imu0", mpu_config);
        }
#endif
    }
#endif

  ////TODO: Begin
#if defined(CONFIG_I2C) && defined(CONFIG_A64_TWI0)
  i2c_bus = 0;
  i2c = a64_i2cbus_initialize(i2c_bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", i2c_bus);
    }
  else
    {
      void touch_panel_initialize(struct i2c_master_s *i2c);
      touch_panel_initialize(i2c);
    }
#endif
  ////TODO: End

  UNUSED(ret);
  return OK;
}

// Testing Touch Panel
#include <nuttx/arch.h>
#include <debug.h>
#include "arm64_arch.h"

////#define TEST_INTERRUPT
#ifdef TEST_INTERRUPT
// Test Touch Panel Interrupt
// Touch Panel Interrupt (CTP-INT) is at PH4
#define CTP_INT (PIO_EINT | PIO_PORT_PIOH | PIO_PIN4)
#define CTP_INT_PIN 4

// IRQ for Touch Panel Interrupt (PH)
#define PH_EINT 53

// Interrupt Handler for Touch Panel
static int touch_panel_interrupt(int irq, void *context, void *arg)
{
  // Poll the Touch Panel Interrupt as GPIO Input
  static bool prev_val = false;

  // Read the GPIO Input
  bool val = a64_pio_read(CTP_INT);

  // Print if value has changed
  if (val != prev_val) {
    if (val) { up_putc('+'); }
    else     { up_putc('-'); }
  }
  prev_val = val;
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
    PIO_INT_CTL(pin),  // Value
    PIO_INT_CTL(pin),  // Mask
    PH_EINT_CTL_REG    // Address
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

#else
// Test Touch Panel Interrupt by Polling as GPIO Input.
// Touch Panel Interrupt (CTP-INT) is at PH4.
// Configure for GPIO Input
#define CTP_INT (PIO_INPUT | PIO_PORT_PIOH | PIO_PIN4)

static void touch_panel_read(struct i2c_master_s *i2c);

// Poll for Touch Panel Interrupt (PH4) by reading as GPIO Input
void touch_panel_initialize(struct i2c_master_s *i2c)
{

  // Configure the Touch Panel Interrupt for GPIO Input
  int ret = a64_pio_config(CTP_INT);
  DEBUGASSERT(ret == 0);

  // Poll the Touch Panel Interrupt as GPIO Input
  bool prev_val = false;
  for (int i = 0; i < 500; i++) {  // Poll for 5 seconds
    // Read the GPIO Input
    bool val = a64_pio_read(CTP_INT);

    // If value has changed...
    if (val != prev_val) {

      // Print the value
      if (val) { up_putc('+'); }
      else     { up_putc('-'); }
      prev_val = val;

      // If value is High...
      if (val) {

        // Read the Touch Panel over I2C
        touch_panel_read(i2c);
      }
    }

    // Wait a while
    up_mdelay(10);
  }
}

// ReadOnly registers (device and coordinates info)
#define GOODIX_REG_ID 0x8140
// Firmware version (LSB 2 bytes)
#define GOODIX_REG_FW_VER 0x8144

// Current output X resolution (LSB 2 bytes)
#define GOODIX_READ_X_RES 0x8146
// Current output Y resolution (LSB 2 bytes)
#define GOODIX_READ_Y_RES 0x8148
// Module vendor ID
#define GOODIX_READ_VENDOR_ID 0x814A

#define GOODIX_READ_COORD_ADDR 0x814E

#define GOODIX_POINT1_X_ADDR 0x8150
#define GOODIX_POINT1_Y_ADDR 0x8152

// Read Touch Panel over I2C
static void touch_panel_read(struct i2c_master_s *i2c)
{
  uint32_t freq = 400000;  // 400 kHz
  uint16_t addr = 0x5d;  // Default I2C Address for Goodix GT917S
  uint16_t reg = GOODIX_REG_ID;  // Read Product ID
  uint8_t regbuf[2] = { reg >> 8, reg & 0xff };  // Flip the bytes

  // Erase the receive buffer
  uint8_t buf[4];
  ssize_t buflen = sizeof(buf);
  memset(buf, 0xff, sizeof(buf));

  // Compose the I2C Messages
  struct i2c_msg_s msgv[2] =
  {
    {
      .frequency = freq,
      .addr      = addr,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = sizeof(regbuf)
    },
    {
      .frequency = freq,
      .addr      = addr,
      .flags     = I2C_M_READ,
      .buffer    = buf,
      .length    = buflen
    }
  };

  // Execute the I2C Transfer
  int ret = I2C_TRANSFER(i2c, msgv, 2);
  if (ret < 0) { _err("I2C Error: %d\n", ret); return; }

  // Dump the receive buffer
  infodumpbuffer("buf", buf, buflen);
  // Shows "39 31 37 53" or "917S"
}
#endif
