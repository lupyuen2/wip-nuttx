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
#include <nuttx/kmalloc.h>
#include <sys/types.h>
#include <syslog.h>
#include <debug.h>
#include "a64_twi.h"
#include "pinephone.h"
#include "pinephone_pmic.h"

#ifdef CONFIG_I2C
#  include <nuttx/i2c/i2c_master.h>
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

#ifdef CONFIG_INPUT_GT9XX
#  include "pinephone_touch.h"
#endif

#ifdef CONFIG_MPU60X0_I2C
#  include <nuttx/sensors/mpu60x0.h>
#endif

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
  int ret = OK;

#if defined(CONFIG_I2C) && defined(CONFIG_A64_TWI0)
  const int i2c0_bus = 0;
  struct i2c_master_s *i2c0 = NULL;
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_A64_TWI1)
  const int i2c1_bus = 1;
  struct i2c_master_s *i2c1 = NULL;
#endif

#if defined(CONFIG_MPU60X0_I2C) && defined(CONFIG_A64_TWI1)
  struct mpu_config_s *mpu_config = NULL;
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

#if defined(CONFIG_I2C) && defined(CONFIG_A64_TWI0)
  /* Initialize TWI0 as I2C Bus 0 */

  i2c0 = a64_i2cbus_initialize(i2c0_bus);
  if (i2c0 == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", i2c0_bus);
    }
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_A64_TWI1)
  /* Initialize TWI1 as I2C Bus 1 */

  i2c1 = a64_i2cbus_initialize(i2c1_bus);
  if (i2c1 == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", i2c1_bus);
    }
#endif

#if defined(CONFIG_SYSTEM_I2CTOOL) && defined(CONFIG_A64_TWI1)
  /* Register I2C Driver for I2C Bus 1 at TWI1 */

  if (i2c1 != NULL)
    {
      ret = i2c_register(i2c1, i2c1_bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 i2c1_bus, ret);
        }
    }
#endif

#if defined(CONFIG_INPUT_GT9XX) && defined(CONFIG_A64_TWI0)
  /* Register Touch Input Driver for GT9XX Touch Panel at TWI0 */

  if (i2c0 != NULL)
    {
      /* Register Touch Input Driver at /dev/input0 */

      ret = pinephone_touch_panel_register("/dev/input0", i2c0);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to register Touch Input GT9xx: %d\n",
                 ret);
        }
    }
#endif

#if defined(CONFIG_MPU60X0_I2C) && defined(CONFIG_A64_TWI1)
  /* Register IMU Driver for MPU-60X0 Accelerometer at TWI1 */

  if (i2c1 != NULL)
    {
      /* Init PMIC */

      ret = pinephone_pmic_init();
      if (ret < 0)
        {
          syslog(LOG_ERR, "Init PMIC failed: %d\n", ret);
          return ret;
        }

      /* Wait 15 milliseconds for power supply and power-on init */

      up_mdelay(15);

      /* Register IMU Driver at /dev/imu0 */

      mpu_config = kmm_zalloc(sizeof(struct mpu_config_s));
      if (mpu_config == NULL)
        {
          syslog(LOG_ERR, "ERROR: Failed to allocate mpu60x0 driver\n");
        }
      else
        {
          mpu_config->i2c = i2c1;
          mpu_config->addr = 0x68;
          mpu60x0_register("/dev/imu0", mpu_config);
        }
    }
#endif

#ifdef CONFIG_USBHOST
  int a64_usbhost_initialize(void); // TODO
  ret = a64_usbhost_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Couldn't start usb %d\n", ret);
    }
#endif

  // Init PinePhone LTE Modem
  int pinephone_modem_init(void);
  ret = pinephone_modem_init();
  DEBUGASSERT(ret == OK);

  UNUSED(ret);
  return OK;
}

// Init PinePhone LTE Modem
int pinephone_modem_init(void)
{
  int ret;

  // Read PH9 to check LTE Modem Status
  #define STATUS (PIO_INPUT | PIO_PORT_PIOH | PIO_PIN9)
  _info("Configure STATUS (PH9) for Input\n");
  ret = a64_pio_config(STATUS);
  DEBUGASSERT(ret == OK);
  _info("Status=%d\n", a64_pio_read(STATUS));

  // Power on DCDC1
  int pinephone_pmic_usb_init(void);
  ret = pinephone_pmic_usb_init();
  DEBUGASSERT(ret == OK);
  _info("Status=%d\n", a64_pio_read(STATUS));

  // Wait 2000 ms
  _info("Wait 2000 ms\n");
  up_mdelay(2000);
  _info("Status=%d\n", a64_pio_read(STATUS));

  // Set PL7 to High to Power On LTE Modem (4G-PWR-BAT)

  #define P_OUTPUT (PIO_OUTPUT | PIO_PULL_NONE | PIO_DRIVE_MEDLOW | \
                   PIO_INT_NONE | PIO_OUTPUT_SET)
  #define PWR_BAT (P_OUTPUT | PIO_PORT_PIOL | PIO_PIN7)
  _info("Configure PWR_BAT (PL7) for Output\n");
  ret = a64_pio_config(PWR_BAT);
  DEBUGASSERT(ret >= 0);

  _info("Set PWR_BAT (PL7) to High\n");
  a64_pio_write(PWR_BAT, true);
  _info("Status=%d\n", a64_pio_read(STATUS));

  // Wait 2000 ms
  _info("Wait 2000 ms\n");
  up_mdelay(2000);
  _info("Status=%d\n", a64_pio_read(STATUS));

  // Set PC4 to High to Deassert LTE Modem Reset (BB-RESET / RESET_N)

  #define RESET_N (P_OUTPUT | PIO_PORT_PIOC | PIO_PIN4)
  _info("Configure RESET_N (PC4) for Output\n");
  ret = a64_pio_config(RESET_N);
  DEBUGASSERT(ret >= 0);

  _info("Set RESET_N (PC4) to High\n");
  a64_pio_write(RESET_N, true);
  _info("Status=%d\n", a64_pio_read(STATUS));

  // Set DTR (PB2) to Low to wake up modem

  #define DTR (P_OUTPUT | PIO_PORT_PIOB | PIO_PIN2)
  _info("Configure DTR (PB2) for Output\n");
  ret = a64_pio_config(DTR);
  DEBUGASSERT(ret >= 0);

  _info("Set DTR (PB2) to Low to wake up modem\n");
  a64_pio_write(DTR, false);
  _info("Status=%d\n", a64_pio_read(STATUS));

  // Set PB3 to Power On LTE Modem (BB-PWRKEY / PWRKEY).
  // PWRKEY should be pulled down at least 500 ms, then pulled up.

  #define PWRKEY (P_OUTPUT | PIO_PORT_PIOB | PIO_PIN3)
  _info("Configure PWRKEY (PB3) for Output\n");
  ret = a64_pio_config(PWRKEY);
  DEBUGASSERT(ret >= 0);

  _info("Set PWRKEY (PB3) to High\n");
  a64_pio_write(PWRKEY, true);
  _info("Status=%d\n", a64_pio_read(STATUS));

  // Wait 2000 ms for VBAT to be stable
  _info("Wait 2000 ms for VBAT to be stable\n");
  up_mdelay(2000);
  _info("Status=%d\n", a64_pio_read(STATUS));

  _info("Set PWRKEY (PB3) to Low\n");
  a64_pio_write(PWRKEY, false);
  _info("Status=%d\n", a64_pio_read(STATUS));

  _info("Wait 2000 ms\n");
  up_mdelay(2000);
  _info("Status=%d\n", a64_pio_read(STATUS));

  _info("Set PWRKEY (PB3) to High\n");
  a64_pio_write(PWRKEY, true);
  _info("Status=%d\n", a64_pio_read(STATUS));

  // Set PH8 to High to Enable LTE Modem and Disable Airplane Mode (BB-DISABLE / W_DISABLE#)

  #define W_DISABLE (P_OUTPUT | PIO_PORT_PIOH | PIO_PIN8)
  _info("Configure W_DISABLE (PH8) for Output\n");
  ret = a64_pio_config(W_DISABLE);
  DEBUGASSERT(ret >= 0);

  _info("Set W_DISABLE (PH8) to High\n");
  a64_pio_write(W_DISABLE, true);
  _info("Status=%d\n", a64_pio_read(STATUS));

  // Test CTS / RTS: Pull RTS (PD4 Output) to Low,
  // check whether CTS (PD5 Input) gets pulled to High

  // TODO: Read CTS (PD5)
  #define CTS (PIO_INPUT | PIO_PORT_PIOD | PIO_PIN5)
  ret = a64_pio_config(CTS);
  DEBUGASSERT(ret == OK);
  _info("CTS=%d\n", a64_pio_read(CTS));

  // TODO: Set RTS (PD4) to Low

  #define RTS (P_OUTPUT | PIO_PORT_PIOD | PIO_PIN4)
  _info("Configure RTS (PD4) for Output\n");
  ret = a64_pio_config(RTS);
  DEBUGASSERT(ret >= 0);

  _info("Set RTS (PD4) to Low\n");
  a64_pio_write(RTS, false);

  // TODO: Read CTS (PD5)
  _info("CTS=%d\n", a64_pio_read(CTS));
  _info("Status=%d\n", a64_pio_read(STATUS));

  // TODO: Read PL6 to handle Ring Indicator / [Unsolicited Result Code](https://embeddedfreak.wordpress.com/2008/08/19/handling-urc-unsolicited-result-code-in-hayes-at-command/)

  // TODO: Set PH7 to High or Low for Sleep State

  // Poll for Modem Status while switching on RGB LEDs

  /* Green LED on PD18 */
  #define GREEN_LED (P_OUTPUT | PIO_PORT_PIOD | PIO_PIN18)

  /* Red LED on PD19 */
  #define RED_LED (P_OUTPUT | PIO_PORT_PIOD | PIO_PIN19)

  /* Blue LED on PD20 */
  #define BLUE_LED (P_OUTPUT | PIO_PORT_PIOD | PIO_PIN20)

  // ret = a64_pio_config(GREEN_LED);
  // DEBUGASSERT(ret >= 0);
  // ret = a64_pio_config(RED_LED);
  // DEBUGASSERT(ret >= 0);
  // ret = a64_pio_config(BLUE_LED);
  // DEBUGASSERT(ret >= 0);

  // _info("Turn on Green LED");
  // a64_pio_write(GREEN_LED, true);
  // a64_pio_write(RED_LED, false);
  // a64_pio_write(BLUE_LED, false);
  // up_mdelay(2000);
  // _info("Status=%d\n", a64_pio_read(STATUS));

  // _info("Turn on Red LED");
  // a64_pio_write(GREEN_LED, false);
  // a64_pio_write(RED_LED, true);
  // a64_pio_write(BLUE_LED, false);
  // up_mdelay(2000);
  // _info("Status=%d\n", a64_pio_read(STATUS));

  // _info("Turn on Blue LED");
  // a64_pio_write(GREEN_LED, false);
  // a64_pio_write(RED_LED, false);
  // a64_pio_write(BLUE_LED, true);
  // up_mdelay(2000);
  // _info("Status=%d\n", a64_pio_read(STATUS));

  return OK;
}
