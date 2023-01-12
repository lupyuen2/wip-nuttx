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
#ifdef CONFIG_INPUT_GT9XX
#  include <nuttx/input/gt9xx.h>
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

// TODO
static int pinephone_gt9xx_irq_attach(const struct gt9xx_board_s *state,
                                      xcpt_t isr,
                                      FAR void *arg);
static void pinephone_gt9xx_irq_enable(const struct gt9xx_board_s *state,
                                       bool enable);
static int pinephone_gt9xx_set_power(const struct gt9xx_board_s *state,
                                     bool on);

// TODO
#ifdef CONFIG_INPUT_GT9XX
static const struct gt9xx_board_s g_pinephone_gt9xx =
{
  .irq_attach = pinephone_gt9xx_irq_attach,
  .irq_enable = pinephone_gt9xx_irq_enable,
  .set_power  = pinephone_gt9xx_set_power
};
#endif /* CONFIG_INPUT_GT9XX */

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
          syslog(LOG_ERR, "ERROR: Init PMIC failed: %d\n", ret);
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
      // int touch_panel_initialize(struct i2c_master_s *i2c);
      // int ret2 = touch_panel_initialize(i2c);
      // DEBUGASSERT(ret2 == 0);
    }
#endif

#ifdef CONFIG_INPUT_GT9XX
  // TODO
  #define CTP_I2C_ADDR 0x5d  // Default I2C Address for Goodix GT917S
  DEBUGASSERT(i2c != NULL);
  ret = gt9xx_register("/dev/input0", i2c, CTP_I2C_ADDR, &g_pinephone_gt9xx);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Register Touch Input GT9xx failed: %d\n", ret);
      return ret;
    }
#endif
  ////TODO: End

  UNUSED(ret);
  return OK;
}

// Testing Touch Panel
#include <nuttx/config.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include "arm64_arch.h"
#include "arm64_gic.h"

#define CTP_FREQ 400000  // I2C Frequency: 400 kHz
#define CTP_I2C_ADDR 0x5d  // Default I2C Address for Goodix GT917S

static void touch_panel_read(struct i2c_master_s *i2c);
static int touch_panel_i2c_read(
  struct i2c_master_s *i2c,  // I2C Bus
  uint16_t reg,  // I2C Register
  uint8_t *buf,  // Receive Buffer
  size_t buflen  // Receive Buffer Size
);
static int touch_panel_set_status(
  struct i2c_master_s *i2c,  // I2C Bus
  uint8_t status  // Status value to be set
);

#define TEST_INTERRUPT
#ifdef TEST_INTERRUPT
// Test Touch Panel Interrupt
// Touch Panel Interrupt (CTP-INT) is at PH4
#define CTP_INT (PIO_EINT | PIO_PORT_PIOH | PIO_PIN4)
#define CTP_INT_PIN 4

#define GT9XX_NPOLLWAITERS 10  // Number of poll waiters supported

struct gt9xx_dev_s
{
  /* I2C bus and address for device */

  struct i2c_master_s *i2c;
  uint8_t addr;

  /* Configuration for device */

  mutex_t devlock;
  uint8_t cref;
  bool int_pending;

  /* Poll Waiters for device */

  struct pollfd *fds[GT9XX_NPOLLWAITERS];
};

// Interrupt Handler for Touch Panel
static int gt9xx_isr_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct gt9xx_dev_s *priv = (FAR struct gt9xx_dev_s *)arg;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  // Set the Interrupt Pending Flag
  flags = enter_critical_section();
  priv->int_pending = true;
  leave_critical_section(flags);

  // Notify the Poll Waiters
  poll_notify(priv->fds, GT9XX_NPOLLWAITERS, POLLIN);
  return 0;
}

// Register the Interrupt Handler for Touch Panel
int touch_panel_initialize(struct i2c_master_s *i2c_dev)
{
  uint8_t i2c_devaddr = CTP_I2C_ADDR;
  int ret;

  // Allocate device private structure
  struct gt9xx_dev_s *priv;
  priv = kmm_zalloc(sizeof(struct gt9xx_dev_s));
  if (!priv)
    {
      _err("Memory cannot be allocated for gt9xx sensor\n");  // TODO
      return -ENOMEM;
    }

  // Setup device structure
  priv->addr = i2c_devaddr;
  priv->i2c = i2c_dev;
  nxmutex_init(&priv->devlock);

  // Attach the PIO Interrupt Handler
  if (irq_attach(A64_IRQ_PH_EINT, gt9xx_isr_handler, priv) < 0)
    {
      _err("irq_attach failed\n");
      return ERROR;
    }

  // Set Interrupt Priority in Generic Interrupt Controller v2
  arm64_gic_irq_set_priority(A64_IRQ_PH_EINT, 2, IRQ_TYPE_EDGE);

  // Enable the PIO Interrupt
  up_enable_irq(A64_IRQ_PH_EINT);

  // Configure the Touch Panel Interrupt
  ret = a64_pio_config(CTP_INT);
  DEBUGASSERT(ret == 0);

  // Enable the Touch Panel Interrupt
  ret = a64_pio_irqenable(CTP_INT);
  DEBUGASSERT(ret == 0);

  // Poll for Touch Panel Interrupt
  // TODO: Move this
  for (int i = 0; i < 6000; i++) {  // Poll for 60 seconds

    // If Touch Panel Interrupt has been triggered...
    if (priv->int_pending) {

      // Read the Touch Panel over I2C
      touch_panel_read(i2c_dev);

      // Reset the Interrupt Pending Flag
      priv->int_pending = false;
    }

    // Wait a while
    up_mdelay(10);  // 10 milliseconds
  }

  // TODO: Register Touch Input Driver
  // ret = register_driver(devpath, &g_gt9xx_fileops, 0666, priv);
  // if (ret < 0)
  //   {
  //     nxmutex_destroy(&priv->devlock);
  //     kmm_free(priv);
  //     gt9xx_dbg("Error occurred during the driver registering\n");
  //     return ret;
  //   }

  // _info("Registered with %d\n", ret);  // TODO

  // TODO: Prepare interrupt line and handler
  // priv->board->irq_attach(priv->board, gt9xx_isr_handler, priv);
  // priv->board->irq_enable(priv->board, false);

  return OK;
}

#else
// Test Touch Panel Interrupt by Polling as GPIO Input.
// Touch Panel Interrupt (CTP-INT) is at PH4.
// Configure for GPIO Input
#define CTP_INT (PIO_INPUT | PIO_PORT_PIOH | PIO_PIN4)

// Poll for Touch Panel Interrupt (PH4) by reading as GPIO Input
int touch_panel_initialize(struct i2c_master_s *i2c)
{

  // Configure the Touch Panel Interrupt for GPIO Input
  int ret = a64_pio_config(CTP_INT);
  DEBUGASSERT(ret == 0);

  // Poll the Touch Panel Interrupt as GPIO Input
  bool prev_val = false;
  for (int i = 0; i < 6000; i++) {  // Poll for 60 seconds

    // Read the GPIO Input
    bool val = a64_pio_read(CTP_INT);

    // If value has changed...
    if (val != prev_val) {

      // Print the value
      if (val) { up_putc('+'); }
      else     { up_putc('-'); }
      prev_val = val;

      // If we have just transitioned from Low to High...
      if (val) {

        // Read the Touch Panel over I2C
        touch_panel_read(i2c);
      }
    }

    // Wait a while
    up_mdelay(10);  // 10 milliseconds
  }
}

#endif  // !TEST_INTERRUPT

// ReadOnly registers (device and coordinates info)
// Product ID (LSB 4 bytes)
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

// Read Touch Panel over I2C
static void touch_panel_read(struct i2c_master_s *i2c)
{
  // Read the Product ID
  // uint8_t id[4];
  // touch_panel_i2c_read(i2c, GOODIX_REG_ID, id, sizeof(id));
  // Shows "39 31 37 53" or "917S"

  // Read the Touch Panel Status
  uint8_t status[1];
  touch_panel_i2c_read(i2c, GOODIX_READ_COORD_ADDR, status, sizeof(status));
  // Shows "81"

  const uint8_t status_code    = status[0] & 0x80;  // Set to 0x80
  const uint8_t touched_points = status[0] & 0x0f;  // Set to 0x01

  if (status_code != 0 &&  // If Touch Panel Status is OK and...
      touched_points >= 1) {  // Touched Points is 1 or more

    // Dump the receive buffer
    infodumpbuffer("buf", status, sizeof(status));

    // Read the First Touch Coordinates
    uint8_t touch[6];
    touch_panel_i2c_read(i2c, GOODIX_POINT1_X_ADDR, touch, sizeof(touch));
    // Shows "92 02 59 05 1b 00"

    // Dump the receive buffer
    infodumpbuffer("buf", touch, sizeof(touch));

    const uint16_t x = touch[0] + (touch[1] << 8);
    const uint16_t y = touch[2] + (touch[3] << 8);
    _info("touch x=%d, y=%d\n", x, y);
    // Shows "touch x=658, y=1369"
  }

  // Set the Touch Panel Status to 0
  touch_panel_set_status(i2c, 0);
}

static int touch_panel_i2c_read(
  struct i2c_master_s *i2c,  // I2C Bus
  uint16_t reg,  // I2C Register
  uint8_t *buf,  // Receive Buffer
  size_t buflen  // Receive Buffer Size
) {
  uint32_t freq = CTP_FREQ;  // 400 kHz
  uint16_t addr = CTP_I2C_ADDR;  // Default I2C Address for Goodix GT917S
  uint8_t regbuf[2] = {
    reg >> 8,   // Swap the bytes
    reg & 0xff  // Swap the bytes
  };

  // Erase the receive buffer
  memset(buf, 0xff, buflen);

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
  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);
  int ret = I2C_TRANSFER(i2c, msgv, msgv_len);
  if (ret < 0) { _err("I2C Error: %d\n", ret); return ret; }

  // Dump the receive buffer
  infodumpbuffer("buf", buf, buflen);
  return OK;
}

static int touch_panel_set_status(
  struct i2c_master_s *i2c,  // I2C Bus
  uint8_t status  // Status value to be set
) {
  uint16_t reg = GOODIX_READ_COORD_ADDR;  // I2C Register
  uint32_t freq = CTP_FREQ;  // 400 kHz
  uint16_t addr = CTP_I2C_ADDR;  // Default I2C Address for Goodix GT917S
  uint8_t buf[3] = {
    reg >> 8,    // Swap the bytes
    reg & 0xff,  // Swap the bytes
    status
  };

  // Compose the I2C Message
  struct i2c_msg_s msgv[1] =
  {
    {
      .frequency = freq,
      .addr      = addr,
      .flags     = 0,
      .buffer    = buf,
      .length    = sizeof(buf)
    }
  };

  // Execute the I2C Transfer
  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);
  int ret = I2C_TRANSFER(i2c, msgv, msgv_len);
  if (ret < 0) { _err("I2C Error: %d\n", ret); return ret; }
  return OK;
}

static int pinephone_gt9xx_irq_attach(const struct gt9xx_board_s *state,
                                      xcpt_t isr,
                                      FAR void *arg)
{
  _info("\n");

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

// Called by Touch Panel Interrupt Handler
static void pinephone_gt9xx_irq_enable(const struct gt9xx_board_s *state,
                                       bool enable)
{
  int ret;

  _info("enable=%d\n", enable);
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
  _info("on=%d\n", on);
  return OK;
}
