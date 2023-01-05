/****************************************************************************
 * drivers/input/gt9xx.c
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
#include <nuttx/input/gt9xx.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef CONFIG_INPUT_GT9XX_I2C_FREQUENCY
#  define CONFIG_INPUT_GT9XX_I2C_FREQUENCY 400000
#endif

#ifndef CONFIG_INPUT_GT9XX_NPOLLWAITERS
#  define CONFIG_INPUT_GT9XX_NPOLLWAITERS 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gt9xx_dev_s
{
  /* I2C bus and address for device */

  struct i2c_master_s *i2c;
  uint8_t addr;

  /* Configuration for device */

  struct gt9xx_board_s *board;
  mutex_t devlock;
  uint8_t cref;
  bool int_pending;

  /* Poll Waiters for device */

  struct pollfd *fds[CONFIG_INPUT_GT9XX_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gt9xx_open(FAR struct file *filep);
static int gt9xx_close(FAR struct file *filep);
static ssize_t gt9xx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static int gt9xx_poll(FAR struct file *filep, FAR struct pollfd *fds,
                      bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_gt9xx_fileops =
{
  gt9xx_open,   /* open */
  gt9xx_close,  /* close */
  gt9xx_read,   /* read */
  NULL,         /* write */
  NULL,         /* seek */
  NULL,         /* ioctl */
  NULL,         /* truncate */
  NULL,         /* mmap */
  gt9xx_poll    /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

static int gt9xx_i2c_read(
  struct i2c_master_s *i2c,  // I2C Bus
  uint16_t reg,  // I2C Register
  uint8_t *buf,  // Receive Buffer
  size_t buflen  // Receive Buffer Size
) {
  uint32_t freq = CONFIG_INPUT_GT9XX_I2C_FREQUENCY;  // 400 kHz
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

static int gt9xx_set_status(
  struct i2c_master_s *i2c,  // I2C Bus
  uint8_t status  // Status value to be set
) {
  uint16_t reg = GOODIX_READ_COORD_ADDR;  // I2C Register
  uint32_t freq = CONFIG_INPUT_GT9XX_I2C_FREQUENCY;  // 400 kHz
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

// Read Touch Panel over I2C
static void gt9xx_get_touch_data(struct i2c_master_s *i2c)
{
  // Read the Product ID
  uint8_t id[4];
  gt9xx_i2c_read(i2c, GOODIX_REG_ID, id, sizeof(id));
  // Shows "39 31 37 53" or "917S"

  // Read the Touch Panel Status
  uint8_t status[1];
  gt9xx_i2c_read(i2c, GOODIX_READ_COORD_ADDR, status, sizeof(status));
  // Shows "81"

  const uint8_t status_code    = status[0] & 0x80;  // Set to 0x80
  const uint8_t touched_points = status[0] & 0x0f;  // Set to 0x01

  if (status_code != 0 &&  // If Touch Panel Status is OK and...
      touched_points >= 1) {  // Touched Points is 1 or more

    // Dump the receive buffer
    // infodumpbuffer("buf", status, sizeof(status));

    // Read the First Touch Coordinates
    uint8_t touch[6];
    gt9xx_i2c_read(i2c, GOODIX_POINT1_X_ADDR, touch, sizeof(touch));
    // Shows "92 02 59 05 1b 00"

    // Dump the receive buffer
    // infodumpbuffer("buf", touch, sizeof(touch));

    const uint16_t x = touch[0] + (touch[1] << 8);
    const uint16_t y = touch[2] + (touch[3] << 8);
    _info("touch x=%d, y=%d\n", x, y);
    // Shows "touch x=658, y=1369"
  }

  // Set the Touch Panel Status to 0
  gt9xx_set_status(i2c, 0);
}

static ssize_t gt9xx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  size_t outlen;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;

  outlen = sizeof(struct gt9xx_sensor_status_s);
  if (buflen >= outlen)
    {
      ret = gt9xx_get_sensor_status(priv, buffer);
    }

  flags = enter_critical_section();
  priv->int_pending = false;
  leave_critical_section(flags);

  nxmutex_unlock(&priv->devlock);
  return ret < 0 ? ret : outlen;
}

static int gt9xx_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  unsigned int use_count;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  use_count = priv->cref + 1;
  if (use_count == 1)
    {
      /* First user, do power on. */

      ret = priv->board->set_power(priv->board, true);
      if (ret < 0)
        {
          goto out_lock;
        }

      /* Let chip to power up before probing */

      nxsig_usleep(100 * 1000);

      /* Check that device exists on I2C. */

      ret = gt9xx_probe_device(priv);
      if (ret < 0)
        {
          /* No such device. Power off the switch. */

          priv->board->set_power(priv->board, false);
          goto out_lock;
        }

      priv->cref = use_count;
    }
  else
    {
      DEBUGASSERT(use_count < UINT8_MAX && use_count > priv->cref);

      priv->cref = use_count;
      ret = 0;
    }

out_lock:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

static int gt9xx_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  int use_count;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  use_count = priv->cref - 1;
  if (use_count == 0)
    {
      /* Disable interrupt */

      priv->board->irq_enable(priv->board, false);

      /* Set chip in low-power mode. */

      gt9xx_enter_low_power_mode(priv);

      /* Last user, do power off. */

      priv->board->set_power(priv->board, false);
      priv->cref = use_count;
    }
  else
    {
      DEBUGASSERT(use_count > 0);

      priv->cref = use_count;
    }

  nxmutex_unlock(&priv->devlock);
  return 0;
}

static int gt9xx_poll(FAR struct file *filep, FAR struct pollfd *fds,
                      bool setup)
{
  FAR struct gt9xx_dev_s *priv;
  FAR struct inode *inode;
  bool pending;
  int ret = 0;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct gt9xx_dev_s *)inode->i_private;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference.
       */

      for (i = 0; i < CONFIG_INPUT_GT9XX_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_INPUT_GT9XX_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
        }
      else
        {
          pending = priv->int_pending;
          if (pending)
            {
              poll_notify(priv->fds,
                          CONFIG_INPUT_GT9XX_NPOLLWAITERS,
                          POLLIN);
            }
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

out:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

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
  poll_notify(priv->fds, CONFIG_INPUT_GT9XX_NPOLLWAITERS, POLLIN);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int gt9xx_register(FAR const char *devpath,
                   FAR struct i2c_master_s *i2c_dev,
                   uint8_t i2c_devaddr,
                   struct gt9xx_board_s *board_config)
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

  // Register Touch Input Driver
  ret = register_driver(devpath, &g_gt9xx_fileops, 0666, priv);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      _err("Error occurred during the driver registering\n");  // TODO
      return ret;
    }

  _info("Registered with %d\n", ret);  // TODO

  // Prepare interrupt line and handler
  priv->board->irq_attach(priv->board, gt9xx_isr_handler, priv);
  priv->board->irq_enable(priv->board, false);

  // TODO
  // Attach the PIO Interrupt Handler
  // if (irq_attach(A64_IRQ_PH_EINT, gt9xx_isr_handler, priv) < 0)
  //   {
  //     _err("irq_attach failed\n");
  //     return ERROR;
  //   }

  // Enable the PIO Interrupt
  // up_enable_irq(A64_IRQ_PH_EINT);

  // Configure the Touch Panel Interrupt
  // ret = a64_pio_config(CTP_INT);
  // DEBUGASSERT(ret == 0);

  // Enable the Touch Panel Interrupt
  // ret = a64_pio_irqenable(CTP_INT);
  // DEBUGASSERT(ret == 0);

  return OK;
}
