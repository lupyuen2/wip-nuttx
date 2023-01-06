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
#include <nuttx/input/touchscreen.h>
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

  const struct gt9xx_board_s *board;
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
  FAR struct gt9xx_dev_s *dev,  // I2C Device
  uint16_t reg,  // I2C Register
  uint8_t *buf,  // Receive Buffer
  size_t buflen  // Receive Buffer Size
) {
  // Send the Register Address, MSB first
  uint8_t regbuf[2] = {
    reg >> 8,   // Swap the bytes
    reg & 0xff  // Swap the bytes
  };

  // Compose the I2C Messages
  struct i2c_msg_s msgv[2] =
  {
    {
      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = sizeof(regbuf)
    },
    {
      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = I2C_M_READ,
      .buffer    = buf,
      .length    = buflen
    }
  };

  // Execute the I2C Transfer
  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);
  int ret = I2C_TRANSFER(dev->i2c, msgv, msgv_len);

  if (ret < 0) { ierr("I2C Error: %d\n", ret); return ret; }

  // Dump the receive buffer
  // infodumpbuffer("buf", buf, buflen);
  return OK;
}

static int gt9xx_set_status(
  FAR struct gt9xx_dev_s *dev,  // I2C Device
  uint8_t status  // Status value to be set
) {
  const uint16_t reg = GOODIX_READ_COORD_ADDR;  // I2C Register
  uint8_t buf[3] = {
    reg >> 8,    // Swap the bytes
    reg & 0xff,  // Swap the bytes
    status
  };

  // Compose the I2C Message
  struct i2c_msg_s msgv[1] =
  {
    {
      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = buf,
      .length    = sizeof(buf)
    }
  };

  // Execute the I2C Transfer
  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);
  int ret = I2C_TRANSFER(dev->i2c, msgv, msgv_len);

  if (ret < 0) { ierr("I2C Error: %d\n", ret); return ret; }
  return OK;
}

// TODO: Move to struct
static uint16_t last_x = 0;
static uint16_t last_y = 0;
static uint8_t last_flags = 0;

// Read Touch Panel over I2C
static int gt9xx_read_touch_data(
  FAR struct gt9xx_dev_s *dev,  // I2C Device
  FAR struct touch_sample_s *sample  // Touch Sample
) {
  iinfo("\n");
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(sample != NULL);
  memset(sample, 0, sizeof(*sample));

  // Read the Product ID
  // uint8_t id[4];
  // gt9xx_i2c_read(dev, GOODIX_REG_ID, id, sizeof(id));
  // Shows "39 31 37 53" or "917S"

  // Read the Touch Panel Status
  uint8_t status[1];
  gt9xx_i2c_read(dev, GOODIX_READ_COORD_ADDR, status, sizeof(status));
  // Shows "81"

  // Decode the Status Code and the Touched Points
  const uint8_t status_code    = status[0] & 0x80;  // Set to 0x80
  const uint8_t touched_points = status[0] & 0x0f;  // Set to 0x01

  if (status_code != 0 &&  // If Touch Panel Status is OK and...
      touched_points >= 1) {  // Touched Points is 1 or more

    // Dump the receive buffer
    // infodumpbuffer("buf", status, sizeof(status));

    // Read the First Touch Coordinates
    uint8_t touch[6];
    gt9xx_i2c_read(dev, GOODIX_POINT1_X_ADDR, touch, sizeof(touch));
    // Shows "92 02 59 05 1b 00"

    // Dump the receive buffer
    // infodumpbuffer("buf", touch, sizeof(touch));

    // Decode the Touch Coordinates
    const uint16_t x = touch[0] + (touch[1] << 8);
    const uint16_t y = touch[2] + (touch[3] << 8);
    const uint8_t flags = TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID;
    iinfo("touch down x=%d, y=%d\n", x, y);
    // Shows "touch x=658, y=1369"

    // Return the Touch Coordinates
    sample->npoints = 1;
    sample->point[0].id = 0;
    sample->point[0].x = x;
    sample->point[0].y = y;
    sample->point[0].flags = flags;

    // Remember the last valid touch data
    last_x = x;
    last_y = y;
    last_flags = flags;
  }

  // Set the Touch Panel Status to 0
  gt9xx_set_status(dev, 0);

  // Enable the PIO Interrupt
  up_enable_irq(A64_IRQ_PH_EINT); //// TODO

  return OK;
}

static ssize_t gt9xx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  struct touch_sample_s sample; /* Support only 1 point per sample */
  const size_t outlen = sizeof(sample);
  irqstate_t flags;
  int ret;

  iinfo("buflen=%ld\n", buflen);
  if (buflen < outlen)
    {
      ierr("Buffer should be at least %ld bytes, got %ld bytes\n", outlen, buflen);
      return -EINVAL;
    }

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  // Enable the PIO Interrupt
  up_enable_irq(A64_IRQ_PH_EINT); //// TODO

  /* Lock mutex to prevent concurrent reads */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;

  // If we're waiting for Touch Up, return the last sample
  if (last_flags & TOUCH_DOWN)
    {
      iinfo("touch up x=%d, y=%d\n", last_x, last_y);
      last_flags = TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID;
      memset(&sample, 0, sizeof(sample));
      sample.npoints = 1;
      sample.point[0].id = 0;
      sample.point[0].x = last_x;
      sample.point[0].y = last_y;
      sample.point[0].flags = last_flags;
      memcpy(buffer, &sample, sizeof(sample));
      ret = OK;
    }
  // Read the Touch Sample only if screen has been touched
  else if (priv->int_pending)
    {
      ret = gt9xx_read_touch_data(priv, &sample);
      memcpy(buffer, &sample, sizeof(sample));

      /* Clear pending flag with critical section */

      flags = enter_critical_section();
      priv->int_pending = false;
      leave_critical_section(flags);
    }

  /* Unlock mutex to allow next read */

  nxmutex_unlock(&priv->devlock);
  return ret < 0 ? ret : outlen;
}

static int gt9xx_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  unsigned int use_count;
  int ret;

  iinfo("\n");

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
      /* First user, do power on */

      DEBUGASSERT(priv->board->set_power != NULL);
      ret = priv->board->set_power(priv->board, true);
      if (ret < 0)
        {
          goto out_lock;
        }

      /* Let chip to power up before probing */

      nxsig_usleep(100 * 1000);

      /* Check that device exists on I2C */

      // TODO
      // ret = gt9xx_probe_device(priv);
      // if (ret < 0)
      //   {
      //     /* No such device, power off the board */

      //     priv->board->set_power(priv->board, false);
      //     goto out_lock;
      //   }

      /* Enable Interrupts */

      DEBUGASSERT(priv->board->irq_enable != NULL);
      priv->board->irq_enable(priv->board, true);

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

  iinfo("\n");

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

      DEBUGASSERT(priv->board->irq_enable != NULL);
      priv->board->irq_enable(priv->board, false);

      /* Last user, do power off */

      DEBUGASSERT(priv->board->set_power != NULL);
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

  iinfo("setup=%d\n", setup);

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

  up_putc('.'); //// TODO
  // Disable the PIO Interrupt
  if (priv->int_pending) { up_disable_irq(A64_IRQ_PH_EINT); } //// TODO

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
                   const struct gt9xx_board_s *board_config)
{
  struct gt9xx_dev_s *priv;
  int ret = 0;

  iinfo("devpath=%s, i2c_devaddr=%d\n", devpath, i2c_devaddr);
  DEBUGASSERT(devpath != NULL && i2c_dev != NULL && board_config != NULL);

  // Allocate device private structure
  priv = kmm_zalloc(sizeof(struct gt9xx_dev_s));
  if (!priv)
    {
      ierr("Memory cannot be allocated for gt9xx\n");  // TODO
      return -ENOMEM;
    }

  // Setup device structure
  priv->addr = i2c_devaddr;
  priv->i2c = i2c_dev;
  priv->board = board_config;
  nxmutex_init(&priv->devlock);

  // Register Touch Input Driver
  ret = register_driver(devpath, &g_gt9xx_fileops, 0666, priv);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      ierr("Error occurred during the gt9xx registration\n");  // TODO
      return ret;
    }

  iinfo("Registered with %d\n", ret);  // TODO

  // Prepare interrupt line and handler
  DEBUGASSERT(priv->board->irq_attach != NULL && priv->board->irq_enable != NULL);
  priv->board->irq_attach(priv->board, gt9xx_isr_handler, priv);
  priv->board->irq_enable(priv->board, false);

////#define TODO
#ifdef TODO
  // Enable Touch Panel Interrupt
  priv->board->irq_enable(priv->board, true);

  // Poll for Touch Panel Interrupt
  for (int i = 0; i < 1000; i++) {  // Poll for 10 seconds

    // If Touch Panel Interrupt has been triggered...
    if (priv->int_pending) {

      // Read the Touch Panel over I2C
      struct touch_sample_s sample;
      ret = gt9xx_read_touch_data(priv, &sample);
      DEBUGASSERT(ret == OK);

      // Reset the Interrupt Pending Flag
      priv->int_pending = false;
    }

    // Wait a while
    up_mdelay(10);  // 10 milliseconds
  }
#endif // TODO

  return OK;
}
