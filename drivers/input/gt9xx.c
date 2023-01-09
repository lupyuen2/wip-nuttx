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

/* Reference:
 * "NuttX RTOS for PinePhone: Touch Panel"
 * https://lupyuen.github.io/articles/touch2
 */

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

/* Default I2C Frequency is 400 kHz */

#ifndef CONFIG_INPUT_GT9XX_I2C_FREQUENCY
#  define CONFIG_INPUT_GT9XX_I2C_FREQUENCY 400000
#endif

/* Default Number of Poll Waiters is 1 */

#ifndef CONFIG_INPUT_GT9XX_NPOLLWAITERS
#  define CONFIG_INPUT_GT9XX_NPOLLWAITERS 1
#endif

/* I2C Registers for Goodix GT9XX Touch Panel */

#define GTP_REG_VERSION    0x8140  /* Product ID */
#define GTP_READ_COOR_ADDR 0x814e  /* Touch Panel Status */
#define GTP_POINT1         0x8150  /* Touch Point 1 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Touch Panel Device */

struct gt9xx_dev_s
{
  /* I2C bus and address for device */

  struct i2c_master_s *i2c;
  uint8_t addr;

  /* Callback for Board-Specific Operations */

  const struct gt9xx_board_s *board;

  /* Device State */

  mutex_t devlock;  /* Mutex to prevent concurrent reads */
  uint8_t cref;     /* Reference Counter for device */
  bool int_pending; /* True if a Touch Interrupt is pending processing */
  uint16_t x;       /* X Coordinate of Last Touch Point */
  uint16_t y;       /* Y Coordinate of Last Touch Point */
  uint8_t flags;    /* Touch Up or Touch Down for Last Touch Point */

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

/* File Operations for Touch Panel */

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

// Read from a Touch Panel Register over I2C
static int gt9xx_i2c_read(
  FAR struct gt9xx_dev_s *dev,  // I2C Device
  uint16_t reg,  // I2C Register
  uint8_t *buf,  // Receive Buffer
  size_t buflen  // Receive Buffer Size
) {
  // Send the Register Address, MSB first
  uint8_t regbuf[2] = {
    reg >> 8,   // First Byte: MSB
    reg & 0xff  // Second Byte: LSB
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

  if (ret < 0) { ierr("I2C Read failed: %d\n", ret); return ret; }

#ifdef CONFIG_DEBUG_INPUT_INFO
  iinfodumpbuffer("gt9xx_i2c_read", buf, buflen);
#endif /* CONFIG_DEBUG_INPUT_INFO */

  return OK;
}

// Write to a Touch Panel Register over I2C
static int gt9xx_i2c_write(
  FAR struct gt9xx_dev_s *dev,  // I2C Device
  uint16_t reg,  // I2C Register
  uint8_t val    // Value to be written
) {
  // Send the Register Address, MSB first, followed by the value
  uint8_t buf[3] = {
    reg >> 8,    // First Byte: MSB
    reg & 0xff,  // Second Byte: LSB
    val          // Value to be written
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

  if (ret < 0) { ierr("I2C Write failed: %d\n", ret); return ret; }
  return OK;
}

// Read the Product ID from the Touch Panel over I2C
static int gt9xx_probe_device(FAR struct gt9xx_dev_s *dev)
{
  int ret;
  uint8_t id[4];

  // Read the Product ID
  ret = gt9xx_i2c_read(dev, GTP_REG_VERSION, id, sizeof(id));
  if (ret < 0)
    {
      ierr("I2C Probe failed: %d\n", ret);
      return ret;
    }

  // For GT917S: Product ID will be "39 31 37 53" or "917S"

#ifdef CONFIG_DEBUG_INPUT_INFO
  iinfodumpbuffer("gt9xx_probe_device", id, sizeof(id));
#endif /* CONFIG_DEBUG_INPUT_INFO */

  return OK;
}

// Set the Touch Panel Status over I2C
static int gt9xx_set_status(
  FAR struct gt9xx_dev_s *dev,  // I2C Device
  uint8_t status  // Status value to be set
) {
  // Write to the Status Register over I2C
  return gt9xx_i2c_write(dev, GTP_READ_COOR_ADDR, status);
}

// Read the Touch Coordinates over I2C
static int gt9xx_read_touch_data(
  FAR struct gt9xx_dev_s *dev,  // I2C Device
  FAR struct touch_sample_s *sample  // Touch Sample
) {
  int ret;

  iinfo("\n");
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(sample != NULL);
  memset(sample, 0, sizeof(*sample));

  // Read the Touch Panel Status
  uint8_t status[1];
  ret = gt9xx_i2c_read(dev, GTP_READ_COOR_ADDR, status, sizeof(status));
  if (ret < 0)
    {
      ierr("Read Touch Panel Status failed: %d\n", ret);
      return ret;
    }
  // Shows "81"

  // Decode the Status Code and the Touched Points
  const uint8_t status_code    = status[0] & 0x80;  // Set to 0x80
  const uint8_t touched_points = status[0] & 0x0f;  // Set to 0x01

  if (status_code != 0 &&  // If Touch Panel Status is OK and...
      touched_points >= 1) {  // Touched Points is 1 or more

    // Read the First Touch Coordinates
    uint8_t touch[6];
    ret = gt9xx_i2c_read(dev, GTP_POINT1, touch, sizeof(touch));
    if (ret < 0)
      {
        ierr("Read Touch Point failed: %d\n", ret);
        return ret;
      }
    // Shows "92 02 59 05 1b 00"

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
  }

  // Set the Touch Panel Status to 0
  ret = gt9xx_set_status(dev, 0);
  if (ret < 0)
    {
      ierr("Set Touch Panel Status failed: %d\n", ret);
      return ret;
    }

  return OK;
}

// Read the Touch Coordinates, if any
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
  DEBUGASSERT(priv->board && priv->board->irq_enable);
  priv->board->irq_enable(priv->board, true);

  /* Begin Mutex: Lock to prevent concurrent reads */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;

  // If we're waiting for Touch Up, return the last sample
  if (priv->flags & TOUCH_DOWN)
    {
      iinfo("touch up x=%d, y=%d\n", priv->x, priv->y);

      // Begin Critical Section
      flags = enter_critical_section();

      // Mark the Last Touch Point as Touch Up
      priv->flags = TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID;

      // End Critical Section
      leave_critical_section(flags);

      // Return the Last Touch Point, changed to Touch Up
      memset(&sample, 0, sizeof(sample));
      sample.npoints = 1;
      sample.point[0].id = 0;
      sample.point[0].x = priv->x;
      sample.point[0].y = priv->y;
      sample.point[0].flags = priv->flags;
      memcpy(buffer, &sample, sizeof(sample));
      ret = OK;
    }

  // Read the Touch Sample only if screen has been touched
  else if (priv->int_pending)
    {
      ret = gt9xx_read_touch_data(priv, &sample);
      memcpy(buffer, &sample, sizeof(sample));

      // Begin Critical Section
      flags = enter_critical_section();

      // Clear the Interrupt Pending Flag
      priv->int_pending = false;

      // Remember the last Touch Point
      if (sample.npoints >= 1)
        {
          priv->x = sample.point[0].x;
          priv->y = sample.point[0].y;
          priv->flags = sample.point[0].flags;
        }

      // End Critical Section
      leave_critical_section(flags);
    }

  /* End Mutex: Unlock to allow next read */

  nxmutex_unlock(&priv->devlock);
  return (ret < 0) ? ret : outlen;
}

// Open the Touch Panel
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

  // Begin Mutex
  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("Lock Mutex failed: %d\n", ret);
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

      ret = gt9xx_probe_device(priv);
      if (ret < 0)
        {
          /* No such device, power off the board */

          priv->board->set_power(priv->board, false);
          goto out_lock;
        }

      /* Enable Interrupts */

      DEBUGASSERT(priv->board->irq_enable);
      priv->board->irq_enable(priv->board, true);

      priv->cref = use_count;
    }
  else
    {
      DEBUGASSERT(use_count < UINT8_MAX && use_count > priv->cref);

      priv->cref = use_count;
      ret = 0;
    }

  // End Mutex
out_lock:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

// Close the Touch Panel
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

  // Begin Mutex
  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("Lock Mutex failed: %d\n", ret);
      return ret;
    }

  use_count = priv->cref - 1;
  if (use_count == 0)
    {
      /* Disable interrupt */

      DEBUGASSERT(priv->board && priv->board->irq_enable);
      priv->board->irq_enable(priv->board, false);

      /* Last user, do power off */

      DEBUGASSERT(priv->board->set_power);
      priv->board->set_power(priv->board, false);
      priv->cref = use_count;
    }
  else
    {
      DEBUGASSERT(use_count > 0);

      priv->cref = use_count;
    }

  // End Mutex
  nxmutex_unlock(&priv->devlock);
  return OK;
}

// Block until a Touch Interrupt has been triggered
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

  // Enable the PIO Interrupt
  DEBUGASSERT(priv->board && priv->board->irq_enable);
  priv->board->irq_enable(priv->board, true);

  // Begin Mutex
  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("Lock Mutex failed: %d\n", ret);
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

  // End Mutex
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

  // Throttle the PIO Interrupt
  if (priv->int_pending) { priv->board->irq_enable(priv->board, false); }

  // Set the Interrupt Pending Flag
  flags = enter_critical_section();
  priv->int_pending = true;
  leave_critical_section(flags);

  // Notify the Poll Waiters
  poll_notify(priv->fds, CONFIG_INPUT_GT9XX_NPOLLWAITERS, POLLIN);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gt9xx_register
 *
 * Description:
 *   Register the Touch Panel Driver.  Attach the Interrupt Handler for the
 *   Touch Panel and disable the Touch Interrupt.
 *
 * Input Parameters:
 *   devpath      - Device Path (e.g. "/dev/input0")
 *   dev          - I2C Bus
 *   i2c_devaddr  - I2C Address of Touch Panel
 *   board_config - Callback for Board-Specific Operations
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
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

  // Allocate the Device Structure
  priv = kmm_zalloc(sizeof(struct gt9xx_dev_s));
  if (!priv)
    {
      ierr("GT9XX Memory Allocation failed\n");
      return -ENOMEM;
    }

  // Setup the Device Structure
  priv->addr = i2c_devaddr;
  priv->i2c = i2c_dev;
  priv->board = board_config;
  nxmutex_init(&priv->devlock);

  // Register the Touch Input Driver
  ret = register_driver(devpath, &g_gt9xx_fileops, 0666, priv);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      ierr("GT9XX Registration failed: %d\n", ret);
      return ret;
    }

  iinfo("Registered with %d\n", ret);

  // Attach the Interrupt Handler
  DEBUGASSERT(priv->board->irq_attach);
  priv->board->irq_attach(priv->board, gt9xx_isr_handler, priv);

  // Disable the Touch Interrupt
  DEBUGASSERT(priv->board->irq_enable);
  priv->board->irq_enable(priv->board, false);

  return OK;
}
