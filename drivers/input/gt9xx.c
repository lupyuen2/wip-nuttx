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

// TODO

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

static int gt9xx_i2c_read(FAR struct gt9xx_dev_s *dev, uint8_t reg,
                          uint8_t *buf, size_t buflen)
{
  // TODO
}

static ssize_t gt9xx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  // TODO
}

static int gt9xx_open(FAR struct file *filep)
{
  // TOOD
}

static int gt9xx_close(FAR struct file *filep)
{
  // TODO
}

static int gt9xx_poll(FAR struct file *filep, FAR struct pollfd *fds,
                      bool setup)
{
  // TODO
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
      gt9xx_dbg("Error occurred during the driver registering\n");
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
