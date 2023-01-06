/****************************************************************************
 * include/nuttx/input/gt9xx.h
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

#ifndef __INCLUDE_NUTTX_INPUT_GT9XX_H
#define __INCLUDE_NUTTX_INPUT_GT9XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Board Configuration */

struct gt9xx_board_s
{
  int (*irq_attach) (const struct gt9xx_board_s *state,
                     xcpt_t isr,
                     FAR void *arg);
  void (*irq_enable) (const struct gt9xx_board_s *state, bool enable);
  int (*set_power) (const struct gt9xx_board_s *state, bool on);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Device Registration */

int gt9xx_register(FAR const char *devpath,
                   FAR struct i2c_master_s *dev,
                   uint8_t i2c_devaddr,
                   const struct gt9xx_board_s *board_config);

#endif /* __INCLUDE_NUTTX_INPUT_GT9XX_H */
