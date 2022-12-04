/****************************************************************************
 * boards/arm64/a64/pinephone/include/board.h
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

#ifndef __BOARDS_ARM64_A64_PINEPHONE_INCLUDE_BOARD_H
#define __BOARDS_ARM64_A64_PINEPHONE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

////#include "hardware/a64_piocfg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Since NuttX is booted from a loader on the A10, clocking should already
 * be setup when NuttX starts.
 */

/* LED definitions **********************************************************/

/* LED index values for use with board_userled() */

#define BOARD_LED1        0  /* Red LED */
#define BOARD_LED2        1  /* Green LED */
#define BOARD_LED3        2  /* Blue LED */
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

#endif /* __BOARDS_ARM64_A64_PINEPHONE_INCLUDE_BOARD_H */
