/****************************************************************************
 * arch/arm/src/sama5/chip.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <arch/sama5/chip.h>
#endif

#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* arch/arm/src/armv7-a/l2cc_pl310.h includes this file and expects it to
 * provide the address of the L2CC-PL310 implementation.
 */

#define L2CC_VBASE SAM_L2CC_VSECTION

/* Cache line sizes (in bytes) */

#define ARMV7A_DCACHE_LINESIZE 32  /* 32 bytes (8 words) */
#define ARMV7A_ICACHE_LINESIZE 32  /* 32 bytes (8 words) */

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_H */
