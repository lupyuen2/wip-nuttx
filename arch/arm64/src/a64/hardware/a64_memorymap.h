/****************************************************************************
 * arch/arm64/src/a64/hardware/a64_memorymap.h
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

#ifndef __ARCH_ARM64_SRC_A64_HARDWARE_A64_MEMORYMAP_H
#define __ARCH_ARM64_SRC_A64_HARDWARE_A64_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A1X offsets from the peripheral section base address */

#define A1X_PIO_OFFSET       0x00020800 /* PIO             0x01c2:0800-0x01c2:0bff 1K */

/* A64 Memory Map */

#define A1X_PERIPH_VSECTION  0x01c00000 /* Peripherals     0x01c0:0000-0x01c4:ffff */

/* Peripheral virtual base addresses */

#define A1X_PIO_VADDR        (A1X_PERIPH_VSECTION+A1X_PIO_OFFSET)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM64_SRC_A64_HARDWARE_A64_MEMORYMAP_H */
