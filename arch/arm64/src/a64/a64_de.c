/****************************************************************************
 * arch/arm64/src/a64/a64_de.c
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
 *
 * "Rendering PinePhone's Display (DE and TCON0)"
 * https://lupyuen.github.io/articles/de
 *
 * "NuttX RTOS for PinePhone: Render Graphics in Zig"
 * https://lupyuen.github.io/articles/de2
 *
 * "A64 Page" refers to Allwinner A64 User Manual
 * https://lupyuen.github.io/images/Allwinner_A64_User_Manual_V1.1.pdf
 * 
 * "DE Page" refers to Allwinner Display Engine 2.0 Specification
 * https://lupyuen.github.io/images/Allwinner_DE2.0_Spec_V1.0.pdf
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include "hardware/a64_memorymap.h"
#include "arm64_arch.h"
#include "a64_de.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/// Hardware Registers for PinePhone's A64 Display Engine.
/// See https://lupyuen.github.io/articles/de#appendix-overview-of-allwinner-a64-display-engine
/// Display Engine Base Address is 0x0100 0000 (DE Page 24)
const A64_DE_ADDR = 0x01000000;

/// MIXER0 is at DE Offset 0x10 0000 (DE Page 24, 0x110 0000)
const A64_MIXER0_ADDR = A64_DE_ADDR + 0x100000;

/// GLB (Global Registers) is at MIXER0 Offset 0x0000 (DE Page 90, 0x110 0000)
const A64_GLB_ADDR = A64_MIXER0_ADDR + 0x0000;

/// BLD (Blender) is at MIXER0 Offset 0x1000 (DE Page 90, 0x110 1000)
const A64_BLD_ADDR = A64_MIXER0_ADDR + 0x1000;

/// OVL_UI(CH1) (UI Overlay 1) is at MIXER0 Offset 0x3000 (DE Page 102, 0x110 3000)
const A64_OVL_UI_CH1_ADDR = A64_MIXER0_ADDR + 0x3000;

/// UI_SCALER1(CH1) (UI Scaler 1) is at MIXER0 Offset 0x04 0000 (DE Page 90, 0x114 0000)
const A64_UI_SCALER1_CH1_ADDR = A64_MIXER0_ADDR + 0x040000;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int a64_de_test_pattern(void)
{

}
