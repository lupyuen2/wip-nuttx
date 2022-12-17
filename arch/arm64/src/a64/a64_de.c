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

/// MIXER0 is at DE Offset 0x10 0000 (DE Page 24, 0x110 0000)
#define A64_MIXER0_ADDR (A64_DE_ADDR + 0x100000)

/// GLB (Global Registers) is at MIXER0 Offset 0x0000 (DE Page 90, 0x110 0000)
#define A64_GLB_ADDR (A64_MIXER0_ADDR + 0x0000)

/// BLD (Blender) is at MIXER0 Offset 0x1000 (DE Page 90, 0x110 1000)
#define A64_BLD_ADDR (A64_MIXER0_ADDR + 0x1000)

/// OVL_UI(CH1) (UI Overlay 1) is at MIXER0 Offset 0x3000 (DE Page 102, 0x110 3000)
#define A64_OVL_UI_CH1_ADDR (A64_MIXER0_ADDR + 0x3000)

/// UI_SCALER1(CH1) (UI Scaler 1) is at MIXER0 Offset 0x04 0000 (DE Page 90, 0x114 0000)
#define A64_UI_SCALER1_CH1_ADDR (A64_MIXER0_ADDR + 0x040000)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

// Init PinePhone's Allwinner A64 Display Engine.
// Called by display_init() in p-boot Display Code.
// Must be called before any DE operations
int a64_de_init(void)
{
  return OK;
}

/// Initialise the UI Blender for PinePhone's A64 Display Engine.
/// Must be called after a64_de_init, and before a64_de_ui_channel_init
int a64_de_blender_init(void)
{
  return OK;
}

/// Initialise a UI Channel for PinePhone's A64 Display Engine.
/// We use 3 UI Channels: Base UI Channel (#1) plus 2 Overlay UI Channels (#2, #3).
/// Must be called after a64_de_blender_init, and before a64_de_enable
int a64_de_ui_channel_init(
  uint8_t channel,   // UI Channel Number: 1, 2 or 3
  void *fbmem,     // Start of frame buffer memory, or null if this channel should be disabled
  size_t fblen,           // Length of frame buffer memory in bytes
  uint16_t stride,  // Length of a line in bytes (4 bytes per pixel)
  uint16_t xres,  // Horizontal resolution in pixel columns
  uint16_t yres,  // Vertical resolution in pixel rows
  uint16_t xoffset,  // Horizontal offset in pixel columns
  uint16_t yoffset  // Vertical offset in pixel rows
)
{
  return OK;
}

/// Set UI Blender Route, enable Blender Pipes and apply the settings for PinePhone's A64 Display Engine.
/// Must be called after a64_de_ui_channel_init
int a64_de_enable(
    uint8_t channels  // Number of enabled UI Channels
)
{
  return OK;
}
