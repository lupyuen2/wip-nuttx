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
  /* Set High Speed SRAM to DMA Mode ****************************************/

  ginfo("Set High Speed SRAM to DMA Mode\n");

  // Set High Speed SRAM to DMA Mode
  // Set BIST_DMA_CTRL_SEL to 0 for DMA (DMB) (A31 Page 191, 0x1C0 0004)
  // BIST_DMA_CTRL_SEL (Bist and DMA Control Select) is Bit 0 of SRAM_CTRL_REG1
  // SRAM_CTRL_REG1 (SRAM Control Register 1) is at SRAM Registers Offset 0x4
  
  const SRAM_CTRL_REG1 = SRAM_REGISTERS_BASE_ADDRESS + 0x4;
  comptime{ assert(SRAM_CTRL_REG1 == 0x1C00004); }
  putreg32(0x0, SRAM_CTRL_REG1);  // TODO: DMB

  /* Set Display Engine PLL to 297 MHz **************************************/

  ginfo("Set Display Engine PLL to 297 MHz\n");

  // Set Display Engine PLL to 297 MHz
  // Set PLL_DE_CTRL_REG to 0x8100 1701 (DMB)
  //   PLL_ENABLE    (Bit 31)       = 1  (Enable PLL)
  //   PLL_MODE_SEL  (Bit 24)       = 1  (Integer Mode)
  //   PLL_FACTOR_N  (Bits 8 to 14) = 23 (N = 24)
  //   PLL_PRE_DIV_M (Bits 0 to 3)  = 1  (M = 2)
  // Actual PLL Output = 24 MHz * N / M = 288 MHz
  // (Slighltly below 297 MHz due to truncation)
  // PLL_DE_CTRL_REG (PLL Display Engine Control Register) is at CCU Offset 0x0048
  // (A64 Page 96, 0x1C2 0048)
  
  const PLL_ENABLE:   u32 = 1  << 31;  // Enable PLL
  const PLL_MODE_SEL: u25 = 1  << 24;  // Integer Mode
  const PLL_FACTOR_N: u15 = 23 <<  8;  // N = 24
  const PLL_PRE_DIV_M: u4 = 1  <<  0;  // M = 2
  const pll = PLL_ENABLE
      | PLL_MODE_SEL
      | PLL_FACTOR_N
      | PLL_PRE_DIV_M;
  comptime{ assert(pll == 0x81001701); }

  const PLL_DE_CTRL_REG = CCU_BASE_ADDRESS + 0x0048;
  comptime{ assert(PLL_DE_CTRL_REG == 0x1C20048); }
  putreg32(pll, PLL_DE_CTRL_REG);  // TODO: DMB

  /* Wait for Display Engine PLL to be stable *******************************/

  ginfo("Wait for Display Engine PLL to be stable\n");

  // Wait for Display Engine PLL to be stable
  // Poll PLL_DE_CTRL_REG (from above) until LOCK (Bit 28) is 1
  // (PLL is Locked and Stable)
  
  while (getreg32(PLL_DE_CTRL_REG) & (1 << 28) == 0) {}

  /* Set Special Clock to Display Engine PLL ********************************/

  ginfo("Set Special Clock to Display Engine PLL\n");

  // Set Special Clock to Display Engine PLL
  // Clear DE_CLK_REG bits 0x0300 0000
  // Set DE_CLK_REG bits 0x8100 0000
  // SCLK_GATING (Bit 31)        = 1 (Enable Special Clock)
  // CLK_SRC_SEL (Bits 24 to 26) = 1 (Clock Source is Display Engine PLL)
  // DE_CLK_REG (Display Engine Clock Register) is at CCU Offset 0x0104
  // (A64 Page 117, 0x1C2 0104)
  
  const SCLK_GATING: u32 = 1 << 31;  // Enable Special Clock
  const CLK_SRC_SEL: u27 = 1 << 24;  // Clock Source is Display Engine PLL
  const clk = SCLK_GATING
      | CLK_SRC_SEL;
  comptime{ assert(clk == 0x81000000); }

  const SCLK_GATING_MASK: u32 = 0b1   << 31;
  const CLK_SRC_SEL_MASK: u27 = 0b111 << 24;
  const clk_mask = SCLK_GATING_MASK
      | CLK_SRC_SEL_MASK;

  const DE_CLK_REG = CCU_BASE_ADDRESS + 0x0104;
  comptime{ assert(DE_CLK_REG == 0x1C20104); }
  modreg32(clk, clk_mask, DE_CLK_REG);

  /* Enable AHB for Display Engine: De-Assert Display Engine ****************/

  ginfo("Enable AHB for Display Engine: De-Assert Display Engine\n");

  // Enable AHB (AMBA High-speed Bus) for Display Engine: De-Assert Display Engine
  // Set BUS_SOFT_RST_REG1 bits 0x1000
  // DE_RST (Bit 12) = 1 (De-Assert Display Engine)
  // BUS_SOFT_RST_REG1 (Bus Software Reset Register 1) is at CCU Offset 0x02C4
  // (A64 Page 140, 0x1C2 02C4)
  
  const DE_RST: u13 = 1 << 12;  // De-Assert Display Engine
  const BUS_SOFT_RST_REG1 = CCU_BASE_ADDRESS + 0x02C4;
  comptime{ assert(BUS_SOFT_RST_REG1 == 0x1C202C4); }
  modreg32(DE_RST, DE_RST, BUS_SOFT_RST_REG1);

  /* Enable AHB for Display Engine: Pass Display Engine *********************/

  ginfo("Enable AHB for Display Engine: Pass Display Engine\n");

  // Enable AHB (AMBA High-speed Bus) for Display Engine: Pass Display Engine
  // Set BUS_CLK_GATING_REG1 bits 0x1000
  // DE_GATING (Bit 12) = 1 (Pass Display Engine)
  // BUS_CLK_GATING_REG1 (Bus Clock Gating Register 1) is at CCU Offset 0x0064
  // (A64 Page 102, 0x1C2 0064)
  
  const DE_GATING: u13 = 1 << 12;  // Pass Display Engine
  const BUS_CLK_GATING_REG1 = CCU_BASE_ADDRESS + 0x0064;
  comptime{ assert(BUS_CLK_GATING_REG1 == 0x1C20064); }
  modreg32(DE_GATING, DE_GATING, BUS_CLK_GATING_REG1);

  /* Enable Clock for MIXER0: SCLK Clock Pass *******************************/

  ginfo("Enable Clock for MIXER0: SCLK Clock Pass\n");

  // Enable Clock for MIXER0: SCLK Clock Pass
  // Set SCLK_GATE bits 0x1
  // CORE0_SCLK_GATE (Bit 0) = 1 (Clock Pass)
  // SCLK_GATE is at DE Offset 0x000
  // (DE Page 25, 0x100 0000)
  
  const CORE0_SCLK_GATE: u1 = 1 << 0;  // Clock Pass
  const SCLK_GATE = DISPLAY_ENGINE_BASE_ADDRESS + 0x000;
  comptime{ assert(SCLK_GATE == 0x1000000); }
  modreg32(CORE0_SCLK_GATE, CORE0_SCLK_GATE, SCLK_GATE);

  /* Enable Clock for MIXER0: HCLK Clock Reset Off **************************/

  ginfo("Enable Clock for MIXER0: HCLK Clock Reset Off\n");

  // Enable Clock for MIXER0: HCLK Clock Reset Off
  // Set AHB_RESET bits 0x1
  // CORE0_HCLK_RESET (Bit 0) = 1 (Reset Off)
  // AHB_RESET is at DE Offset 0x008
  // (DE Page 25, 0x100 0008)
  
  const CORE0_HCLK_RESET: u1 = 1 << 0;  // Reset Off
  const AHB_RESET = DISPLAY_ENGINE_BASE_ADDRESS + 0x008;
  comptime{ assert(AHB_RESET == 0x1000008); }
  modreg32(CORE0_HCLK_RESET, CORE0_HCLK_RESET, AHB_RESET);

  /* Enable Clock for MIXER0: HCLK Clock Pass *******************************/

  ginfo("Enable Clock for MIXER0: HCLK Clock Pass\n");

  // Enable Clock for MIXER0: HCLK Clock Pass
  // Set HCLK_GATE bits 0x1
  // CORE0_HCLK_GATE (Bit 0) = 1 (Clock Pass)
  // HCLK_GATE is at DE Offset 0x004
  // (DE Page 25, 0x100 0004)
  
  const CORE0_HCLK_GATE: u1 = 1 << 0;  // Clock Pass
  const HCLK_GATE = DISPLAY_ENGINE_BASE_ADDRESS + 0x004;
  comptime{ assert(HCLK_GATE == 0x1000004); }
  modreg32(CORE0_HCLK_GATE, CORE0_HCLK_GATE, HCLK_GATE);

  /* Route MIXER0 to TCON0 **************************************************/

  ginfo("Route MIXER0 to TCON0\n");

  // Route MIXER0 to TCON0
  // Clear DE2TCON_MUX bits 0x1
  // DE2TCON_MUX (Bit 0) = 0
  // (Route MIXER0 to TCON0; Route MIXER1 to TCON1)
  // DE2TCON_MUX is at DE Offset 0x010
  // (DE Page 26, 0x100 0010)
  
  const DE2TCON_MUX_MASK: u1 = 1 << 0;  // Route MIXER0 to TCON0; Route MIXER1 to TCON1
  const DE2TCON_MUX = DISPLAY_ENGINE_BASE_ADDRESS + 0x010;
  comptime{ assert(DE2TCON_MUX == 0x1000010); }
  modreg32(0, DE2TCON_MUX_MASK, DE2TCON_MUX);

  /* Clear MIXER0 Registers: GLB, BLD, OVL_V, OVL_UI ************************/

  ginfo("Clear MIXER0 Registers: GLB, BLD, OVL_V, OVL_UI\n");

  // Clear MIXER0 Registers: Global Registers (GLB), Blender (BLD), Video Overlay (OVL_V), UI Overlay (OVL_UI)
  // Set MIXER0 Offsets 0x0000 - 0x5FFF to 0
  // GLB (Global Regisers) at MIXER0 Offset 0x0000
  // BLD (Blender) at MIXER0 Offset 0x1000
  // OVL_V(CH0) (Video Overlay) at MIXER0 Offset 0x2000
  // OVL_UI(CH1) (UI Overlay 1) at MIXER0 Offset 0x3000
  // OVL_UI(CH2) (UI Overlay 2) at MIXER0 Offset 0x4000
  // OVL_UI(CH3) (UI Overlay 3) at MIXER0 Offset 0x5000
  // (DE Page 90, 0x110 0000 - 0x110 5FFF)
  
  var i: usize = 0;
  while (i < 0x6000) : (i += 4) {
      putreg32(0, MIXER0_BASE_ADDRESS + i);
      enableLog = false;
  }
  enableLog = true;
  debug("  to *0x{x} = 0x0", .{ MIXER0_BASE_ADDRESS + i - 1 });

  /* Disable MIXER0 VSU *****************************************************/

  ginfo("Disable MIXER0 VSU\n");

  // Disable MIXER0 Video Scaler (VSU)
  // Set to 0: VS_CTRL_REG at VIDEO_SCALER(CH0) Offset 0
  // EN (Bit 0) = 0 (Disable Video Scaler)
  // (DE Page 130, 0x112 0000)
  
  const VS_CTRL_REG = VIDEO_SCALER_BASE_ADDRESS + 0;
  comptime{ assert(VS_CTRL_REG == 0x1120000); }
  putreg32(0, VS_CTRL_REG);

  /* Disable MIXER0 Undocumented ********************************************/

  ginfo("Disable MIXER0 Undocumented\n");

  // TODO: 0x113 0000 is undocumented
  // Is there a mixup with UI_SCALER3?
  
  const 1130000 = 0x1130000;
  putreg32(0, 1130000);

  /* Disable MIXER0 UI_SCALER1 **********************************************/

  ginfo("Disable MIXER0 UI_SCALER1\n");

  // Disable MIXER0 UI_SCALER1
  // Set to 0: UIS_CTRL_REG at UI_SCALER1(CH1) Offset 0
  // EN (Bit 0) = 0 (Disable UI Scaler)
  // (DE Page 66, 0x114 0000)
  
  const UIS_CTRL_REG1 = UI_SCALER1_BASE_ADDRESS + 0;
  comptime{ assert(UIS_CTRL_REG1 == 0x1140000); }
  putreg32(0, UIS_CTRL_REG1);

  /* Disable MIXER0 UI_SCALER2 **********************************************/

  ginfo("Disable MIXER0 UI_SCALER2\n");

  // Disable MIXER0 UI_SCALER2
  // Set to 0: UIS_CTRL_REG at UI_SCALER2(CH2) Offset 0
  // EN (Bit 0) = 0 (Disable UI Scaler)
  // (DE Page 66, 0x115 0000)
  
  const UIS_CTRL_REG2 = UI_SCALER2_BASE_ADDRESS + 0;
  comptime{ assert(UIS_CTRL_REG2 == 0x1150000); }
  putreg32(0, UIS_CTRL_REG2);

  // TODO: Missing UI_SCALER3(CH3) at MIXER0 Offset 0x06 0000 (DE Page 90, 0x116 0000)
  // Is there a mixup with 0x113 0000 above?

  /* Disable MIXER0 FCE *****************************************************/

  ginfo("Disable MIXER0 FCE\n");

  // Disable MIXER0 FCE
  // Set to 0: GCTRL_REG(FCE) at FCE Offset 0
  // EN (Bit 0) = 0 (Disable FCE)
  // (DE Page 62, 0x11A 0000)
  
  const GCTRL_REG_FCE = FCE_BASE_ADDRESS + 0;
  comptime{ assert(GCTRL_REG_FCE == 0x11A0000); }
  putreg32(0, GCTRL_REG_FCE);

  /* Disable MIXER0 BWS *****************************************************/

  ginfo("Disable MIXER0 BWS\n");

  // Disable MIXER0 BWS
  // Set to 0: GCTRL_REG(BWS) at BWS Offset 0
  // EN (Bit 0) = 0 (Disable BWS)
  // (DE Page 42, 0x11A 2000)
  
  const GCTRL_REG_BWS = BWS_BASE_ADDRESS + 0;
  comptime{ assert(GCTRL_REG_BWS == 0x11A2000); }
  putreg32(0, GCTRL_REG_BWS);

  /* Disable MIXER0 LTI *****************************************************/

  ginfo("Disable MIXER0 LTI\n");

  // Disable MIXER0 LTI
  // Set to 0: LTI_CTL at LTI Offset 0
  // LTI_EN (Bit 0) = 0 (Close LTI)
  // (DE Page 72, 0x11A 4000)
  
  const LTI_CTL = LTI_BASE_ADDRESS + 0;
  comptime{ assert(LTI_CTL == 0x11A4000); }
  putreg32(0, LTI_CTL);

  /* Disable MIXER0 PEAKING *************************************************/

  ginfo("Disable MIXER0 PEAKING\n");

  // Disable MIXER0 PEAKING
  // Set to 0: LP_CTRL_REG at PEAKING Offset 0
  // EN (Bit 0) = 0 (Disable PEAKING)
  // (DE Page 80, 0x11A 6000)
  
  const LP_CTRL_REG = PEAKING_BASE_ADDRESS + 0;
  comptime{ assert(LP_CTRL_REG == 0x11A6000); }
  putreg32(0, LP_CTRL_REG);

  /* Disable MIXER0 ASE *****************************************************/

  ginfo("Disable MIXER0 ASE\n");

  // Disable MIXER0 ASE
  // Set to 0: ASE_CTL_REG at ASE Offset 0
  // ASE_EN (Bit 0) = 0 (Disable ASE)
  // (DE Page 40, 0x11A 8000)
  
  const ASE_CTL_REG = ASE_BASE_ADDRESS + 0;
  comptime{ assert(ASE_CTL_REG == 0x11A8000); }
  putreg32(0, ASE_CTL_REG);

  /* Disable MIXER0 FCC *****************************************************/

  ginfo("Disable MIXER0 FCC\n");

  // Disable MIXER0 FCC
  // Set to 0: FCC_CTL_REG at FCC Offset 0
  // Enable (Bit 0) = 0 (Disable FCC)
  // (DE Page 56, 0x11A A000)
  
  const FCC_CTL_REG = FCC_BASE_ADDRESS + 0;
  comptime{ assert(FCC_CTL_REG == 0x11A_A000); }
  putreg32(0, FCC_CTL_REG);

  /* Disable MIXER0 DRC *****************************************************/

  ginfo("Disable MIXER0 DRC\n");

  // Disable MIXER0 DRC
  // Set to 0: GNECTL_REG at DRC Offset 0
  // BIST_EN (Bit 0) = 0 (Disable BIST)
  // (DE Page 49, 0x11B 0000)
  
  const GNECTL_REG = DRC_BASE_ADDRESS + 0;
  comptime{ assert(GNECTL_REG == 0x11B0000); }
  putreg32(0, GNECTL_REG);

  /* Enable MIXER0 **********************************************************/

  ginfo("Enable MIXER0\n");

  // Enable MIXER0
  // Set GLB_CTL to 1 (DMB)
  // EN (Bit 0) = 1 (Enable Mixer)
  // (DE Page 92)
  // GLB_CTL is at MIXER0 Offset 0
  // (DE Page 90, 0x110 0000)
  
  const EN_MIXER: u1 = 1 << 0;  // Enable Mixer
  const GLB_CTL = MIXER0_BASE_ADDRESS + 0;
  comptime{ assert(GLB_CTL == 0x1100000); }
  putreg32(EN_MIXER, GLB_CTL);  // TODO: DMB

  return OK;
}

/// Initialize the UI Blender for PinePhone's A64 Display Engine.
/// Must be called after a64_de_init, and before a64_de_ui_channel_init
int a64_de_blender_init(void)
{
  /* Set Blender Background *************************************************/

  ginfo("Set Blender Background\n");

  // Set Blender Background
  // BLD_BK_COLOR (Blender Background Color) at BLD Offset 0x88
  // Set to 0xFF00 0000 (Black Background Color)
  // RESERVED (Bits 24 to 31) = 0xFF (Undocumented)
  // RED   (Bits 16 to 23) = 0
  // GREEN (Bits 8  to 15) = 0
  // BLUE  (Bits 0  to 7)  = 0
  // (DE Page 109, 0x110 1088)
  
  const RESERVED: u32 = 0xFF << 24;
  const RED:      u24 = 0    << 16;
  const GREEN:    u16 = 0    << 8;
  const BLUE:     u8  = 0    << 0;
  const color = RESERVED
      | RED
      | GREEN
      | BLUE;
  comptime{ assert(color == 0xFF000000); }

  const BLD_BK_COLOR = BLD_BASE_ADDRESS + 0x88;
  comptime{ assert(BLD_BK_COLOR == 0x1101088); }
  putreg32(color, BLD_BK_COLOR);

  /* Set Blender Pre-Multiply ***********************************************/

  ginfo("Set Blender Pre-Multiply\n");

  // Set Blender Pre-Multiply
  // BLD_PREMUL_CTL (Blender Pre-Multiply Control) at BLD Offset 0x84
  // Set to 0 (No Pre-Multiply for Alpha, Pipes 0 to 3)
  // P3_ALPHA_MODE (Bit 3) = 0 (Pipe 3: No Pre-Multiply)
  // P2_ALPHA_MODE (Bit 2) = 0 (Pipe 2: No Pre-Multiply)
  // P1_ALPHA_MODE (Bit 1) = 0 (Pipe 1: No Pre-Multiply)
  // P0_ALPHA_MODE (Bit 0) = 0 (Pipe 0: No Pre-Multiply)
  // (DE Page 109, 0x110 1084)
  
  const P3_ALPHA_MODE: u4 = 0 << 3;  // Pipe 3: No Pre-Multiply
  const P2_ALPHA_MODE: u3 = 0 << 2;  // Pipe 2: No Pre-Multiply
  const P1_ALPHA_MODE: u2 = 0 << 1;  // Pipe 1: No Pre-Multiply
  const P0_ALPHA_MODE: u1 = 0 << 0;  // Pipe 0: No Pre-Multiply
  const premultiply = P3_ALPHA_MODE
      | P2_ALPHA_MODE
      | P1_ALPHA_MODE
      | P0_ALPHA_MODE;
  comptime{ assert(premultiply == 0); }

  const BLD_PREMUL_CTL = BLD_BASE_ADDRESS + 0x84;
  comptime{ assert(BLD_PREMUL_CTL == 0x1101084); }
  putreg32(premultiply, BLD_PREMUL_CTL);

  return OK;
}

/// Initialize a UI Channel for PinePhone's A64 Display Engine.
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
  // Validate Framebuffer Size and Stride at Compile Time
  comptime {
      assert(channel >= 1 and channel <= 3);
      assert(fblen == @intCast(usize, xres) * yres * 4);
      assert(stride == @intCast(usize, xres) * 4);
  }

  // OVL_UI(CH1) (UI Overlay 1) is at MIXER0 Offset 0x3000
  // OVL_UI(CH2) (UI Overlay 2) is at MIXER0 Offset 0x4000
  // OVL_UI(CH3) (UI Overlay 3) is at MIXER0 Offset 0x5000
  // (DE Page 102, 0x110 3000 / 0x110 4000 / 0x110 5000)
  const OVL_UI_BASE_ADDRESS = OVL_UI_CH1_BASE_ADDRESS
      + @intCast(u64, channel - 1) * 0x1000;
  comptime{ assert(OVL_UI_BASE_ADDRESS == 0x1103000 or OVL_UI_BASE_ADDRESS == 0x1104000 or OVL_UI_BASE_ADDRESS == 0x1105000); }

  // UI_SCALER1(CH1) is at MIXER0 Offset 0x04 0000
  // UI_SCALER2(CH2) is at MIXER0 Offset 0x05 0000
  // UI_SCALER3(CH3) is at MIXER0 Offset 0x06 0000
  // (DE Page 90, 0x114 0000 / 0x115 0000 / 0x116 0000)
  const UI_SCALER_BASE_ADDRESS = UI_SCALER1_CH1_BASE_ADDRESS
      + @intCast(u64, channel - 1) * 0x10000;

  // If UI Channel should be disabled...
  if (fbmem == null) {
      /* Disable Overlay and Pipe *******************************************/

      ginfo("Channel %d: Disable Overlay and Pipe\n", channel);

      // Disable Overlay and Pipe:
      // OVL_UI_ATTR_CTL (UI Overlay Attribute Control) at OVL_UI Offset 0x00
      // Set to 0 (Disable UI Overlay Channel)
      // LAY_EN (Bit 0) = 0 (Disable Layer)
      // (DE Page 102)
      debug("", .{  });
      const OVL_UI_ATTR_CTL = OVL_UI_BASE_ADDRESS + 0x00;
      comptime{ assert(OVL_UI_ATTR_CTL == 0x1103000 or OVL_UI_ATTR_CTL == 0x1104000 or OVL_UI_ATTR_CTL == 0x1105000); }
      putreg32(0, OVL_UI_ATTR_CTL);

      /* Disable Scaler *****************************************************/

      ginfo("Channel %d: Disable Scaler\n", channel);

      // Disable Scaler:
      // UIS_CTRL_REG at Offset 0 of UI_SCALER1(CH1) or UI_SCALER2(CH2) or UI_SCALER3(CH3)
      // Set to 0 (Disable UI Scaler)
      // EN (Bit 0) = 0 (Disable UI Scaler)
      // (DE Page 66)
      debug("", .{  });
      const UIS_CTRL_REG = UI_SCALER_BASE_ADDRESS + 0;
      comptime{ assert(UIS_CTRL_REG == 0x1140000 or UIS_CTRL_REG == 0x1150000 or UIS_CTRL_REG == 0x1160000); }
      putreg32(0, UIS_CTRL_REG);
      
      // Skip to next UI Channel
      return;
  }

  /* Set Overlay ************************************************************/

  ginfo("Channel %d: Set Overlay (%d x %d)\n", channel, xres, yres);

  // Set Overlay (Assume Layer = 0)
  // OVL_UI_ATTR_CTL (UI Overlay Attribute Control) at OVL_UI Offset 0x00
  // For Channel 1: Set to 0xFF00 0405
  // For Channel 2: Set to 0xFF00 0005
  // For Channel 3: Set to 0x7F00 0005
  // LAY_GLBALPHA (Bits 24 to 31) = 0xFF or 0x7F
  //   (Global Alpha Value is Opaque or Semi-Transparent)
  // LAY_FBFMT (Bits 8 to 12) = 4 or 0
  //   (Input Data Format is XRGB 8888 or ARGB 8888)
  // LAY_ALPHA_MODE (Bits 1 to 2) = 2
  //   (Global Alpha is mixed with Pixel Alpha)
  //   (Input Alpha Value = Global Alpha Value * Pixel’s Alpha Value)
  // LAY_EN (Bit 0) = 1 (Enable Layer)
  // (DE Page 102, 0x110 3000 / 0x110 4000 / 0x110 5000)
  debug("", .{  });
  const LAY_GLBALPHA: u32 = switch (channel) {  // For Global Alpha Value...
      1 => 0xFF,  // Channel 1: Opaque
      2 => 0xFF,  // Channel 2: Opaque
      3 => 0x7F,  // Channel 3: Semi-Transparent
      else => unreachable,
  } << 24;  // Bits 24 to 31

  const LAY_FBFMT: u13 = switch (channel) {  // For Input Data Format...
      1 => 4,  // Channel 1: XRGB 8888
      2 => 0,  // Channel 2: ARGB 8888
      3 => 0,  // Channel 3: ARGB 8888
      else => unreachable,
  } << 8;  // Bits 8 to 12

  const LAY_ALPHA_MODE: u3 = 2 << 1;  // Global Alpha is mixed with Pixel Alpha
  const LAY_EN:         u1 = 1 << 0;  // Enable Layer
  const attr = LAY_GLBALPHA
      | LAY_FBFMT
      | LAY_ALPHA_MODE
      | LAY_EN;
  comptime{ assert(attr == 0xFF000405 or attr == 0xFF000005 or attr == 0x7F000005); }

  const OVL_UI_ATTR_CTL = OVL_UI_BASE_ADDRESS + 0x00;
  comptime{ assert(OVL_UI_ATTR_CTL == 0x1103000 or OVL_UI_ATTR_CTL == 0x1104000 or OVL_UI_ATTR_CTL == 0x1105000); }
  putreg32(attr, OVL_UI_ATTR_CTL);

  // OVL_UI_TOP_LADD (UI Overlay Top Field Memory Block Low Address) at OVL_UI Offset 0x10
  // Set to Framebuffer Address: fb0, fb1 or fb2
  // (DE Page 104, 0x110 3010 / 0x110 4010 / 0x110 5010)
  const ptr = @ptrToInt(fbmem.?);
  const OVL_UI_TOP_LADD = OVL_UI_BASE_ADDRESS + 0x10;
  comptime{ assert(OVL_UI_TOP_LADD == 0x1103010 or OVL_UI_TOP_LADD == 0x1104010 or OVL_UI_TOP_LADD == 0x1105010); }
  putreg32(@intCast(u32, ptr), OVL_UI_TOP_LADD);

  // OVL_UI_PITCH (UI Overlay Memory Pitch) at OVL_UI Offset 0x0C
  // Set to (width * 4), number of bytes per row
  // (DE Page 104, 0x110 300C / 0x110 400C / 0x110 500C)
  const OVL_UI_PITCH = OVL_UI_BASE_ADDRESS + 0x0C;
  comptime{ assert(OVL_UI_PITCH == 0x110300C or OVL_UI_PITCH == 0x110400C or OVL_UI_PITCH == 0x110500C); }
  putreg32(xres * 4, OVL_UI_PITCH);

  // OVL_UI_MBSIZE (UI Overlay Memory Block Size) at OVL_UI Offset 0x04
  // Set to (height-1) << 16 + (width-1)
  // (DE Page 104, 0x110 3004 / 0x110 4004 / 0x110 5004)
  const height_width: u32 = @intCast(u32, yres - 1) << 16
      | (xres - 1);
  const OVL_UI_MBSIZE = OVL_UI_BASE_ADDRESS + 0x04;
  comptime{ assert(OVL_UI_MBSIZE == 0x1103004 or OVL_UI_MBSIZE == 0x1104004 or OVL_UI_MBSIZE == 0x1105004); }
  putreg32(height_width, OVL_UI_MBSIZE);

  // OVL_UI_SIZE (UI Overlay Overlay Window Size) at OVL_UI Offset 0x88
  // Set to (height-1) << 16 + (width-1)
  // (DE Page 106, 0x110 3088 / 0x110 4088 / 0x110 5088)
  const OVL_UI_SIZE = OVL_UI_BASE_ADDRESS + 0x88;
  comptime{ assert(OVL_UI_SIZE == 0x1103088 or OVL_UI_SIZE == 0x1104088 or OVL_UI_SIZE == 0x1105088); }
  putreg32(height_width, OVL_UI_SIZE);

  // OVL_UI_COOR (UI Overlay Memory Block Coordinate) at OVL_UI Offset 0x08
  // Set to 0 (Overlay at X=0, Y=0)
  // (DE Page 104, 0x110 3008 / 0x110 4008 / 0x110 5008)
  const OVL_UI_COOR = OVL_UI_BASE_ADDRESS + 0x08;
  comptime{ assert(OVL_UI_COOR == 0x1103008 or OVL_UI_COOR == 0x1104008 or OVL_UI_COOR == 0x1105008); }
  putreg32(0, OVL_UI_COOR);

  // For Channel 1: Set Blender Output
  if (channel == 1)
  {
    /* Set Blender Output ***************************************************/

    ginfo("Channel %d: Set Blender Output\n", channel);

    // BLD_SIZE (Blender Output Size Setting) at BLD Offset 0x08C
    // Set to (height-1) << 16 + (width-1)
    // (DE Page 110, 0x110 108C)
    const BLD_SIZE = BLD_BASE_ADDRESS + 0x08C;
    comptime{ assert(BLD_SIZE == 0x110108C); }
    putreg32(height_width, BLD_SIZE);
            
    // GLB_SIZE (Global Size) at GLB Offset 0x00C
    // Set to (height-1) << 16 + (width-1)
    // (DE Page 93, 0x110 000C)
    const GLB_SIZE = GLB_BASE_ADDRESS + 0x00C;
    comptime{ assert(GLB_SIZE == 0x110000C); }
    putreg32(height_width, GLB_SIZE);
  }

  /* Set Blender Input Pipe *************************************************/

  ginfo("Channel %d: Set Blender Input Pipe %d (%d x %d)\n", channel, pipe, xres, yres);

  // Set Blender Input Pipe (N = Pipe Number, from 0 to 2 for Channels 1 to 3)
  const pipe: u64 = channel - 1;
  debug("", .{  });

  // Note: DE Page 91 shows incorrect offset N*0x14 for 
  // BLD_CH_ISIZE, BLD_FILL_COLOR and BLD_CH_OFFSET. 
  // Correct offset is N*0x10, see DE Page 108

  // BLD_CH_ISIZE (Blender Input Memory Size) at BLD Offset 0x008 + N*0x10 (N=0,1,2,3,4)
  // Set to (height-1) << 16 + (width-1)
  // (DE Page 108, 0x110 1008 / 0x110 1018 / 0x110 1028)
  const BLD_CH_ISIZE = BLD_BASE_ADDRESS + 0x008 + pipe * 0x10;
  comptime{ assert(BLD_CH_ISIZE == 0x1101008 or BLD_CH_ISIZE == 0x1101018 or BLD_CH_ISIZE == 0x1101028); }
  putreg32(height_width, BLD_CH_ISIZE);

  // BLD_FILL_COLOR (Blender Fill Color) at BLD Offset 0x004 + N*0x10 (N=0,1,2,3,4)
  // Set to 0xFF00 0000 (Opaque Black)
  // ALPHA (Bits 24 to 31) = 0xFF
  // RED   (Bits 16 to 23) = 0
  // GREEN (Bits 8  to 15) = 0
  // BLUE  (Bits 0  to 7)  = 0
  // (DE Page 107, 0x110 1004 / 0x110 1014 / 0x110 1024)
  const ALPHA: u32 = 0xFF << 24;  // Opaque
  const RED:   u24 = 0    << 16;  // Black
  const GREEN: u18 = 0    << 8;
  const BLUE:  u8  = 0    << 0;
  const color = ALPHA
      | RED
      | GREEN
      | BLUE;
  comptime{ assert(color == 0xFF000000); }

  const BLD_FILL_COLOR = BLD_BASE_ADDRESS + 0x004 + pipe * 0x10;
  comptime{ assert(BLD_FILL_COLOR == 0x1101004 or BLD_FILL_COLOR == 0x1101014 or BLD_FILL_COLOR == 0x1101024); }
  putreg32(color, BLD_FILL_COLOR);

  // BLD_CH_OFFSET (Blender Input Memory Offset) at BLD Offset 0x00C + N*0x10 (N=0,1,2,3,4)
  // Set to y_offset << 16 + x_offset
  // For Channel 1: Set to 0
  // For Channel 2: Set to 0x34 0034
  // For Channel 3: Set to 0
  // (DE Page 108, 0x110 100C / 0x110 101C / 0x110 102C)
  const offset = @intCast(u32, yoffset) << 16
      | xoffset;
  comptime{ assert(offset == 0 or offset == 0x340034); }

  const BLD_CH_OFFSET = BLD_BASE_ADDRESS + 0x00C + pipe * 0x10;
  comptime{ assert(BLD_CH_OFFSET == 0x110100C or BLD_CH_OFFSET == 0x110101C or BLD_CH_OFFSET == 0x110102C); }
  putreg32(offset, BLD_CH_OFFSET);

  // BLD_CTL (Blender Control) at BLD Offset 0x090 + N*4
  // Set to 0x301 0301
  // BLEND_AFD (Bits 24 to 27) = 3
  //   (Coefficient for destination alpha data Q[d] is 1-A[s])
  // BLEND_AFS (Bits 16 to 19) = 1
  //   (Coefficient for source alpha data Q[s] is 1)
  // BLEND_PFD (Bits 8 to 11) = 3
  //   (Coefficient for destination pixel data F[d] is 1-A[s])
  // BLEND_PFS (Bits 0 to 3) = 1
  //   (Coefficient for source pixel data F[s] is 1)
  // (DE Page 110, 0x110 1090 / 0x110 1094 / 0x110 1098)
  const BLEND_AFD: u28 = 3 << 24;  // Coefficient for destination alpha data Q[d] is 1-A[s]
  const BLEND_AFS: u20 = 1 << 16;  // Coefficient for source alpha data Q[s] is 1
  const BLEND_PFD: u12 = 3 << 8;   // Coefficient for destination pixel data F[d] is 1-A[s]
  const BLEND_PFS: u4  = 1 << 0;   // Coefficient for source pixel data F[s] is 1
  const blend = BLEND_AFD
      | BLEND_AFS
      | BLEND_PFD
      | BLEND_PFS;

  const BLD_CTL = BLD_BASE_ADDRESS + 0x090 + pipe * 4;
  comptime{ assert(BLD_CTL == 0x1101090 or BLD_CTL == 0x1101094 or BLD_CTL == 0x1101098); }
  putreg32(blend, BLD_CTL);

  /* Disable Scaler *********************************************************/

  ginfo("Channel %d: Disable Scaler\n", channel);

  // Disable Scaler (Assume we’re not scaling)
  // UIS_CTRL_REG at Offset 0 of UI_SCALER1(CH1) or UI_SCALER2(CH2) or UI_SCALER3(CH3)
  // Set to 0 (Disable UI Scaler)
  // EN (Bit 0) = 0 (Disable UI Scaler)
  // (DE Page 66, 0x114 0000 / 0x115 0000 / 0x116 0000)
  debug("", .{  });
  const UIS_CTRL_REG = UI_SCALER_BASE_ADDRESS + 0;
  comptime{ assert(UIS_CTRL_REG == 0x1140000 or UIS_CTRL_REG == 0x1150000 or UIS_CTRL_REG == 0x1160000); }
  putreg32(0, UIS_CTRL_REG);

  return OK;
}

/// Set UI Blender Route, enable Blender Pipes and apply the settings for PinePhone's A64 Display Engine.
/// Must be called after a64_de_ui_channel_init
int a64_de_enable(
  uint8_t channels  // Number of enabled UI Channels
)
{
  comptime { assert(channels == 1 or channels == 3); }

  /* Set Blender Route ******************************************************/

  ginfo("Set Blender Route\n");

  // Set Blender Route
  // BLD_CH_RTCTL (Blender Routing Control) at BLD Offset 0x080
  // If Rendering 3 UI Channels: Set to 0x321 (DMB)
  //   P2_RTCTL (Bits 8 to 11) = 3 (Pipe 2 from Channel 3)
  //   P1_RTCTL (Bits 4 to 7)  = 2 (Pipe 1 from Channel 2)
  //   P0_RTCTL (Bits 0 to 3)  = 1 (Pipe 0 from Channel 1)
  // If Rendering 1 UI Channel: Set to 1 (DMB)
  //   P0_RTCTL (Bits 0 to 3) = 1 (Pipe 0 from Channel 1)
  // (DE Page 108, 0x110 1080)
  const P2_RTCTL: u12 = switch (channels) {  // For Pipe 2...
      3 => 3,  // 3 UI Channels: Select Pipe 2 from UI Channel 3
      1 => 0,  // 1 UI Channel:  Unused Pipe 2
      else => unreachable,
  } << 8;  // Bits 8 to 11

  const P1_RTCTL: u8 = switch (channels) {  // For Pipe 1...
      3 => 2,  // 3 UI Channels: Select Pipe 1 from UI Channel 2
      1 => 0,  // 1 UI Channel:  Unused Pipe 1
      else => unreachable,
  } << 4;  // Bits 4 to 7

  const P0_RTCTL: u4 = 1 << 0;  // Select Pipe 0 from UI Channel 1
  const route = P2_RTCTL
      | P1_RTCTL
      | P0_RTCTL;
  comptime{ assert(route == 0x321 or route == 1); }

  const BLD_CH_RTCTL = BLD_BASE_ADDRESS + 0x080;
  comptime{ assert(BLD_CH_RTCTL == 0x1101080); }
  putreg32(route, BLD_CH_RTCTL);  // TODO: DMB

  /* Enable Blender Pipes ***************************************************/

  ginfo("Enable Blender Pipes\n");

  // Enable Blender Pipes
  // BLD_FILL_COLOR_CTL (Blender Fill Color Control) at BLD Offset 0x000
  // If Rendering 3 UI Channels: Set to 0x701 (DMB)
  //   P2_EN   (Bit 10) = 1 (Enable Pipe 2)
  //   P1_EN   (Bit 9)  = 1 (Enable Pipe 1)
  //   P0_EN   (Bit 8)  = 1 (Enable Pipe 0)
  //   P0_FCEN (Bit 0)  = 1 (Enable Pipe 0 Fill Color)
  // If Rendering 1 UI Channel: Set to 0x101 (DMB)
  //   P0_EN   (Bit 8)  = 1 (Enable Pipe 0)
  //   P0_FCEN (Bit 0)  = 1 (Enable Pipe 0 Fill Color)
  // (DE Page 106, 0x110 1000)
  const P2_EN: u11 = switch (channels) {  // For Pipe 2...
      3 => 1,  // 3 UI Channels: Enable Pipe 2
      1 => 0,  // 1 UI Channel:  Disable Pipe 2
      else => unreachable,
  } << 10;  // Bit 10

  const P1_EN: u10 = switch (channels) {  // For Pipe 1...
      3 => 1,  // 3 UI Channels: Enable Pipe 1
      1 => 0,  // 1 UI Channel:  Disable Pipe 1
      else => unreachable,
  } << 9;  // Bit 9

  const P0_EN:   u9 = 1 << 8;  // Enable Pipe 0
  const P0_FCEN: u1 = 1 << 0;  // Enable Pipe 0 Fill Color
  const fill = P2_EN
      | P1_EN
      | P0_EN
      | P0_FCEN;
  comptime{ assert(fill == 0x701 or fill == 0x101); }

  const BLD_FILL_COLOR_CTL = BLD_BASE_ADDRESS + 0x000;
  comptime{ assert(BLD_FILL_COLOR_CTL == 0x1101000); }
  putreg32(fill, BLD_FILL_COLOR_CTL);  // TODO: DMB

  /* Apply Settings *********************************************************/

  ginfo("Apply Settings\n");

  // Apply Settings
  // GLB_DBUFFER (Global Double Buffer Control) at GLB Offset 0x008
  // Set to 1 (DMB)
  // DOUBLE_BUFFER_RDY (Bit 0) = 1
  // (Register Value is ready for update)
  // (DE Page 93, 0x110 0008)
  
  const DOUBLE_BUFFER_RDY: u1 = 1 << 0;  // Register Value is ready for update
  comptime{ assert(DOUBLE_BUFFER_RDY == 1); }

  const GLB_DBUFFER = GLB_BASE_ADDRESS + 0x008;
  comptime{ assert(GLB_DBUFFER == 0x1100008); }
  putreg32(DOUBLE_BUFFER_RDY, GLB_DBUFFER);  // TODO: DMB

  return OK;
}
