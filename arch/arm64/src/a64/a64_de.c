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
#ifdef TODO

    // Set High Speed SRAM to DMA Mode
    // Set BIST_DMA_CTRL_SEL to 0 for DMA (DMB) (A31 Page 191, 0x1C0 0004)
    // BIST_DMA_CTRL_SEL (Bist and DMA Control Select) is Bit 0 of SRAM_CTRL_REG1
    // SRAM_CTRL_REG1 (SRAM Control Register 1) is at SRAM Registers Offset 0x4
    debug("Set High Speed SRAM to DMA Mode", .{});
    const SRAM_CTRL_REG1 = SRAM_REGISTERS_BASE_ADDRESS + 0x4;
    comptime{ assert(SRAM_CTRL_REG1 == 0x1C0_0004); }
    putreg32(0x0, SRAM_CTRL_REG1);  // TODO: DMB

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
    debug("Set Display Engine PLL to 297 MHz", .{});
    const PLL_ENABLE:   u32 = 1  << 31;  // Enable PLL
    const PLL_MODE_SEL: u25 = 1  << 24;  // Integer Mode
    const PLL_FACTOR_N: u15 = 23 <<  8;  // N = 24
    const PLL_PRE_DIV_M: u4 = 1  <<  0;  // M = 2
    const pll = PLL_ENABLE
        | PLL_MODE_SEL
        | PLL_FACTOR_N
        | PLL_PRE_DIV_M;
    comptime{ assert(pll == 0x8100_1701); }

    const PLL_DE_CTRL_REG = CCU_BASE_ADDRESS + 0x0048;
    comptime{ assert(PLL_DE_CTRL_REG == 0x1C2_0048); }
    putreg32(pll, PLL_DE_CTRL_REG);  // TODO: DMB

    // Wait for Display Engine PLL to be stable
    // Poll PLL_DE_CTRL_REG (from above) until LOCK (Bit 28) is 1
    // (PLL is Locked and Stable)
    debug("Wait for Display Engine PLL to be stable", .{});
    while (getreg32(PLL_DE_CTRL_REG) & (1 << 28) == 0) {}

    // Set Special Clock to Display Engine PLL
    // Clear DE_CLK_REG bits 0x0300 0000
    // Set DE_CLK_REG bits 0x8100 0000
    // SCLK_GATING (Bit 31)        = 1 (Enable Special Clock)
    // CLK_SRC_SEL (Bits 24 to 26) = 1 (Clock Source is Display Engine PLL)
    // DE_CLK_REG (Display Engine Clock Register) is at CCU Offset 0x0104
    // (A64 Page 117, 0x1C2 0104)
    debug("Set Special Clock to Display Engine PLL", .{});
    const SCLK_GATING: u32 = 1 << 31;  // Enable Special Clock
    const CLK_SRC_SEL: u27 = 1 << 24;  // Clock Source is Display Engine PLL
    const clk = SCLK_GATING
        | CLK_SRC_SEL;
    comptime{ assert(clk == 0x8100_0000); }

    const SCLK_GATING_MASK: u32 = 0b1   << 31;
    const CLK_SRC_SEL_MASK: u27 = 0b111 << 24;
    const clk_mask = SCLK_GATING_MASK
        | CLK_SRC_SEL_MASK;

    const DE_CLK_REG = CCU_BASE_ADDRESS + 0x0104;
    comptime{ assert(DE_CLK_REG == 0x1C2_0104); }
    modreg32(clk, clk_mask, DE_CLK_REG);

    // Enable AHB (AMBA High-speed Bus) for Display Engine: De-Assert Display Engine
    // Set BUS_SOFT_RST_REG1 bits 0x1000
    // DE_RST (Bit 12) = 1 (De-Assert Display Engine)
    // BUS_SOFT_RST_REG1 (Bus Software Reset Register 1) is at CCU Offset 0x02C4
    // (A64 Page 140, 0x1C2 02C4)
    debug("Enable AHB for Display Engine: De-Assert Display Engine", .{});
    const DE_RST: u13 = 1 << 12;  // De-Assert Display Engine
    const BUS_SOFT_RST_REG1 = CCU_BASE_ADDRESS + 0x02C4;
    comptime{ assert(BUS_SOFT_RST_REG1 == 0x1C2_02C4); }
    modreg32(DE_RST, DE_RST, BUS_SOFT_RST_REG1);

    // Enable AHB (AMBA High-speed Bus) for Display Engine: Pass Display Engine
    // Set BUS_CLK_GATING_REG1 bits 0x1000
    // DE_GATING (Bit 12) = 1 (Pass Display Engine)
    // BUS_CLK_GATING_REG1 (Bus Clock Gating Register 1) is at CCU Offset 0x0064
    // (A64 Page 102, 0x1C2 0064)
    debug("Enable AHB for Display Engine: Pass Display Engine", .{});
    const DE_GATING: u13 = 1 << 12;  // Pass Display Engine
    const BUS_CLK_GATING_REG1 = CCU_BASE_ADDRESS + 0x0064;
    comptime{ assert(BUS_CLK_GATING_REG1 == 0x1C2_0064); }
    modreg32(DE_GATING, DE_GATING, BUS_CLK_GATING_REG1);

    // Enable Clock for MIXER0: SCLK Clock Pass
    // Set SCLK_GATE bits 0x1
    // CORE0_SCLK_GATE (Bit 0) = 1 (Clock Pass)
    // SCLK_GATE is at DE Offset 0x000
    // (DE Page 25, 0x100 0000)
    debug("Enable Clock for MIXER0: SCLK Clock Pass", .{});
    const CORE0_SCLK_GATE: u1 = 1 << 0;  // Clock Pass
    const SCLK_GATE = DISPLAY_ENGINE_BASE_ADDRESS + 0x000;
    comptime{ assert(SCLK_GATE == 0x100_0000); }
    modreg32(CORE0_SCLK_GATE, CORE0_SCLK_GATE, SCLK_GATE);

    // Enable Clock for MIXER0: HCLK Clock Reset Off
    // Set AHB_RESET bits 0x1
    // CORE0_HCLK_RESET (Bit 0) = 1 (Reset Off)
    // AHB_RESET is at DE Offset 0x008
    // (DE Page 25, 0x100 0008)
    debug("Enable Clock for MIXER0: HCLK Clock Reset Off", .{});
    const CORE0_HCLK_RESET: u1 = 1 << 0;  // Reset Off
    const AHB_RESET = DISPLAY_ENGINE_BASE_ADDRESS + 0x008;
    comptime{ assert(AHB_RESET == 0x100_0008); }
    modreg32(CORE0_HCLK_RESET, CORE0_HCLK_RESET, AHB_RESET);

    // Enable Clock for MIXER0: HCLK Clock Pass
    // Set HCLK_GATE bits 0x1
    // CORE0_HCLK_GATE (Bit 0) = 1 (Clock Pass)
    // HCLK_GATE is at DE Offset 0x004
    // (DE Page 25, 0x100 0004)
    debug("Enable Clock for MIXER0: HCLK Clock Pass", .{});
    const CORE0_HCLK_GATE: u1 = 1 << 0;  // Clock Pass
    const HCLK_GATE = DISPLAY_ENGINE_BASE_ADDRESS + 0x004;
    comptime{ assert(HCLK_GATE == 0x100_0004); }
    modreg32(CORE0_HCLK_GATE, CORE0_HCLK_GATE, HCLK_GATE);

    // Route MIXER0 to TCON0
    // Clear DE2TCON_MUX bits 0x1
    // DE2TCON_MUX (Bit 0) = 0
    // (Route MIXER0 to TCON0; Route MIXER1 to TCON1)
    // DE2TCON_MUX is at DE Offset 0x010
    // (DE Page 26, 0x100 0010)
    debug("Route MIXER0 to TCON0", .{});
    const DE2TCON_MUX_MASK: u1 = 1 << 0;  // Route MIXER0 to TCON0; Route MIXER1 to TCON1
    const DE2TCON_MUX = DISPLAY_ENGINE_BASE_ADDRESS + 0x010;
    comptime{ assert(DE2TCON_MUX == 0x100_0010); }
    modreg32(0, DE2TCON_MUX_MASK, DE2TCON_MUX);

    // Clear MIXER0 Registers: Global Registers (GLB), Blender (BLD), Video Overlay (OVL_V), UI Overlay (OVL_UI)
    // Set MIXER0 Offsets 0x0000 - 0x5FFF to 0
    // GLB (Global Regisers) at MIXER0 Offset 0x0000
    // BLD (Blender) at MIXER0 Offset 0x1000
    // OVL_V(CH0) (Video Overlay) at MIXER0 Offset 0x2000
    // OVL_UI(CH1) (UI Overlay 1) at MIXER0 Offset 0x3000
    // OVL_UI(CH2) (UI Overlay 2) at MIXER0 Offset 0x4000
    // OVL_UI(CH3) (UI Overlay 3) at MIXER0 Offset 0x5000
    // (DE Page 90, 0x110 0000 - 0x110 5FFF)
    debug("Clear MIXER0 Registers: GLB, BLD, OVL_V, OVL_UI", .{});
    var i: usize = 0;
    while (i < 0x6000) : (i += 4) {
        putreg32(0, MIXER0_BASE_ADDRESS + i);
        enableLog = false;
    }
    enableLog = true;
    debug("  to *0x{x} = 0x0", .{ MIXER0_BASE_ADDRESS + i - 1 });

    // Disable MIXER0 Video Scaler (VSU)
    // Set to 0: VS_CTRL_REG at VIDEO_SCALER(CH0) Offset 0
    // EN (Bit 0) = 0 (Disable Video Scaler)
    // (DE Page 130, 0x112 0000)
    debug("Disable MIXER0 VSU", .{});
    const VS_CTRL_REG = VIDEO_SCALER_BASE_ADDRESS + 0;
    comptime{ assert(VS_CTRL_REG == 0x112_0000); }
    putreg32(0, VS_CTRL_REG);

    // TODO: 0x113 0000 is undocumented
    // Is there a mixup with UI_SCALER3?
    debug("Disable MIXER0 Undocumented", .{});
    const _1130000 = 0x1130000;
    putreg32(0, _1130000);

    // Disable MIXER0 UI_SCALER1
    // Set to 0: UIS_CTRL_REG at UI_SCALER1(CH1) Offset 0
    // EN (Bit 0) = 0 (Disable UI Scaler)
    // (DE Page 66, 0x114 0000)
    debug("Disable MIXER0 UI_SCALER1", .{});
    const UIS_CTRL_REG1 = UI_SCALER1_BASE_ADDRESS + 0;
    comptime{ assert(UIS_CTRL_REG1 == 0x114_0000); }
    putreg32(0, UIS_CTRL_REG1);

    // Disable MIXER0 UI_SCALER2
    // Set to 0: UIS_CTRL_REG at UI_SCALER2(CH2) Offset 0
    // EN (Bit 0) = 0 (Disable UI Scaler)
    // (DE Page 66, 0x115 0000)
    debug("Disable MIXER0 UI_SCALER2", .{});
    const UIS_CTRL_REG2 = UI_SCALER2_BASE_ADDRESS + 0;
    comptime{ assert(UIS_CTRL_REG2 == 0x115_0000); }
    putreg32(0, UIS_CTRL_REG2);

    // TODO: Missing UI_SCALER3(CH3) at MIXER0 Offset 0x06 0000 (DE Page 90, 0x116 0000)
    // Is there a mixup with 0x113 0000 above?

    // Disable MIXER0 FCE
    // Set to 0: GCTRL_REG(FCE) at FCE Offset 0
    // EN (Bit 0) = 0 (Disable FCE)
    // (DE Page 62, 0x11A 0000)
    debug("Disable MIXER0 FCE", .{});
    const GCTRL_REG_FCE = FCE_BASE_ADDRESS + 0;
    comptime{ assert(GCTRL_REG_FCE == 0x11A_0000); }
    putreg32(0, GCTRL_REG_FCE);

    // Disable MIXER0 BWS
    // Set to 0: GCTRL_REG(BWS) at BWS Offset 0
    // EN (Bit 0) = 0 (Disable BWS)
    // (DE Page 42, 0x11A 2000)
    debug("Disable MIXER0 BWS", .{});
    const GCTRL_REG_BWS = BWS_BASE_ADDRESS + 0;
    comptime{ assert(GCTRL_REG_BWS == 0x11A_2000); }
    putreg32(0, GCTRL_REG_BWS);

    // Disable MIXER0 LTI
    // Set to 0: LTI_CTL at LTI Offset 0
    // LTI_EN (Bit 0) = 0 (Close LTI)
    // (DE Page 72, 0x11A 4000)
    debug("Disable MIXER0 LTI", .{});
    const LTI_CTL = LTI_BASE_ADDRESS + 0;
    comptime{ assert(LTI_CTL == 0x11A_4000); }
    putreg32(0, LTI_CTL);

    // Disable MIXER0 PEAKING
    // Set to 0: LP_CTRL_REG at PEAKING Offset 0
    // EN (Bit 0) = 0 (Disable PEAKING)
    // (DE Page 80, 0x11A 6000)
    debug("Disable MIXER0 PEAKING", .{});
    const LP_CTRL_REG = PEAKING_BASE_ADDRESS + 0;
    comptime{ assert(LP_CTRL_REG == 0x11A_6000); }
    putreg32(0, LP_CTRL_REG);

    // Disable MIXER0 ASE
    // Set to 0: ASE_CTL_REG at ASE Offset 0
    // ASE_EN (Bit 0) = 0 (Disable ASE)
    // (DE Page 40, 0x11A 8000)
    debug("Disable MIXER0 ASE", .{});
    const ASE_CTL_REG = ASE_BASE_ADDRESS + 0;
    comptime{ assert(ASE_CTL_REG == 0x11A_8000); }
    putreg32(0, ASE_CTL_REG);

    // Disable MIXER0 FCC
    // Set to 0: FCC_CTL_REG at FCC Offset 0
    // Enable (Bit 0) = 0 (Disable FCC)
    // (DE Page 56, 0x11A A000)
    debug("Disable MIXER0 FCC", .{});
    const FCC_CTL_REG = FCC_BASE_ADDRESS + 0;
    comptime{ assert(FCC_CTL_REG == 0x11A_A000); }
    putreg32(0, FCC_CTL_REG);

    // Disable MIXER0 DRC
    // Set to 0: GNECTL_REG at DRC Offset 0
    // BIST_EN (Bit 0) = 0 (Disable BIST)
    // (DE Page 49, 0x11B 0000)
    debug("Disable MIXER0 DRC", .{});
    const GNECTL_REG = DRC_BASE_ADDRESS + 0;
    comptime{ assert(GNECTL_REG == 0x11B_0000); }
    putreg32(0, GNECTL_REG);

    // Enable MIXER0
    // Set GLB_CTL to 1 (DMB)
    // EN (Bit 0) = 1 (Enable Mixer)
    // (DE Page 92)
    // GLB_CTL is at MIXER0 Offset 0
    // (DE Page 90, 0x110 0000)
    debug("Enable MIXER0", .{});
    const EN_MIXER: u1 = 1 << 0;  // Enable Mixer
    const GLB_CTL = MIXER0_BASE_ADDRESS + 0;
    comptime{ assert(GLB_CTL == 0x110_0000); }
    putreg32(EN_MIXER, GLB_CTL);  // TODO: DMB

#endif
  return OK;
}

/// Initialize the UI Blender for PinePhone's A64 Display Engine.
/// Must be called after a64_de_init, and before a64_de_ui_channel_init
int a64_de_blender_init(void)
{
#ifdef TODO

    // Set Blender Background
    // BLD_BK_COLOR (Blender Background Color) at BLD Offset 0x88
    // Set to 0xFF00 0000 (Black Background Color)
    // RESERVED (Bits 24 to 31) = 0xFF (Undocumented)
    // RED   (Bits 16 to 23) = 0
    // GREEN (Bits 8  to 15) = 0
    // BLUE  (Bits 0  to 7)  = 0
    // (DE Page 109, 0x110 1088)
    debug("Set Blender Background", .{});
    const RESERVED: u32 = 0xFF << 24;
    const RED:      u24 = 0    << 16;
    const GREEN:    u16 = 0    << 8;
    const BLUE:     u8  = 0    << 0;
    const color = RESERVED
        | RED
        | GREEN
        | BLUE;
    comptime{ assert(color == 0xFF00_0000); }

    const BLD_BK_COLOR = BLD_BASE_ADDRESS + 0x88;
    comptime{ assert(BLD_BK_COLOR == 0x110_1088); }
    putreg32(color, BLD_BK_COLOR);

    // Set Blender Pre-Multiply
    // BLD_PREMUL_CTL (Blender Pre-Multiply Control) at BLD Offset 0x84
    // Set to 0 (No Pre-Multiply for Alpha, Pipes 0 to 3)
    // P3_ALPHA_MODE (Bit 3) = 0 (Pipe 3: No Pre-Multiply)
    // P2_ALPHA_MODE (Bit 2) = 0 (Pipe 2: No Pre-Multiply)
    // P1_ALPHA_MODE (Bit 1) = 0 (Pipe 1: No Pre-Multiply)
    // P0_ALPHA_MODE (Bit 0) = 0 (Pipe 0: No Pre-Multiply)
    // (DE Page 109, 0x110 1084)
    debug("Set Blender Pre-Multiply", .{});
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
    comptime{ assert(BLD_PREMUL_CTL == 0x110_1084); }
    putreg32(premultiply, BLD_PREMUL_CTL);

#endif
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
#ifdef TODO
#endif
  return OK;
}

/// Set UI Blender Route, enable Blender Pipes and apply the settings for PinePhone's A64 Display Engine.
/// Must be called after a64_de_ui_channel_init
int a64_de_enable(
  uint8_t channels  // Number of enabled UI Channels
)
{
#ifdef TODO
#endif
  return OK;
}
