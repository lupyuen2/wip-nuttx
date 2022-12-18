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
 * "A31 Page" refers to Allwinner A31 User Manual
 * https://lupyuen.github.io/images/A31_User_Manual_v1.3_20150510.pdf
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

/* Timeout for Display Engine PLL in milliseconds */

#define PLL_TIMEOUT_MS      5

// Base Addresses

/// Mixer 0 Base (DE Page 24)
#define A64_MIXER0_ADDR (A64_DE_ADDR + 0x100000)

/// Global Registers (DE Page 90)
#define A64_GLB_ADDR (A64_MIXER0_ADDR + 0x0000)

/// Blender (DE Page 90)
#define A64_BLD_ADDR (A64_MIXER0_ADDR + 0x1000)

/// UI Overlay 1 (DE Page 102)
#define A64_OVL_UI_CH1_ADDR (A64_MIXER0_ADDR + 0x3000)

// SRAM Registers (A31 Page 191)
#define A64_SRAM_REG_ADDR 0x01C00000

// Dynamic Range Controller (DE Page 48)
#define A64_DRC_ADDR 0x011B0000

// Video Scaler (DE Page 90)
#define A64_VIDEO_SCALER_ADDR (A64_MIXER0_ADDR + 0x020000)

// UI Scaler 1 (DE Page 90)
#define A64_UI_SCALER1_ADDR (A64_MIXER0_ADDR + 0x040000)

// UI Scaler 2 (DE Page 90)
#define A64_UI_SCALER2_ADDR (A64_MIXER0_ADDR + 0x050000)

// Fresh and Contrast Enhancement (DE Page 61)
#define A64_FCE_ADDR (A64_MIXER0_ADDR + 0x0A0000)

// Black and White Stretch (DE Page 42)
#define A64_BWS_ADDR (A64_MIXER0_ADDR + 0x0A2000)

// Luminance Transient Improvement (DE Page 71)
#define A64_LTI_ADDR (A64_MIXER0_ADDR + 0x0A4000)

// Luma Peaking (DE Page 80)
#define A64_PEAKING_ADDR (A64_MIXER0_ADDR + 0x0A6000)

// Adaptive Saturation Enhancement (DE Page 40)
#define A64_ASE_ADDR (A64_MIXER0_ADDR + 0x0A8000)

// Fancy Color Curvature Change (DE Page 56)
#define A64_FCC_ADDR (A64_MIXER0_ADDR + 0x0AA000)

// PLL Display Engine Control Register (A64 Page 96)
#define PLL_DE_CTRL_REG (A64_CCU_ADDR + 0x0048)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a64_wait_pll
 *
 * Description:
 *   Wait for Display Engine PLL to be stable.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if timeout.
 *
 ****************************************************************************/

static int a64_wait_pll(void)
{
  int i;

  for (i = 0; i < PLL_TIMEOUT_MS; i++)
    {
      /* Poll on LOCK (Bit 28) of PLL Display Engine Control Register */

      if ((getreg32(PLL_DE_CTRL_REG) & (1 << 28)) != 0)
        {
          /* If LOCK is 1, then Display Engine PLL is stable */

          return OK;
        }

      /* Sleep 1 millisecond and try again */

      up_mdelay(1);
    }

  gerr("PLL Timeout");
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

// Init PinePhone's Allwinner A64 Display Engine.
// Called by display_init() in p-boot Display Code.
// Must be called before any DE operations
int a64_de_init(void)
{
  int i;

  /* Set High Speed SRAM to DMA Mode ****************************************/

  ginfo("Set High Speed SRAM to DMA Mode\n");

  // SRAM Control Register 1 (A31 Page 191)
  // Set BIST_DMA_CTRL_SEL (Bit 31) to 0 (DMA)
  #define SRAM_CTRL_REG1 (A64_SRAM_REG_ADDR + 0x4)
  DEBUGASSERT(SRAM_CTRL_REG1 == 0x1C00004);
  putreg32(0x0, SRAM_CTRL_REG1);

  /* Set Display Engine PLL to 297 MHz **************************************/

  ginfo("Set Display Engine PLL to 297 MHz\n");

  // PLL Display Engine Control Register (A64 Page 96)
  // Set PLL_ENABLE (Bit 31) to 1 (Enable PLL)
  // Set PLL_MODE_SEL (Bit 24) to 1 (Integer Mode)
  // Set PLL_FACTOR_N (Bits 8 to 14) to 23 (N = 24)
  // Set PLL_PRE_DIV_M (Bits 0 to 3) to 1 (M = 2)
  // Actual PLL Output = 24 MHz * N / M = 288 MHz
  // (Slighltly below 297 MHz due to truncation)
  #define PLL_ENABLE (1  << 31)
  #define PLL_MODE_SEL (1  << 24)
  #define PLL_FACTOR_N (23 <<  8)
  #define PLL_PRE_DIV_M (1  <<  0)
  uint32_t pll;
  pll = PLL_ENABLE
      | PLL_MODE_SEL
      | PLL_FACTOR_N
      | PLL_PRE_DIV_M;
  DEBUGASSERT(pll == 0x81001701);

  DEBUGASSERT(PLL_DE_CTRL_REG == 0x1C20048);
  putreg32(pll, PLL_DE_CTRL_REG);

  /* Wait for Display Engine PLL to be stable *******************************/

  ginfo("Wait for Display Engine PLL to be stable\n");

  int ret;
  ret = a64_wait_pll();
  if (ret < 0)
    {
      return ret;
    }

  /* Set Special Clock to Display Engine PLL ********************************/

  ginfo("Set Special Clock to Display Engine PLL\n");

  // DE_CLK_REG (Display Engine Clock Register) is at CCU Offset 0x0104
  // (A64 Page 117, 0x1C2 0104)
  // Clear DE_CLK_REG bits 0x0300 0000
  // Set DE_CLK_REG bits 0x8100 0000
  // Set SCLK_GATING (Bit 31)        = 1 (Enable Special Clock)
  // Set CLK_SRC_SEL (Bits 24 to 26) = 1 (Clock Source is Display Engine PLL)

  #define SCLK_GATING (1 << 31)
  #define CLK_SRC_SEL (1 << 24)
  uint32_t clk;
  clk = SCLK_GATING
      | CLK_SRC_SEL;
  DEBUGASSERT(clk == 0x81000000);

  #define SCLK_GATING_MASK (0b1   << 31)
  #define CLK_SRC_SEL_MASK (0b111 << 24)
  uint32_t clk_mask;
  clk_mask = SCLK_GATING_MASK
      | CLK_SRC_SEL_MASK;

  #define DE_CLK_REG (A64_CCU_ADDR + 0x0104)
  DEBUGASSERT(DE_CLK_REG == 0x1C20104);
  modreg32(clk, clk_mask, DE_CLK_REG);

  /* Enable AHB for Display Engine: De-Assert Display Engine ****************/

  ginfo("Enable AHB for Display Engine: De-Assert Display Engine\n");

  // BUS_SOFT_RST_REG1 (Bus Software Reset Register 1) is at CCU Offset 0x02C4
  // (A64 Page 140, 0x1C2 02C4)
  // Set BUS_SOFT_RST_REG1 bits 0x1000
  // Set DE_RST (Bit 12) = 1 (De-Assert Display Engine)

  #define DE_RST (1 << 12)
  #define BUS_SOFT_RST_REG1 (A64_CCU_ADDR + 0x02C4)
  DEBUGASSERT(BUS_SOFT_RST_REG1 == 0x1C202C4);
  modreg32(DE_RST, DE_RST, BUS_SOFT_RST_REG1);

  /* Enable AHB for Display Engine: Pass Display Engine *********************/

  ginfo("Enable AHB for Display Engine: Pass Display Engine\n");

  // BUS_CLK_GATING_REG1 (Bus Clock Gating Register 1) is at CCU Offset 0x0064
  // (A64 Page 102, 0x1C2 0064)
  // Set BUS_CLK_GATING_REG1 bits 0x1000
  // Set DE_GATING (Bit 12) = 1 (Pass Display Engine)

  #define DE_GATING (1 << 12)
  #define BUS_CLK_GATING_REG1 (A64_CCU_ADDR + 0x0064)
  DEBUGASSERT(BUS_CLK_GATING_REG1 == 0x1C20064);
  modreg32(DE_GATING, DE_GATING, BUS_CLK_GATING_REG1);

  /* Enable Clock for MIXER0: SCLK Clock Pass *******************************/

  ginfo("Enable Clock for MIXER0: SCLK Clock Pass\n");

  // SCLK_GATE is at DE Offset 0x000
  // (DE Page 25, 0x100 0000)
  // Set SCLK_GATE bits 0x1
  // Set CORE0_SCLK_GATE (Bit 0) = 1 (Clock Pass)

  #define CORE0_SCLK_GATE (1 << 0)
  #define SCLK_GATE (A64_DE_ADDR + 0x000)
  DEBUGASSERT(SCLK_GATE == 0x1000000);
  modreg32(CORE0_SCLK_GATE, CORE0_SCLK_GATE, SCLK_GATE);

  /* Enable Clock for MIXER0: HCLK Clock Reset Off **************************/

  ginfo("Enable Clock for MIXER0: HCLK Clock Reset Off\n");

  // AHB_RESET is at DE Offset 0x008
  // (DE Page 25, 0x100 0008)
  // Set AHB_RESET bits 0x1
  // Set CORE0_HCLK_RESET (Bit 0) = 1 (Reset Off)

  #define CORE0_HCLK_RESET (1 << 0)
  #define AHB_RESET (A64_DE_ADDR + 0x008)
  DEBUGASSERT(AHB_RESET == 0x1000008);
  modreg32(CORE0_HCLK_RESET, CORE0_HCLK_RESET, AHB_RESET);

  /* Enable Clock for MIXER0: HCLK Clock Pass *******************************/

  ginfo("Enable Clock for MIXER0: HCLK Clock Pass\n");

  // HCLK_GATE is at DE Offset 0x004
  // (DE Page 25, 0x100 0004)
  // Set HCLK_GATE bits 0x1
  // Set CORE0_HCLK_GATE (Bit 0) = 1 (Clock Pass)

  #define CORE0_HCLK_GATE (1 << 0)
  #define HCLK_GATE (A64_DE_ADDR + 0x004)
  DEBUGASSERT(HCLK_GATE == 0x1000004);
  modreg32(CORE0_HCLK_GATE, CORE0_HCLK_GATE, HCLK_GATE);

  /* Route MIXER0 to TCON0 **************************************************/

  ginfo("Route MIXER0 to TCON0\n");

  // DE2TCON_MUX is at DE Offset 0x010
  // (DE Page 26, 0x100 0010)
  // Route MIXER0 to TCON0
  // Clear DE2TCON_MUX bits 0x1
  // Set DE2TCON_MUX (Bit 0) = 0
  // (Route MIXER0 to TCON0; Route MIXER1 to TCON1)

  #define DE2TCON_MUX_MASK (1 << 0)
  #define DE2TCON_MUX (A64_DE_ADDR + 0x010)
  DEBUGASSERT(DE2TCON_MUX == 0x1000010);
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

  for (i = 0; i < 0x6000; i += 4)
  {
    putreg32(0, A64_MIXER0_ADDR + i);
  }

  /* Disable MIXER0 VSU *****************************************************/

  ginfo("Disable MIXER0 VSU\n");

  // Video Scaler Control register (DE Page 130)
  // Set EN (Bit 0) = 0 (Disable Video Scaler)

  #define VS_CTRL_REG (A64_VIDEO_SCALER_ADDR + 0)
  DEBUGASSERT(VS_CTRL_REG == 0x1120000);
  putreg32(0, VS_CTRL_REG);

  /* Disable MIXER0 Undocumented ********************************************/

  ginfo("Disable MIXER0 Undocumented\n");

  // TODO: 0x113 0000 is undocumented
  // Is there a mixup with UI_SCALER3?

  #define _1130000 0x1130000
  putreg32(0, _1130000);

  /* Disable MIXER0 UI_SCALER1 **********************************************/

  ginfo("Disable MIXER0 UI_SCALER1\n");

  // UI Scaler 1 Control Register (DE Page 66)
  // Set EN (Bit 0) = 0 (Disable UI Scaler)

  #define UIS_CTRL_REG1 (A64_UI_SCALER1_ADDR + 0)
  DEBUGASSERT(UIS_CTRL_REG1 == 0x1140000);
  putreg32(0, UIS_CTRL_REG1);

  /* Disable MIXER0 UI_SCALER2 **********************************************/

  ginfo("Disable MIXER0 UI_SCALER2\n");

  // UI Scaler 2 Control Register (DE Page 66)
  // Set EN (Bit 0) = 0 (Disable UI Scaler)

  #define UIS_CTRL_REG2 (A64_UI_SCALER2_ADDR + 0)
  DEBUGASSERT(UIS_CTRL_REG2 == 0x1150000);
  putreg32(0, UIS_CTRL_REG2);

  // TODO: Missing UI_SCALER3(CH3) at MIXER0 Offset 0x06 0000 (DE Page 90, 0x116 0000)
  // Is there a mixup with 0x113 0000 above?

  /* Disable MIXER0 FCE *****************************************************/

  ginfo("Disable MIXER0 FCE\n");

  // Fresh and Contrast Enhancement Global Control Register (DE Page 61)
  // Set EN (Bit 0) = 0 (Disable FCE)

  #define GCTRL_REG_FCE (A64_FCE_ADDR + 0)
  DEBUGASSERT(GCTRL_REG_FCE == 0x11A0000);
  putreg32(0, GCTRL_REG_FCE);

  /* Disable MIXER0 BWS *****************************************************/

  ginfo("Disable MIXER0 BWS\n");

  // Black and White Stretch Global Control Register (DE Page 42)
  // Set EN (Bit 0) = 0 (Disable BWS)

  #define GCTRL_REG_BWS (A64_BWS_ADDR + 0)
  DEBUGASSERT(GCTRL_REG_BWS == 0x11A2000);
  putreg32(0, GCTRL_REG_BWS);

  /* Disable MIXER0 LTI *****************************************************/

  ginfo("Disable MIXER0 LTI\n");

  // Luminance Transient Improvement Global Control Register (DE Page 72)
  // Set LTI_EN (Bit 0) = 0 (Close LTI)

  #define LTI_CTL (A64_LTI_ADDR + 0)
  DEBUGASSERT(LTI_CTL == 0x11A4000);
  putreg32(0, LTI_CTL);

  /* Disable MIXER0 PEAKING *************************************************/

  ginfo("Disable MIXER0 PEAKING\n");

  // Luma Peaking Module Control Register (DE Page 80)
  // Set EN (Bit 0) = 0 (Disable PEAKING)

  #define LP_CTRL_REG (A64_PEAKING_ADDR + 0)
  DEBUGASSERT(LP_CTRL_REG == 0x11A6000);
  putreg32(0, LP_CTRL_REG);

  /* Disable MIXER0 ASE *****************************************************/

  ginfo("Disable MIXER0 ASE\n");

  // Adaptive Saturation Enhancement Global Control Register (DE Page 40)
  // Set ASE_EN (Bit 0) = 0 (Disable ASE)

  #define ASE_CTL_REG (A64_ASE_ADDR + 0)
  DEBUGASSERT(ASE_CTL_REG == 0x11A8000);
  putreg32(0, ASE_CTL_REG);

  /* Disable MIXER0 FCC *****************************************************/

  ginfo("Disable MIXER0 FCC\n");

  // Fancy Color Curvature Change Control Register (DE Page 56)
  // Set Enable (Bit 0) = 0 (Disable FCC)

  #define FCC_CTL_REG (A64_FCC_ADDR + 0)
  DEBUGASSERT(FCC_CTL_REG == 0x11AA000);
  putreg32(0, FCC_CTL_REG);

  /* Disable MIXER0 DRC *****************************************************/

  ginfo("Disable MIXER0 DRC\n");

  // Dynamic Range Controller Module General Control Register (DE Page 49)
  // Set BIST_EN (Bit 0) = 0 (Disable BIST)

  #define GNECTL_REG (A64_DRC_ADDR + 0)
  DEBUGASSERT(GNECTL_REG == 0x11B0000);
  putreg32(0, GNECTL_REG);

  /* Enable MIXER0 **********************************************************/

  ginfo("Enable MIXER0\n");

  // Mixer 0 Global Control Register (DE Page 92)
  // Set EN (Bit 0) = 1 (Enable Mixer)

  #define EN_MIXER (1 << 0)
  #define GLB_CTL (A64_MIXER0_ADDR + 0)
  DEBUGASSERT(GLB_CTL == 0x1100000);
  putreg32(EN_MIXER, GLB_CTL);

  return OK;
}

/// Initialize the UI Blender for PinePhone's A64 Display Engine.
/// Must be called after a64_de_init, and before a64_de_ui_channel_init
int a64_de_blender_init(void)
{
  /* Set Blender Background *************************************************/

  ginfo("Set Blender Background\n");

  // Blender Background Color (DE Page 109)
  // Set to 0xFF00 0000 (Black Background Color)
  // Set RESERVED (Bits 24 to 31) = 0xFF (Undocumented)
  // Set RED   (Bits 16 to 23) = 0
  // Set GREEN (Bits 8  to 15) = 0
  // Set BLUE  (Bits 0  to 7)  = 0

  #define RESERVED (0xFF << 24)
  #define RED (0    << 16)
  #define GREEN (0    << 8)
  #define BLUE (0    << 0)
  uint32_t color;
  color = RESERVED
      | RED
      | GREEN
      | BLUE;
  DEBUGASSERT(color == 0xFF000000);

  #define BLD_BK_COLOR (A64_BLD_ADDR + 0x88)
  DEBUGASSERT(BLD_BK_COLOR == 0x1101088);
  putreg32(color, BLD_BK_COLOR);

  /* Set Blender Pre-Multiply ***********************************************/

  ginfo("Set Blender Pre-Multiply\n");

  // Blender Pre-Multiply Control (DE Page 109)
  // Set to 0 (No Pre-Multiply for Alpha, Pipes 0 to 3)
  // Set P3_ALPHA_MODE (Bit 3) = 0 (Pipe 3: No Pre-Multiply)
  // Set P2_ALPHA_MODE (Bit 2) = 0 (Pipe 2: No Pre-Multiply)
  // Set P1_ALPHA_MODE (Bit 1) = 0 (Pipe 1: No Pre-Multiply)
  // Set P0_ALPHA_MODE (Bit 0) = 0 (Pipe 0: No Pre-Multiply)

  #define P3_ALPHA_MODE (0 << 3)
  #define P2_ALPHA_MODE (0 << 2)
  #define P1_ALPHA_MODE (0 << 1)
  #define P0_ALPHA_MODE (0 << 0)
  uint32_t premultiply;
  premultiply = P3_ALPHA_MODE
      | P2_ALPHA_MODE
      | P1_ALPHA_MODE
      | P0_ALPHA_MODE;
  DEBUGASSERT(premultiply == 0);

  #define BLD_PREMUL_CTL (A64_BLD_ADDR + 0x84)
  DEBUGASSERT(BLD_PREMUL_CTL == 0x1101084);
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
  DEBUGASSERT(channel >= 1 && channel <= 3);
  DEBUGASSERT(fblen == xres * yres * 4);
  DEBUGASSERT(stride == xres * 4);

  // OVL_UI(CH1) (UI Overlay 1) is at MIXER0 Offset 0x3000
  // OVL_UI(CH2) (UI Overlay 2) is at MIXER0 Offset 0x4000
  // OVL_UI(CH3) (UI Overlay 3) is at MIXER0 Offset 0x5000
  // (DE Page 102, 0x110 3000 / 0x110 4000 / 0x110 5000)
  #define OVL_UI_BASE_ADDRESS(ch) (A64_OVL_UI_CH1_ADDR + ((ch) - 1) * 0x1000)
  DEBUGASSERT(OVL_UI_BASE_ADDRESS(channel) == 0x1103000 || OVL_UI_BASE_ADDRESS(channel) == 0x1104000 || OVL_UI_BASE_ADDRESS(channel) == 0x1105000);

  // UI_SCALER1(CH1) is at MIXER0 Offset 0x04 0000
  // UI_SCALER2(CH2) is at MIXER0 Offset 0x05 0000
  // UI_SCALER3(CH3) is at MIXER0 Offset 0x06 0000
  // (DE Page 90, 0x114 0000 / 0x115 0000 / 0x116 0000)
  #define UI_SCALER_BASE_ADDRESS(ch) (A64_UI_SCALER1_ADDR + ((ch) - 1) * 0x10000)

  // If UI Channel should be disabled...
  if (fbmem == NULL) {
      /* Disable Overlay and Pipe *******************************************/

      ginfo("Channel %d: Disable Overlay and Pipe\n", channel);

      // UI Overlay Attribute Control (DE Page 102)
      // Set LAY_EN (Bit 0) = 0 (Disable Layer)
      #define OVL_UI_ATTR_CTL(ch) (OVL_UI_BASE_ADDRESS(ch) + 0x00)
      DEBUGASSERT(OVL_UI_ATTR_CTL(channel) == 0x1103000 || OVL_UI_ATTR_CTL(channel) == 0x1104000 || OVL_UI_ATTR_CTL(channel) == 0x1105000);
      putreg32(0, OVL_UI_ATTR_CTL(channel));

      /* Disable Scaler *****************************************************/

      ginfo("Channel %d: Disable Scaler\n", channel);

      // UI Scaler Control Register (DE Page 66)
      // Set EN (Bit 0) = 0 (Disable UI Scaler)
      #define UIS_CTRL_REG(ch) (UI_SCALER_BASE_ADDRESS(ch) + 0)
      DEBUGASSERT(UIS_CTRL_REG(channel) == 0x1140000 || UIS_CTRL_REG(channel) == 0x1150000 || UIS_CTRL_REG(channel) == 0x1160000);
      putreg32(0, UIS_CTRL_REG(channel));

      // Skip to next UI Channel
      return OK;
  }

  /* Set Overlay (Assume Layer = 0) *****************************************/

  ginfo("Channel %d: Set Overlay (%d x %d)\n", channel, xres, yres);

  // UI Overlay Attribute Control (DE Page 102)
  // Set LAY_GLBALPHA (Bits 24 to 31) = 0xFF or 0x7F
  //   (Global Alpha Value is Opaque or Semi-Transparent)
  // Set LAY_FBFMT (Bits 8 to 12) = 4 or 0
  //   (Input Data Format is XRGB 8888 or ARGB 8888)
  // Set LAY_ALPHA_MODE (Bits 1 to 2) = 2
  //   (Global Alpha is mixed with Pixel Alpha)
  //   (Input Alpha Value = Global Alpha Value * Pixel’s Alpha Value)
  // Set LAY_EN (Bit 0) = 1 (Enable Layer)
  uint32_t lay_glbalpha;
  lay_glbalpha = (
      (channel == 1) ? 0xff :  // Channel 1: Opaque
      (channel == 2) ? 0xff :  // Channel 2: Opaque
      (channel == 3) ? 0x7f :  // Channel 3: Semi-Transparent
      0xff
  ) << 24;  // Bits 24 to 31

  uint32_t lay_fbfmt;
  lay_fbfmt = (
      (channel == 1) ? 4 :  // Channel 1: XRGB 8888
      (channel == 2) ? 0 :  // Channel 2: ARGB 8888
      (channel == 3) ? 0 :  // Channel 3: ARGB 8888
      0
  ) << 8;  // Bits 8 to 12

  #define LAY_ALPHA_MODE (2 << 1)
  #define LAY_EN (1 << 0)
  uint32_t attr;
  attr = lay_glbalpha
      | lay_fbfmt
      | LAY_ALPHA_MODE
      | LAY_EN;
  DEBUGASSERT(attr == 0xFF000405 || attr == 0xFF000005 || attr == 0x7F000005);

  #define OVL_UI_ATTR_CTL(ch) (OVL_UI_BASE_ADDRESS(ch) + 0x00)
  DEBUGASSERT(OVL_UI_ATTR_CTL(channel) == 0x1103000 || OVL_UI_ATTR_CTL(channel) == 0x1104000 || OVL_UI_ATTR_CTL(channel) == 0x1105000);
  putreg32(attr, OVL_UI_ATTR_CTL(channel));

  // UI Overlay Top Field Memory Block Low Address (DE Page 104)
  // Set to Framebuffer Address
  DEBUGASSERT((((uint64_t)fbmem) & 0xffffffff) == (uint64_t)fbmem);  // 32 bits only
  #define OVL_UI_TOP_LADD(ch) (OVL_UI_BASE_ADDRESS(ch) + 0x10)
  DEBUGASSERT(OVL_UI_TOP_LADD(channel) == 0x1103010 || OVL_UI_TOP_LADD(channel) == 0x1104010 || OVL_UI_TOP_LADD(channel) == 0x1105010);
  putreg32((uint64_t)fbmem, OVL_UI_TOP_LADD(channel));

  // UI Overlay Memory Pitch (DE Page 104)
  // Set to (width * 4), number of bytes per row
  #define OVL_UI_PITCH(ch) (OVL_UI_BASE_ADDRESS(ch) + 0x0C)
  DEBUGASSERT(OVL_UI_PITCH(channel) == 0x110300C || OVL_UI_PITCH(channel) == 0x110400C || OVL_UI_PITCH(channel) == 0x110500C);
  putreg32(xres * 4, OVL_UI_PITCH(channel));

  // UI Overlay Memory Block Size (DE Page 104)
  // Set to (height-1) << 16 + (width-1)
  uint32_t height_width;
  height_width = ((yres - 1) << 16)
      | (xres - 1);
  #define OVL_UI_MBSIZE(ch) (OVL_UI_BASE_ADDRESS(ch) + 0x04)
  DEBUGASSERT(OVL_UI_MBSIZE(channel) == 0x1103004 || OVL_UI_MBSIZE(channel) == 0x1104004 || OVL_UI_MBSIZE(channel) == 0x1105004);
  putreg32(height_width, OVL_UI_MBSIZE(channel));

  // UI Overlay Overlay Window Size (DE Page 106)
  // Set to (height-1) << 16 + (width-1)
  #define OVL_UI_SIZE(ch) (OVL_UI_BASE_ADDRESS(ch) + 0x88)
  DEBUGASSERT(OVL_UI_SIZE(channel) == 0x1103088 || OVL_UI_SIZE(channel) == 0x1104088 || OVL_UI_SIZE(channel) == 0x1105088);
  putreg32(height_width, OVL_UI_SIZE(channel));

  // UI Overlay Memory Block Coordinate (DE Page 104)
  // Set to 0 (Overlay at X=0, Y=0)
  #define OVL_UI_COOR(ch) (OVL_UI_BASE_ADDRESS(ch) + 0x08)
  DEBUGASSERT(OVL_UI_COOR(channel) == 0x1103008 || OVL_UI_COOR(channel) == 0x1104008 || OVL_UI_COOR(channel) == 0x1105008);
  putreg32(0, OVL_UI_COOR(channel));

  // For Channel 1: Set Blender Output
  if (channel == 1)
  {
    /* Set Blender Output ***************************************************/

    ginfo("Channel %d: Set Blender Output\n", channel);

    // Blender Output Size Setting (DE Page 110)
    // Set to (height-1) << 16 + (width-1)
    #define BLD_SIZE (A64_BLD_ADDR + 0x08C)
    DEBUGASSERT(BLD_SIZE == 0x110108C);
    putreg32(height_width, BLD_SIZE);

    // Global Size (DE Page 93)
    // Set to (height-1) << 16 + (width-1)
    #define GLB_SIZE (A64_GLB_ADDR + 0x00C)
    DEBUGASSERT(GLB_SIZE == 0x110000C);
    putreg32(height_width, GLB_SIZE);
  }

  uint8_t pipe;
  pipe = channel - 1;

  /* Set Blender Input Pipe *************************************************/

  ginfo("Channel %d: Set Blender Input Pipe %d (%d x %d)\n", channel, pipe, xres, yres);

  // Note: DE Page 91 shows incorrect offset N*0x14 for
  // BLD_CH_ISIZE, BLD_FILL_COLOR and BLD_CH_OFFSET.
  // Correct offset is N*0x10, see DE Page 108
  // (N = Pipe Number, from 0 to 2 for Channels 1 to 3)

  // Blender Input Memory Size (DE Page 108)
  // Set to (height-1) << 16 + (width-1)
  #define BLD_CH_ISIZE(p) (A64_BLD_ADDR + 0x008 + (p) * 0x10)
  DEBUGASSERT(BLD_CH_ISIZE(pipe) == 0x1101008 || BLD_CH_ISIZE(pipe) == 0x1101018 || BLD_CH_ISIZE(pipe) == 0x1101028);
  putreg32(height_width, BLD_CH_ISIZE(pipe));

  // Blender Fill Color (DE Page 107)
  // Set to Opaque Black
  // Set ALPHA (Bits 24 to 31) = 0xFF (Opaque)
  // Set RED   (Bits 16 to 23) = 0
  // Set GREEN (Bits 8  to 15) = 0
  // Set BLUE  (Bits 0  to 7)  = 0
  #define ALPHA (0xFF << 24)
  #define RED (0    << 16)
  #define GREEN (0    << 8)
  #define BLUE (0    << 0)
  uint32_t color;
  color = ALPHA
      | RED
      | GREEN
      | BLUE;
  DEBUGASSERT(color == 0xFF000000);

  #define BLD_FILL_COLOR(p) (A64_BLD_ADDR + 0x004 + (p) * 0x10)
  DEBUGASSERT(BLD_FILL_COLOR(pipe) == 0x1101004 || BLD_FILL_COLOR(pipe) == 0x1101014 || BLD_FILL_COLOR(pipe) == 0x1101024);
  putreg32(color, BLD_FILL_COLOR(pipe));

  // Blender Input Memory Offset (DE Page 108)
  // Set to y_offset << 16 + x_offset
  uint32_t offset;
  offset = ((yoffset) << 16)
      | xoffset;
  DEBUGASSERT(offset == 0 || offset == 0x340034);

  #define BLD_CH_OFFSET(p) (A64_BLD_ADDR + 0x00C + (p) * 0x10)
  DEBUGASSERT(BLD_CH_OFFSET(pipe) == 0x110100C || BLD_CH_OFFSET(pipe) == 0x110101C || BLD_CH_OFFSET(pipe) == 0x110102C);
  putreg32(offset, BLD_CH_OFFSET(pipe));

  // Blender Control (DE Page 110)
  // Set BLEND_AFD (Bits 24 to 27) = 3
  //   (Coefficient for destination alpha data Q[d] is 1-A[s])
  // Set BLEND_AFS (Bits 16 to 19) = 1
  //   (Coefficient for source alpha data Q[s] is 1)
  // Set BLEND_PFD (Bits 8 to 11) = 3
  //   (Coefficient for destination pixel data F[d] is 1-A[s])
  // Set BLEND_PFS (Bits 0 to 3) = 1
  //   (Coefficient for source pixel data F[s] is 1)
  #define BLEND_AFD (3 << 24)
  #define BLEND_AFS (1 << 16)
  #define BLEND_PFD (3 << 8)
  #define BLEND_PFS (1 << 0)
  uint32_t blend;
  blend = BLEND_AFD
      | BLEND_AFS
      | BLEND_PFD
      | BLEND_PFS;

  #define BLD_CTL(p) (A64_BLD_ADDR + 0x090 + (p) * 4)
  DEBUGASSERT(BLD_CTL(pipe) == 0x1101090 || BLD_CTL(pipe) == 0x1101094 || BLD_CTL(pipe) == 0x1101098);
  putreg32(blend, BLD_CTL(pipe));

  /* Disable Scaler *********************************************************/

  ginfo("Channel %d: Disable Scaler\n", channel);

  // UI Scaler Control Register (DE Page 66)
  // Set EN (Bit 0) = 0 (Disable UI Scaler)
  #define UIS_CTRL_REG(ch) (UI_SCALER_BASE_ADDRESS(ch) + 0)
  DEBUGASSERT(UIS_CTRL_REG(channel) == 0x1140000 || UIS_CTRL_REG(channel) == 0x1150000 || UIS_CTRL_REG(channel) == 0x1160000);
  putreg32(0, UIS_CTRL_REG(channel));

  return OK;
}

/// Set UI Blender Route, enable Blender Pipes and apply the settings for PinePhone's A64 Display Engine.
/// Must be called after a64_de_ui_channel_init
int a64_de_enable(
  uint8_t channels  // Number of enabled UI Channels
)
{
  DEBUGASSERT(channels == 1 || channels == 3);

  /* Set Blender Route ******************************************************/

  ginfo("Set Blender Route\n");

  // Blender Routing Control (DE Page 108)
  // If Rendering 1 UI Channel:
  //   Set P0_RTCTL (Bits 0 to 3) = 1 (Pipe 0 from Channel 1)
  // If Rendering 3 UI Channels:
  //   Set P2_RTCTL (Bits 8 to 11) = 3 (Pipe 2 from Channel 3)
  //   Set P1_RTCTL (Bits 4 to 7)  = 2 (Pipe 1 from Channel 2)
  //   Set P0_RTCTL (Bits 0 to 3)  = 1 (Pipe 0 from Channel 1)
  uint32_t p2_rtctl;
  p2_rtctl = (
      (channels == 1) ? 0 :  // 1 UI Channel:  Unused Pipe 2
      (channels == 3) ? 3 :  // 3 UI Channels: Select Pipe 2 from UI Channel 3
      0
  ) << 8;

  uint32_t p1_rtctl;
  p1_rtctl = (
      (channels == 1) ? 0 :  // 1 UI Channel:  Unused Pipe 1
      (channels == 3) ? 2 :  // 3 UI Channels: Select Pipe 1 from UI Channel 2
      0
   ) << 4;

  #define P0_RTCTL (1 << 0)
  uint32_t route;
  route = p2_rtctl
      | p1_rtctl
      | P0_RTCTL;
  DEBUGASSERT(route == 0x321 || route == 1);

  #define BLD_CH_RTCTL (A64_BLD_ADDR + 0x080)
  DEBUGASSERT(BLD_CH_RTCTL == 0x1101080);
  putreg32(route, BLD_CH_RTCTL);

  /* Enable Blender Pipes ***************************************************/

  ginfo("Enable Blender Pipes\n");

  // Blender Fill Color Control (DE Page 106)
  // If Rendering 1 UI Channel:
  //   Set P0_EN   (Bit 8)  = 1 (Enable Pipe 0)
  //   Set P0_FCEN (Bit 0)  = 1 (Enable Pipe 0 Fill Color)
  // If Rendering 3 UI Channels:
  //   Set P2_EN   (Bit 10) = 1 (Enable Pipe 2)
  //   Set P1_EN   (Bit 9)  = 1 (Enable Pipe 1)
  //   Set P0_EN   (Bit 8)  = 1 (Enable Pipe 0)
  //   Set P0_FCEN (Bit 0)  = 1 (Enable Pipe 0 Fill Color)
  uint32_t p2_en;
  p2_en = (
      (channels == 1) ? 0 :  // 1 UI Channel:  Disable Pipe 2
      (channels == 3) ? 1 :  // 3 UI Channels: Enable Pipe 2
      0
   ) << 10;

  uint32_t p1_en;
  p1_en = (
      (channels == 1) ? 0 :  // 1 UI Channel:  Disable Pipe 1
      (channels == 3) ? 1 :  // 3 UI Channels: Enable Pipe 1
      0
   ) << 9;

  #define P0_EN (1 << 8)
  #define P0_FCEN (1 << 0)
  uint32_t fill;
  fill = p2_en
      | p1_en
      | P0_EN
      | P0_FCEN;
  DEBUGASSERT(fill == 0x701 || fill == 0x101);

  #define BLD_FILL_COLOR_CTL (A64_BLD_ADDR + 0x000)
  DEBUGASSERT(BLD_FILL_COLOR_CTL == 0x1101000);
  putreg32(fill, BLD_FILL_COLOR_CTL);

  /* Apply Settings *********************************************************/

  ginfo("Apply Settings\n");

  // Global Double Buffer Control (DE Page 93)
  // Set DOUBLE_BUFFER_RDY (Bit 0) = 1
  // (Register Value is ready for update)

  #define DOUBLE_BUFFER_RDY (1 << 0)
  DEBUGASSERT(DOUBLE_BUFFER_RDY == 1);

  #define GLB_DBUFFER (A64_GLB_ADDR + 0x008)
  DEBUGASSERT(GLB_DBUFFER == 0x1100008);
  putreg32(DOUBLE_BUFFER_RDY, GLB_DBUFFER);

  return OK;
}

//// TODO: Remove Test Code
#include "../../pinephone-nuttx/test/test_a64_de.c" //// TODO: Remove Test Code
