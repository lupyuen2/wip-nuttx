/****************************************************************************
 * arch/arm64/src/a64/a64_tcon0.c
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
#include "arm64_arch.h"
#include "a64_tcon0.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A64 CCU Registers and Bit Definitions ************************************/

// PLL_VIDEO0 Control Register (A64 Page 86)
#define PLL_VIDEO0_CTRL_REG (A64_CCU_ADDR + 0x10)
#define PLL_ENABLE      (1 << 31)
#define PLL_MODE_SEL    (1 << 24)
#define PLL_FACTOR_N(n) ((n) << 8)
#define PLL_PREDIV_M(n) ((n) << 0)

// PLL_MIPI Control Register (A64 Page 94)
#define PLL_MIPI_CTRL_REG (A64_CCU_ADDR + 0x40)
#define PLL_ENABLE (1  << 31)
#define LDO1_EN (1 << 23)
#define LDO2_EN (1 << 22)
#define PLL_SRC(n) ((n)  << 21)
#define PLL_FACTOR_N(n) ((n)  << 8)
#define PLL_FACTOR_K (1  << 4)
#define PLL_PRE_DIV_M(n) ((n) << 0)

// TCON0 Clock Register (A64 Page 117)
#define TCON0_CLK_REG (A64_CCU_ADDR + 0x118)
#define SCLK_GATING (1 << 31)
#define CLK_SRC_SEL(n) ((n) << 24)

// Bus Clock Gating Register 1 (A64 Page 102)
#define BUS_CLK_GATING_REG1 (A64_CCU_ADDR + 0x64)
#define TCON0_GATING (1 << 3)

// Bus Software Reset Register 1 (A64 Page 140)
#define BUS_SOFT_RST_REG1 (A64_CCU_ADDR + 0x2c4)
#define TCON0_RST (1 << 3)

/* A64 TCON0 Registers and Bit Definitions **********************************/

// TCON Global Control Register (A64 Page 508)
#define TCON_GCTL_REG (A64_TCON0_ADDR + 0x00)
#define TCON_En (1 << 31)

// TCON Global Interrupt Register 0 (A64 Page 509)
#define TCON_GINT0_REG (A64_TCON0_ADDR + 0x04)

// TCON Global Interrupt Register 1 (A64 Page 510)
#define TCON_GINT1_REG (A64_TCON0_ADDR + 0x08)

// TCON0 IO Trigger Register (A64 Page 520)
#define TCON0_IO_TRI_REG (A64_TCON0_ADDR + 0x8c)
#define IO_Reserved (0b111 << 29)
#define RGB_Endian(n) ((n)     << 28)
#define IO3_Output_Tri_En(n) ((n)     << 27)
#define IO2_Output_Tri_En(n) ((n)     << 26)
#define IO1_Output_Tri_En(n) ((n)     << 25)
#define IO0_Output_Tri_En(n) ((n)     << 24)
#define Data_Output_Tri_En(n) ((n)     << 0)

// TCON1 IO Trigger Register (A64 Page 520)
#define TCON1_IO_TRI_REG (A64_TCON0_ADDR + 0xf4)

// TCON0 Data Clock Register (A64 Page 513)
#define TCON0_DCLK_REG (A64_TCON0_ADDR + 0x44)
#define TCON0_Dclk_En(n) ((n) << 28)
#define TCON0_Dclk_Div(n) ((n) << 0)

// TCON0 Control Register (A64 Page 512)
#define TCON0_CTL_REG (A64_TCON0_ADDR + 0x40)
#define TCON0_En (1 << 31)
#define TCON0_IF(n) ((n) << 24)
#define TCON0_SRC_SEL(n) ((n) << 0);

// TCON0 Basic Timing Register 0 (A64 Page 514)
#define TCON0_BASIC0_REG (A64_TCON0_ADDR + 0x48)
#define TCON0_X(n) ((n) << 16)
#define TCON0_Y(n) ((n) << 0)

// TCON0 ECC FIFO Register (Undocumented)
#define TCON0_ECC_FIFO (A64_TCON0_ADDR + 0xf8)

// TCON0 CPU Panel Interface Register (A64 Page 516)
#define TCON0_CPU_IF_REG (A64_TCON0_ADDR + 0x60)
#define CPU_Mode (1 << 28)
#define FLUSH (1 << 16)
#define Trigger_FIFO_En (1 << 2)
#define Trigger_En (1 << 0)

// TCON0 CPU Panel Trigger Register 0 (A64 Page 521)
#define TCON0_CPU_TRI0_REG (A64_TCON0_ADDR + 0x160)
#define Block_Space(n) ((n)  << 16)
#define Block_Size(n) ((n) << 0)

// TCON0 CPU Panel Trigger Register 1 (A64 Page 522)
#define TCON0_CPU_TRI1_REG (A64_TCON0_ADDR + 0x164)
#define Block_Current_Num (0    << 16)
#define Block_Num(n) ((n) << 0)

// TCON0 CPU Panel Trigger Register 2 (A64 Page 522)
#define TCON0_CPU_TRI2_REG (A64_TCON0_ADDR + 0x168)
#define Start_Delay(n) ((n) << 16)
#define Trans_Start_Mode(n) ((n)    << 15)
#define Sync_Mode(n) ((n)    << 13)
#define Trans_Start_Set(n) ((n)   << 0)

// TCON Safe Period Register (A64 Page 525)
#define TCON_SAFE_PERIOD_REG (A64_TCON0_ADDR + 0x1f0)
#define Safe_Period_FIFO_Num(n) ((n) << 16)
#define Safe_Period_Line(n) ((n)    << 4)
#define Safe_Period_Mode(n) ((n)    << 0)

/* A64 CCU Registers and Bit Definitions ************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a64_tcon0_init
 *
 * Description:
 *   Initialize Timing Controller TCON0 to stream pixel data from Display
 *   Engine to MIPI Display Serial Interface. Should be called before
 *   enabling the MIPI DSI Block on the SoC.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

int a64_tcon0_init(void)
{
  /* Configure PLL_VIDEO0 ***************************************************/

  ginfo("Configure PLL_VIDEO0\n");

  // PLL_VIDEO0 Control Register (A64 Page 86)
  // Set PLL_ENABLE (Bit 31) to 1 (Enable PLL)
  // Set PLL_MODE_SEL (Bit 24) to 1 (Integer Mode)
  // Set PLL_FACTOR_N (Bits 8 to 14) to 0x62 (PLL Factor N)
  // Set PLL_PREDIV_M (Bits 0 to 3) to 7 (PLL Pre Divider)
  DEBUGASSERT(PLL_VIDEO0_CTRL_REG == 0x1c20010);
  uint32_t pll_video0_ctrl;
  pll_video0_ctrl = PLL_ENABLE
          | PLL_MODE_SEL
          | PLL_FACTOR_N(0x62)
          | PLL_PREDIV_M(7);
  DEBUGASSERT(pll_video0_ctrl == 0x81006207);
  putreg32(pll_video0_ctrl, PLL_VIDEO0_CTRL_REG);

  /* Enable LDO1 and LDO2 ***************************************************/

  ginfo("Enable LDO1 and LDO2\n");

  // PLL_MIPI Control Register (A64 Page 94)
  // Set LDO1_EN (Bit 23) to 1 (Enable On-chip LDO1)
  // Set LDO2_EN (Bit 22) to 1 (Enable On-chip LDO2)
  DEBUGASSERT(PLL_MIPI_CTRL_REG == 0x1c20040);
  uint32_t pll_mipi_ctrl;
  pll_mipi_ctrl = LDO1_EN
      | LDO2_EN;
  DEBUGASSERT(pll_mipi_ctrl == 0xc00000);
  putreg32(pll_mipi_ctrl, PLL_MIPI_CTRL_REG);

  // Wait at least 100 microseconds
  up_mdelay(1);

  /* Configure MIPI PLL *****************************************************/

  ginfo("Configure MIPI PLL\n");

  // PLL_MIPI Control Register (A64 Page 94)
  // Set PLL_ENABLE (Bit 31) to 1 (Enable MIPI PLL)
  // Set LDO1_EN (Bit 23) to 1 (Enable On-chip LDO1)
  // Set LDO2_EN (Bit 22) to 1 (Enable On-chip LDO2)
  // Set PLL_SRC (Bit 21) to 0 (PLL Source is VIDEO0 PLL)
  // Set PLL_FACTOR_N (Bits 8 to 11) to 7 (PLL Factor N)
  // Set PLL_FACTOR_K (Bits 4 to 5) to 1 (PLL Factor K)
  // Set PLL_PRE_DIV_M (Bits 0 to 3) to 10 (PLL Pre Divider)
  DEBUGASSERT(PLL_MIPI_CTRL_REG == 0x1c20040);
  pll_mipi_ctrl = PLL_ENABLE
      | LDO1_EN
      | LDO2_EN
      | PLL_SRC(0)
      | PLL_FACTOR_N(7)
      | PLL_FACTOR_K
      | PLL_PRE_DIV_M(10);
  DEBUGASSERT(pll_mipi_ctrl == 0x80c0071a);
  putreg32(pll_mipi_ctrl, PLL_MIPI_CTRL_REG);

  /* Set TCON0 Clock Source to MIPI PLL *************************************/

  ginfo("Set TCON0 Clock Source to MIPI PLL\n");

  // TCON0 Clock Register (A64 Page 117)
  // Set SCLK_GATING (Bit 31) to 1 (Special Clock is On)
  // Set CLK_SRC_SEL (Bits 24 to 26) to 0 (Clock Source is MIPI PLL)
  DEBUGASSERT(TCON0_CLK_REG == 0x1c20118);
  uint32_t tcon0_clk;
  tcon0_clk = SCLK_GATING
      | CLK_SRC_SEL(0);
  DEBUGASSERT(tcon0_clk == 0x80000000);
  putreg32(tcon0_clk, TCON0_CLK_REG);

  /* Enable TCON0 Clock *****************************************************/

  ginfo("Enable TCON0 Clock\n");

  // Bus Clock Gating Register 1 (A64 Page 102)
  // Set TCON0_GATING (Bit 3) to 1 (Pass Clock for TCON0)
  DEBUGASSERT(BUS_CLK_GATING_REG1 == 0x1c20064);
  putreg32(TCON0_GATING, BUS_CLK_GATING_REG1);

  /* Deassert TCON0 Reset ***************************************************/

  ginfo("Deassert TCON0 Reset\n");

  // Bus Software Reset Register 1 (A64 Page 140)
  // Set TCON0_RST (Bit 3) to 1 (Deassert TCON0 Reset)
  DEBUGASSERT(BUS_SOFT_RST_REG1 == 0x1c202c4);
  putreg32(TCON0_RST, BUS_SOFT_RST_REG1);

  /* Disable TCON0 and Interrupts *******************************************/

  ginfo("Disable TCON0 and Interrupts\n");

  // TCON Global Control Register (A64 Page 508)
  // Set to 0 (Disable TCON0)
  DEBUGASSERT(TCON_GCTL_REG == 0x1c0c000);
  putreg32(0, TCON_GCTL_REG);

  // TCON Global Interrupt Register 0 (A64 Page 509)
  // Set to 0 (Disable TCON0 Interrupts)
  DEBUGASSERT(TCON_GINT0_REG == 0x1c0c004);
  putreg32(0x0, TCON_GINT0_REG);

  // TCON Global Interrupt Register 1 (A64 Page 510)
  // Set to 0 (Disable TCON0 Interrupts)
  DEBUGASSERT(TCON_GINT1_REG == 0x1c0c008);
  putreg32(0x0, TCON_GINT1_REG);

  /* Enable Tristate Output *************************************************/

  ginfo("Enable Tristate Output\n");

  // TCON0 IO Trigger Register (A64 Page 520)
  // Set to 0xffff ffff to Enable TCON0 Tristate Output
  DEBUGASSERT(TCON0_IO_TRI_REG == 0x1c0c08c);
  putreg32(0xffffffff, TCON0_IO_TRI_REG);

  // TCON1 IO Trigger Register (A64 Page 520)
  // Set to 0xffff ffff to Enable TCON1 Tristate Output
  // Note: TCON1_IO_TRI_REG is actually in TCON0 Address Range, not in TCON1 Address Range as stated in A64 User Manual
  DEBUGASSERT(TCON1_IO_TRI_REG == 0x1c0c0f4);
  putreg32(0xffffffff, TCON1_IO_TRI_REG);

  /* Set DCLK to MIPI PLL / 6 ***********************************************/

  ginfo("Set DCLK to MIPI PLL / 6\n");

  // TCON0 Data Clock Register (A64 Page 513)
  // Set TCON0_Dclk_En (Bits 28 to 31) to 8 (Enable TCON0 Clocks: DCLK, DCLK1, DCLK2, DCLKM2)
  // Set TCON0_Dclk_Div (Bits 0 to 6) to 6 (DCLK Divisor)
  DEBUGASSERT(TCON0_DCLK_REG == 0x1c0c044);
  uint32_t tcon0_dclk;
  tcon0_dclk = TCON0_Dclk_En(8)
      | TCON0_Dclk_Div(6);
  DEBUGASSERT(tcon0_dclk == 0x80000006);
  putreg32(tcon0_dclk, TCON0_DCLK_REG);

  // TCON0 Control Register (A64 Page 512)
  // Set TCON0_En (Bit 31) to 1 (Enable TCON0)
  // Set TCON0_IF (Bits 24 to 25) to 1 (8080 Interface)
  // Set TCON0_SRC_SEL (Bits 0 to 2) to 0 (TCON0 Source is DE0)
  DEBUGASSERT(TCON0_CTL_REG == 0x1c0c040);
  uint32_t tcon0_ctl;
  tcon0_ctl = TCON0_En
      | TCON0_IF(1)
      | TCON0_SRC_SEL(0);
  DEBUGASSERT(tcon0_ctl == 0x81000000);
  putreg32(tcon0_ctl, TCON0_CTL_REG);

  // TCON0 Basic Timing Register 0 (A64 Page 514)
  // Set TCON0_X (Bits 16 to 27) to 719 (Panel Width - 1)
  // Set TCON0_Y (Bits 0 to 11) to 1439 (Panel Height - 1)
  DEBUGASSERT(TCON0_BASIC0_REG == 0x1c0c048);
  uint32_t tcon0_basic0;
  tcon0_basic0 = TCON0_X(A64_TCON0_PANEL_WIDTH  - 1)
      | TCON0_Y(A64_TCON0_PANEL_HEIGHT - 1);
  DEBUGASSERT(tcon0_basic0 == 0x2cf059f);
  putreg32(tcon0_basic0, TCON0_BASIC0_REG);

  // TCON0 ECC FIFO Register (Undocumented)
  // Set to 8
  DEBUGASSERT(TCON0_ECC_FIFO == 0x1c0c0f8);
  putreg32(0x8, TCON0_ECC_FIFO);

  // TCON0 CPU Panel Interface Register (A64 Page 516)
  // Set CPU_Mode (Bits 28 to 31) to 1 (24-bit DSI)
  // Set FLUSH (Bit 16) to 1 (Enable Direct Transfer Mode)
  // Set Trigger_FIFO_En (Bit 2) to 1 (Enable FIFO Trigger)
  // Set Trigger_En (Bit 0) to 1 (Enable Trigger Mode)
  DEBUGASSERT(TCON0_CPU_IF_REG == 0x1c0c060);

  uint32_t tcon0_cpu_if;
  tcon0_cpu_if = CPU_Mode
      | FLUSH
      | Trigger_FIFO_En
      | Trigger_En;
  DEBUGASSERT(tcon0_cpu_if == 0x10010005);
  putreg32(tcon0_cpu_if, TCON0_CPU_IF_REG);

  /* Set CPU Panel Trigger **************************************************/

  ginfo("Set CPU Panel Trigger\n");

  // TCON0 CPU Panel Trigger Register 0 (A64 Page 521)
  // Set Block_Space (Bits 16 to 27) to 47 (Block Space)
  // Set Block_Size (Bits 0 to 11) to 719 (Panel Width - 1)
  // Note: Block Space is probably derived from Panel Width
  DEBUGASSERT(TCON0_CPU_TRI0_REG == 0x1c0c160);

  uint32_t tcon0_cpu_tri0;
  tcon0_cpu_tri0 = Block_Space(47)
      | Block_Size(A64_TCON0_PANEL_WIDTH - 1);
  DEBUGASSERT(tcon0_cpu_tri0 == 0x2f02cf);
  putreg32(tcon0_cpu_tri0, TCON0_CPU_TRI0_REG);

  // TCON0 CPU Panel Trigger Register 1 (A64 Page 522)
  // Set Block_Current_Num (Bits 16 to 31) to 0 (Block Current Number)
  // Set Block_Num (Bits 0 to 15) to 1439 (Panel Height - 1)
  DEBUGASSERT(TCON0_CPU_TRI1_REG == 0x1c0c164);

  uint32_t tcon0_cpu_tri1;
  tcon0_cpu_tri1 = Block_Current_Num
      | Block_Num(A64_TCON0_PANEL_HEIGHT - 1);
  DEBUGASSERT(tcon0_cpu_tri1 == 0x59f);
  putreg32(tcon0_cpu_tri1, TCON0_CPU_TRI1_REG);

  // TCON0 CPU Panel Trigger Register 2 (A64 Page 522)
  // Set Start_Delay (Bits 16 to 31) to 7106 (Start Delay)
  // Set Trans_Start_Mode (Bit 15) to 0 (Trans Start Mode is ECC FIFO + TRI FIFO)
  // Set Sync_Mode (Bits 13 to 14) to 0 (Sync Mode is Auto)
  // Set Trans_Start_Set (Bits 0 to 12) to 10 (Trans Start Set)
  DEBUGASSERT(TCON0_CPU_TRI2_REG == 0x1c0c168);

  uint32_t tcon0_cpu_tri2;
  tcon0_cpu_tri2 = Start_Delay(7106)
      | Trans_Start_Mode(0)
      | Sync_Mode(0)
      | Trans_Start_Set(10);
  DEBUGASSERT(tcon0_cpu_tri2 == 0x1bc2000a);
  putreg32(tcon0_cpu_tri2, TCON0_CPU_TRI2_REG);

  /* Set Safe Period ********************************************************/

  ginfo("Set Safe Period\n");

  // TCON Safe Period Register (A64 Page 525)
  // Set Safe_Period_FIFO_Num (Bits 16 to 28) to 3000
  // Set Safe_Period_Line (Bits 4 to 15) to 0
  // Set Safe_Period_Mode (Bits 0 to 2) to 3 
  // (Safe Period Mode: Safe at 2 and safe at sync active)
  DEBUGASSERT(TCON_SAFE_PERIOD_REG == 0x1c0c1f0);

  uint32_t tcon_safe_period;
  tcon_safe_period = Safe_Period_FIFO_Num(3000)
      | Safe_Period_Line(0)
      | Safe_Period_Mode(3);
  DEBUGASSERT(tcon_safe_period == 0xbb80003);
  putreg32(tcon_safe_period, TCON_SAFE_PERIOD_REG);

  /* Enable Output Triggers *************************************************/

  ginfo("Enable Output Triggers\n");

  // TCON0 IO Trigger Register (A64 Page 520)
  // Set Reserved (Bits 29 to 31) to 0b111
  // Set RGB_Endian (Bit 28) to 0 (Normal RGB Endian)
  // Set IO3_Output_Tri_En (Bit 27) to 0 (Enable IO3 Output Tri)
  // Set IO2_Output_Tri_En (Bit 26) to 0 (Enable IO2 Output Tri)
  // Set IO1_Output_Tri_En (Bit 25) to 0 (Enable IO1 Output Tri)
  // Set IO0_Output_Tri_En (Bit 24) to 0 (Enable IO0 Output Tri)
  // Set Data_Output_Tri_En (Bits 0 to 23) to 0 (Enable TCON0 Output Port)
  DEBUGASSERT(TCON0_IO_TRI_REG == 0x1c0c08c);

  uint32_t tcon0_io_tri;
  tcon0_io_tri = IO_Reserved
      | RGB_Endian(0)
      | IO3_Output_Tri_En(0)
      | IO2_Output_Tri_En(0)
      | IO1_Output_Tri_En(0)
      | IO0_Output_Tri_En(0)
      | Data_Output_Tri_En(0);
  DEBUGASSERT(tcon0_io_tri == 0xe0000000);
  putreg32(tcon0_io_tri, TCON0_IO_TRI_REG);

  /* Enable TCON0 ***********************************************************/

  ginfo("Enable TCON0\n");

  // TCON Global Control Register (A64 Page 508)
  // Set TCON_En (Bit 31) to 1 (Enable TCON0)
  DEBUGASSERT(TCON_GCTL_REG == 0x1c0c000);
  modreg32(TCON_En, TCON_En, TCON_GCTL_REG);

  return OK;
}
