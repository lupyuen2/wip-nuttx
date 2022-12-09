/****************************************************************************
 * arch/arm64/src/a64/a64_mipi_dphy.c
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
 *   ???
 */

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include "arm64_arch.h"
#include "a64_mipi_dphy.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

/// Enable MIPI Display Physical Layer (DPHY).
/// Based on https://lupyuen.github.io/articles/dsi#appendix-enable-mipi-display-physical-layer-dphy
int a64_mipi_dphy_enable(void)
{
  // Set DSI Clock to 150 MHz (600 MHz / 4)
  // MIPI_DSI_CLK_REG: CCU Offset 0x168 (A64 Page 122)
  // Set DSI_DPHY_GATING (Bit 15) to 1 (DSI DPHY Clock is On)
  // Set DSI_DPHY_SRC_SEL (Bits 8 to 9) to 0b10 (DSI DPHY Clock Source is PLL_PERIPH0(1X))
  // Set DPHY_CLK_DIV_M (Bits 0 to 3) to 3 (DSI DPHY Clock divide ratio - 1)
  ginfo("Set DSI Clock to 150 MHz\n");
  const uint64_t MIPI_DSI_CLK_REG = A64_CCU_ADDR + 0x168;
  DEBUGASSERT(MIPI_DSI_CLK_REG == 0x1c20168);

  const uint32_t DSI_DPHY_GATING = 1    << 15;
  const uint32_t DSI_DPHY_SRC_SEL = 0b10 << 8;
  const uint32_t DPHY_CLK_DIV_M  = 3    << 0;
  const uint32_t MIPI_DSI_CLK = DSI_DPHY_GATING
      | DSI_DPHY_SRC_SEL
      | DPHY_CLK_DIV_M;
  DEBUGASSERT(MIPI_DSI_CLK == 0x8203);
  putreg32(MIPI_DSI_CLK, MIPI_DSI_CLK_REG);  // TODO: DMB

  // Power on DPHY Tx (Undocumented)
  // DPHY_TX_CTL_REG: DPHY Offset 0x04
  // Set to 0x1000 0000
  ginfo("Power on DPHY Tx\n");
  const uint64_t DPHY_TX_CTL_REG = A64_DPHY_ADDR + 0x04;
  DEBUGASSERT(DPHY_TX_CTL_REG == 0x1ca1004);
  putreg32(0x10000000, DPHY_TX_CTL_REG);  // TODO: DMB

  // DPHY_TX_TIME0_REG: DPHY Offset 0x10
  // Set to 0xa06 000e
  const uint64_t DPHY_TX_TIME0_REG = A64_DPHY_ADDR + 0x10;
  DEBUGASSERT(DPHY_TX_TIME0_REG == 0x1ca1010);
  putreg32(0xa06000e, DPHY_TX_TIME0_REG);  // TODO: DMB

  // DPHY_TX_TIME1_REG: DPHY Offset 0x14
  // Set to 0xa03 3207
  const uint64_t DPHY_TX_TIME1_REG = A64_DPHY_ADDR + 0x14;
  DEBUGASSERT(DPHY_TX_TIME1_REG == 0x1ca1014);
  putreg32(0xa033207, DPHY_TX_TIME1_REG);  // TODO: DMB

  // DPHY_TX_TIME2_REG: DPHY Offset 0x18
  // Set to 0x1e
  const uint64_t DPHY_TX_TIME2_REG = A64_DPHY_ADDR + 0x18;
  DEBUGASSERT(DPHY_TX_TIME2_REG == 0x1ca1018);
  putreg32(0x1e, DPHY_TX_TIME2_REG);  // TODO: DMB

  // DPHY_TX_TIME3_REG: DPHY Offset 0x1c
  // Set to 0x0
  const uint64_t DPHY_TX_TIME3_REG = A64_DPHY_ADDR + 0x1c;
  DEBUGASSERT(DPHY_TX_TIME3_REG == 0x1ca101c);
  putreg32(0x0, DPHY_TX_TIME3_REG);  // TODO: DMB

  // DPHY_TX_TIME4_REG: DPHY Offset 0x20
  // Set to 0x303
  const uint64_t DPHY_TX_TIME4_REG = A64_DPHY_ADDR + 0x20;
  DEBUGASSERT(DPHY_TX_TIME4_REG == 0x1ca1020);
  putreg32(0x303, DPHY_TX_TIME4_REG);  // TODO: DMB

  // Enable DPHY (Undocumented)
  // DPHY_GCTL_REG: DPHY Offset 0x00 (Enable DPHY)
  // Set to 0x31
  ginfo("Enable DPHY\n");
  const uint64_t DPHY_GCTL_REG = A64_DPHY_ADDR + 0x00;
  DEBUGASSERT(DPHY_GCTL_REG == 0x1ca1000);
  putreg32(0x31, DPHY_GCTL_REG);  // TODO: DMB

  // DPHY_ANA0_REG: DPHY Offset 0x4c (PWS)
  // Set to 0x9f00 7f00
  const uint64_t DPHY_ANA0_REG = A64_DPHY_ADDR + 0x4c;
  DEBUGASSERT(DPHY_ANA0_REG == 0x1ca104c);
  putreg32(0x9f007f00, DPHY_ANA0_REG);  // TODO: DMB

  // DPHY_ANA1_REG: DPHY Offset 0x50 (CSMPS)
  // Set to 0x1700 0000
  const uint64_t DPHY_ANA1_REG = A64_DPHY_ADDR + 0x50;
  DEBUGASSERT(DPHY_ANA1_REG == 0x1ca1050);
  putreg32(0x17000000, DPHY_ANA1_REG);  // TODO: DMB

  // DPHY_ANA4_REG: DPHY Offset 0x5c (CKDV)
  // Set to 0x1f0 1555
  const uint64_t DPHY_ANA4_REG = A64_DPHY_ADDR + 0x5c;
  DEBUGASSERT(DPHY_ANA4_REG == 0x1ca105c);
  putreg32(0x1f01555, DPHY_ANA4_REG);  // TODO: DMB

  // DPHY_ANA2_REG: DPHY Offset 0x54 (ENIB)
  // Set to 0x2
  const uint64_t DPHY_ANA2_REG = A64_DPHY_ADDR + 0x54;
  DEBUGASSERT(DPHY_ANA2_REG == 0x1ca1054);
  putreg32(0x2, DPHY_ANA2_REG);  // TODO: DMB

  // Wait 5 microseconds
  up_mdelay(1);  // 5 microseconds is sufficient

  // Enable LDOR, LDOC, LDOD (Undocumented)
  // DPHY_ANA3_REG: DPHY Offset 0x58 (Enable LDOR, LDOC, LDOD)
  // Set to 0x304 0000
  ginfo("Enable LDOR, LDOC, LDOD\n");
  const uint64_t DPHY_ANA3_REG = A64_DPHY_ADDR + 0x58;
  DEBUGASSERT(DPHY_ANA3_REG == 0x1ca1058);
  putreg32(0x3040000, DPHY_ANA3_REG);  // TODO: DMB

  // Wait 1 microsecond
  up_mdelay(1);  // 1 microsecond is sufficient

  // DPHY_ANA3_REG: DPHY Offset 0x58 (Enable VTTC, VTTD)
  // Set bits 0xf800 0000
  DEBUGASSERT(DPHY_ANA3_REG == 0x1ca1058);
  const uint32_t EnableVTTC = 0xf8000000;
  modreg32(EnableVTTC, EnableVTTC, DPHY_ANA3_REG);  // TODO: DMB

  // Wait 1 microsecond
  up_mdelay(1);  // 1 microsecond is sufficient

  // DPHY_ANA3_REG: DPHY Offset 0x58 (Enable DIV)
  // Set bits 0x400 0000
  DEBUGASSERT(DPHY_ANA3_REG == 0x1ca1058);
  const uint32_t EnableDIV = 0x4000000;
  modreg32(EnableDIV, EnableDIV, DPHY_ANA3_REG);  // TODO: DMB

  // Wait 1 microsecond
  up_mdelay(1);  // 1 microsecond is sufficient

  // DPHY_ANA2_REG: DPHY Offset 0x54 (Enable CK_CPU)
  DEBUGASSERT(DPHY_ANA2_REG == 0x1ca1054);
  const uint32_t EnableCKCPU = 0x10;
  modreg32(EnableCKCPU, EnableCKCPU, DPHY_ANA2_REG);  // TODO: DMB

  // Wait 1 microsecond
  up_mdelay(1);  // 1 microsecond is sufficient

  // DPHY_ANA1_REG: DPHY Offset 0x50 (VTT Mode)
  // Set bits 0x8000 0000
  DEBUGASSERT(DPHY_ANA1_REG == 0x1ca1050);
  const uint32_t VTTMode = 0x80000000;
  modreg32(VTTMode, VTTMode, DPHY_ANA1_REG);  // TODO: DMB

  // DPHY_ANA2_REG: DPHY Offset 0x54 (Enable P2S CPU)
  // Set bits 0xf00 0000
  DEBUGASSERT(DPHY_ANA2_REG == 0x1ca1054);
  const uint32_t EnableP2SCPU = 0xf000000;
  modreg32(EnableP2SCPU, EnableP2SCPU, DPHY_ANA2_REG);  // TODO: DMB

  return OK;
}
