/****************************************************************************
 * arch/arm64/src/a64/a64_mipi_dsi.c
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
#include "mipi_dsi.h"
#include "a64_mipi_dsi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

// DSI_BASIC_CTL0_REG: DSI Offset 0x10 (A31 Page 845)
/// (DSI Configuration Register 0) at Offset 0x10
/// INSTRU_EN is Bit 0 of DSI_BASIC_CTL0_REG 
#define DSI_BASIC_CTL0_REG (A64_DSI_ADDR + 0x10)
#define INSTRU_EN (1 << 0)
const uint32_t CRC_En = 1 << 17;
const uint32_t ECC_En = 1 << 16;

// BUS_CLK_GATING_REG0: CCU Offset 0x60 (A64 Page 100)
const uint64_t BUS_CLK_GATING_REG0 = A64_CCU_ADDR + 0x60;
const uint32_t MIPIDSI_GATING = 1 << 1;

// BUS_SOFT_RST_REG0: CCU Offset 0x2C0 (A64 Page 138)
const uint64_t BUS_SOFT_RST_REG0 = A64_CCU_ADDR + 0x2C0;
const uint32_t MIPI_DSI_RST = 1 << 1;

// DSI_CTL_REG: DSI Offset 0x0 (A31 Page 843)
const uint64_t DSI_CTL_REG = A64_DSI_ADDR + 0x0;
const uint32_t DSI_En = 1 << 0;

// DSI_TRANS_START_REG: DSI Offset 0x60 (Undocumented)
const uint64_t DSI_TRANS_START_REG = A64_DSI_ADDR + 0x60;

// DSI_TRANS_ZERO_REG: DSI Offset 0x78 (Undocumented)
const uint64_t DSI_TRANS_ZERO_REG = A64_DSI_ADDR + 0x78;

// DSI_DEBUG_DATA_REG: DSI Offset 0x2f8 (Undocumented)
const uint64_t DSI_DEBUG_DATA_REG = A64_DSI_ADDR + 0x2f8;

// DSI_BASIC_CTL1_REG: DSI Offset 0x14 (A31 Page 846)
const uint64_t DSI_BASIC_CTL1_REG = A64_DSI_ADDR + 0x14;
#define Video_Start_Delay(n) (n << 4)
const uint32_t Video_Precision_Mode_Align  = 1    << 2;
const uint32_t Video_Frame_Start  = 1    << 1;
const uint32_t DSI_Mode  = 1    << 0;

// DSI_TCON_DRQ_REG: DSI Offset 0x7c (Undocumented)
const uint64_t DSI_TCON_DRQ_REG = A64_DSI_ADDR + 0x7c;

// DSI_INST_LOOP_SEL_REG: DSI Offset 0x40 (Undocumented)
const uint64_t DSI_INST_LOOP_SEL_REG = A64_DSI_ADDR + 0x40;

// DSI_PIXEL_PH_REG: DSI Offset 0x90 (A31 Page 848)
const uint64_t DSI_PIXEL_PH_REG = A64_DSI_ADDR + 0x90;

// DSI_PIXEL_PF0_REG: DSI Offset 0x98 (A31 Page 849)
const uint64_t DSI_PIXEL_PF0_REG = A64_DSI_ADDR + 0x98;
const uint32_t CRC_Force = 0xffff;

// DSI_PIXEL_PF1_REG: DSI Offset 0x9c (A31 Page 849)
const uint64_t DSI_PIXEL_PF1_REG = A64_DSI_ADDR + 0x9c;
#define CRC_Init_LineN(n) (n << 16)
#define CRC_Init_Line0(n) (n << 0)

// DSI_PIXEL_CTL0_REG: DSI Offset 0x80 (A31 Page 847)
const uint64_t DSI_PIXEL_CTL0_REG = A64_DSI_ADDR + 0x80;
const uint32_t PD_Plug_Dis = 1 << 16;
const uint32_t Pixel_Endian  = 0 << 4;
#define Pixel_Format(n) (n << 0)

// DSI_BASIC_CTL_REG: DSI Offset 0x0c (Undocumented)
const uint64_t DSI_BASIC_CTL_REG = A64_DSI_ADDR + 0x0c;

// DSI_SYNC_HSS_REG: DSI Offset 0xb0 (A31 Page 850)
const uint64_t DSI_SYNC_HSS_REG = A64_DSI_ADDR + 0xb0;

// DSI_SYNC_HSE_REG: DSI Offset 0xb4 (A31 Page 850)
const uint64_t DSI_SYNC_HSE_REG = A64_DSI_ADDR + 0xb4;

// DSI_SYNC_VSS_REG: DSI Offset 0xb8 (A31 Page 851)
const uint64_t DSI_SYNC_VSS_REG = A64_DSI_ADDR + 0xb8;

// DSI_SYNC_VSE_REG: DSI Offset 0xbc (A31 Page 851)
const uint64_t DSI_SYNC_VSE_REG = A64_DSI_ADDR + 0xbc;

// DSI_BASIC_SIZE0_REG: DSI Offset 0x18 (Undocumented)
const uint64_t DSI_BASIC_SIZE0_REG = A64_DSI_ADDR + 0x18;
#define Video_VBP(n) (n << 16)
#define Video_VSA(n) (n << 0)

// DSI_BASIC_SIZE1_REG: DSI Offset 0x1c (Undocumented)
const uint64_t DSI_BASIC_SIZE1_REG = A64_DSI_ADDR + 0x1c;
#define Video_VT(n) (n << 16)
#define Video_VACT(n) (n << 0)

// DSI_BLK_HSA0_REG: DSI Offset 0xc0 (A31 Page 852)
const uint64_t DSI_BLK_HSA0_REG = A64_DSI_ADDR + 0xc0;

// DSI_BLK_HSA1_REG: DSI Offset 0xc4 (A31 Page 852)
const uint64_t DSI_BLK_HSA1_REG = A64_DSI_ADDR + 0xc4;
#define HSA_PF(n) (n << 16)
#define HSA_PD(n) (n      << 0)

// DSI_BLK_HBP0_REG: DSI Offset 0xc8 (A31 Page 852)
const uint64_t DSI_BLK_HBP0_REG = A64_DSI_ADDR + 0xc8;

// DSI_BLK_HBP1_REG: DSI Offset 0xcc (A31 Page 852)
const uint64_t DSI_BLK_HBP1_REG = A64_DSI_ADDR + 0xcc;
#define HBP_PF(n) (n << 16)
#define HBP_PD(n) (n      << 0)

// DSI_BLK_HFP0_REG: DSI Offset 0xd0 (A31 Page 852)
const uint64_t DSI_BLK_HFP0_REG = A64_DSI_ADDR + 0xd0;

// DSI_BLK_HFP1_REG: DSI Offset 0xd4 (A31 Page 853)
const uint64_t DSI_BLK_HFP1_REG = A64_DSI_ADDR + 0xd4;
#define HFP_PF(n) (n << 16)
#define HFP_PD(n) (n      << 0)

// DSI_BLK_HBLK0_REG: DSI Offset 0xe0 (A31 Page 853)
const uint64_t DSI_BLK_HBLK0_REG = A64_DSI_ADDR + 0xe0;

// DSI_BLK_HBLK1_REG: DSI Offset 0xe4 (A31 Page 853)
const uint64_t DSI_BLK_HBLK1_REG = A64_DSI_ADDR + 0xe4;
#define HBLK_PF(n) (n << 16)
#define HBLK_PD(n) (n      << 0)

// DSI_BLK_VBLK0_REG: DSI Offset 0xe8 (A31 Page 854)
const uint64_t DSI_BLK_VBLK0_REG = A64_DSI_ADDR + 0xe8;

// DSI_BLK_VBLK1_REG: DSI Offset 0xec (A31 Page 854)
const uint64_t DSI_BLK_VBLK1_REG = A64_DSI_ADDR + 0xec;
#define VBLK_PF(n) (n << 16)
#define VBLK_PD(n) (n      << 0)

/// DSI Instruction Functions (Undocumented)
#define DSI_INST_FUNC_REG(n) (A64_DSI_ADDR + 0x020 + n * 0x04)
const uint32_t DSI_INST_FUNC_LANE_CEN = 1 << 4;

/// DSI Instruction Jump Configuration (Undocumented)
#define DSI_INST_JUMP_CFG_REG(n) (A64_DSI_ADDR + 0x04c + n * 0x04)
const uint32_t DSI_INST_JUMP_CFG = 0;

/// DSI Instruction Loop Number (Undocumented)
#define DSI_INST_LOOP_NUM_REG(n) (A64_DSI_ADDR + 0x044 + n * 0x10)

// DSI_INST_JUMP_SEL_REG: DSI Offset 0x48 (Undocumented)
const uint64_t DSI_INST_JUMP_SEL_REG = A64_DSI_ADDR + 0x48;
const uint32_t DSI_INST_ID_LP11 = 0;
const uint32_t DSI_INST_ID_TBA = 1;
const uint32_t DSI_INST_ID_HSC = 2;
const uint32_t DSI_INST_ID_HSD = 3;
const uint32_t DSI_INST_ID_LPDT = 4;
const uint32_t DSI_INST_ID_HSCEXIT = 5;
const uint32_t DSI_INST_ID_NOP = 6;
const uint32_t DSI_INST_ID_DLY = 7;
const uint32_t DSI_INST_ID_END  = 15;

// DSI_CMD_CTL_REG (DSI Low Power Control Register) at Offset 0x200
const uint64_t DSI_CMD_CTL_REG = A64_DSI_ADDR + 0x200;
const uint32_t RX_Overflow = 1 << 26;
const uint32_t RX_Flag     = 1 << 25;
const uint32_t TX_Flag     = 1 << 9;

// (DSI Low Power Transmit Package Register) at Offset 0x300 to 0x3FC
const uint64_t DSI_CMD_TX_REG = A64_DSI_ADDR + 0x300;

/************************************************************************************************
 * Private Data
 ************************************************************************************************/

/************************************************************************************************
 * Private Functions
 ************************************************************************************************/

/// Disable DSI Processing. See https://lupyuen.github.io/articles/dsi#transmit-packet-over-mipi-dsi
static void a64_disable_dsi_processing(void)
{
  // Set INSTRU_EN (Bit 0) to 0
  modreg32(0, INSTRU_EN, DSI_BASIC_CTL0_REG);  // TODO: DMB
}

/// Enable DSI Processing. See https://lupyuen.github.io/articles/dsi#transmit-packet-over-mipi-dsi
static void a64_enable_dsi_processing(void)
{
  // Set INSTRU_EN (Bit 0) to 1
  modreg32(INSTRU_EN, INSTRU_EN, DSI_BASIC_CTL0_REG);  // TODO: DMB
}

/// Wait for transmit to complete. Returns 0 if completed, -1 if timeout.
/// See https://lupyuen.github.io/articles/dsi#transmit-packet-over-mipi-dsi
static int a64_wait_dsi_transmit(void)
{
  // Wait up to 5 milliseconds
  int i;
  for (i = 0; i < 5; i++)
    {
      // To check whether the transmission is complete, we poll on INSTRU_EN (Bit 0)
      if ((getreg32(DSI_BASIC_CTL0_REG) & INSTRU_EN) == 0)
        {
          // If INSTRU_EN is 0, then transmission is complete
          return 0;
        }
      // Sleep 1 millisecond
      up_mdelay(1);
    }
  _err("timeout");
  return -1;  // TODO
}

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

/// Write to MIPI DSI. See https://lupyuen.github.io/articles/dsi#transmit-packet-over-mipi-dsi
/// On Success: Return number of written bytes. On Error: Return negative error code
ssize_t a64_mipi_dsi_write(
    uint8_t channel,  // Virtual Channel ID
    uint8_t cmd,      // DCS Command
    FAR const uint8_t *txbuf,  // Transmit Buffer
    size_t txlen          // Buffer Length
)
{  
  int ret;
  ssize_t pktlen = -1;
  ginfo("channel=%d, cmd=0x%x, txlen=%d\n", channel, cmd, (int) txlen); // TODO
  DEBUGASSERT(txbuf != NULL);
  if (cmd == MIPI_DSI_DCS_SHORT_WRITE)       
    { 
      DEBUGASSERT(txlen == 1); 
      if (txlen != 1) { return -1; }  // TODO
    }
  if (cmd == MIPI_DSI_DCS_SHORT_WRITE_PARAM)
    {
      DEBUGASSERT(txlen == 2); 
      if (txlen != 2) { return -1; }  // TODO
    }

  // Allocate Packet Buffer
  uint8_t pkt[128];  // TODO
  memset(pkt, 0, sizeof(pkt));

  // Compose Short or Long Packet depending on DCS Command
  switch (cmd)
    {
      // For DCS Long Write: Compose Long Packet
      case MIPI_DSI_DCS_LONG_WRITE:
        pktlen = mipi_dsi_long_packet(pkt, sizeof(pkt), channel, cmd, txbuf, txlen);
        break;

      // For DCS Short Write (with and without parameter):
      // Compose Short Packet
      case MIPI_DSI_DCS_SHORT_WRITE:
        pktlen = mipi_dsi_short_packet(pkt, sizeof(pkt), channel, cmd, txbuf, txlen);
        break;

      case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
        pktlen = mipi_dsi_short_packet(pkt, sizeof(pkt), channel, cmd, txbuf, txlen);
        break;

      default:
        DEBUGPANIC();  // Invalid DCS Command
        break;
    };
  if (pktlen < 0)
    {
      return pktlen; // TODO
    }
  DEBUGASSERT(pktlen > 0);

  // Dump the packet
  ginfo("pktlen=%d\n", (int) pktlen);
  ginfodumpbuffer("pkt", pkt, pktlen);

  // Set the following bits to 1 in DSI_CMD_CTL_REG (DSI Low Power Control Register) at Offset 0x200:
  // RX_Overflow (Bit 26): Clear flag for "Receive Overflow"
  // RX_Flag (Bit 25): Clear flag for "Receive has started"
  // TX_Flag (Bit 9): Clear flag for "Transmit has started"
  // All other bits must be set to 0.
  putreg32(
      RX_Overflow | RX_Flag | TX_Flag,
      DSI_CMD_CTL_REG
  );

  // Write the Long Packet to DSI_CMD_TX_REG 
  // (DSI Low Power Transmit Package Register) at Offset 0x300 to 0x3FC
  uint64_t addr = DSI_CMD_TX_REG;
  int i;
  for (i = 0; i < pktlen; i += 4)
    {
      // Fetch the next 4 bytes, fill with 0 if not available
      const uint32_t b[4] =
        {
          pkt[i],
          (i + 1 < pktlen) ? pkt[i + 1] : 0,
          (i + 2 < pktlen) ? pkt[i + 2] : 0,
          (i + 3 < pktlen) ? pkt[i + 3] : 0,
        };

      // Merge the next 4 bytes into a 32-bit value
      const uint32_t v =
          b[0]
          + (b[1] << 8)
          + (b[2] << 16)
          + (b[3] << 24);

      // Write the 32-bit value
      DEBUGASSERT(addr <= A64_DSI_ADDR + 0x3FC);
      modreg32(v, 0xFFFFFFFF, addr);  // TODO: DMB
      addr += 4;      
    }

  // Set Packet Length - 1 in Bits 0 to 7 (TX_Size) of
  // DSI_CMD_CTL_REG (DSI Low Power Control Register) at Offset 0x200
  modreg32(pktlen - 1, 0xFF, DSI_CMD_CTL_REG);  // TODO: DMB

  // Set DSI_INST_JUMP_SEL_REG (Offset 0x48, undocumented) 
  // to begin the Low Power Transmission (LPTX)
  putreg32(
      DSI_INST_ID_LPDT << (4 * DSI_INST_ID_LP11) |
      DSI_INST_ID_END  << (4 * DSI_INST_ID_LPDT),
      DSI_INST_JUMP_SEL_REG
  );

  // Disable DSI Processing then Enable DSI Processing
  a64_disable_dsi_processing();
  a64_enable_dsi_processing();

  // Wait for transmission to complete
  ret = a64_wait_dsi_transmit();
  if (ret < 0) {
      a64_disable_dsi_processing();
      return ret;
  }

  // Return number of written bytes
  return txlen;
}

/// Enable MIPI DSI Block.
/// Based on https://lupyuen.github.io/articles/dsi#appendix-enable-mipi-dsi-block
int a64_mipi_dsi_enable(void)
{
  // Enable MIPI DSI Bus
  // BUS_CLK_GATING_REG0: CCU Offset 0x60 (A64 Page 100)
  // Set MIPIDSI_GATING (Bit 1) to 1 (Pass Gating Clock for MIPI DSI)
  ginfo("Enable MIPI DSI Bus\n");
  DEBUGASSERT(BUS_CLK_GATING_REG0 == 0x1c20060);

  DEBUGASSERT(MIPIDSI_GATING == 2);
  modreg32(MIPIDSI_GATING, MIPIDSI_GATING, BUS_CLK_GATING_REG0);  // TODO: DMB

  // BUS_SOFT_RST_REG0: CCU Offset 0x2C0 (A64 Page 138)
  // Set MIPI_DSI_RST (Bit 1) to 1 (Deassert MIPI DSI Reset)
  DEBUGASSERT(BUS_SOFT_RST_REG0 == 0x1c202c0);

  DEBUGASSERT(MIPI_DSI_RST == 2);
  modreg32(MIPI_DSI_RST, MIPI_DSI_RST, BUS_SOFT_RST_REG0);  // TODO: DMB

  // Enable DSI Block
  // DSI_CTL_REG: DSI Offset 0x0 (A31 Page 843)
  // Set DSI_En (Bit 0) to 1 (Enable DSI)
  ginfo("Enable DSI Block\n");
  DEBUGASSERT(DSI_CTL_REG == 0x1ca0000);

  DEBUGASSERT(DSI_En == 1);
  putreg32(DSI_En, DSI_CTL_REG);  // TODO: DMB

  // DSI_BASIC_CTL0_REG: DSI Offset 0x10 (A31 Page 845)
  // Set CRC_En (Bit 17) to 1 (Enable CRC)
  // Set ECC_En (Bit 16) to 1 (Enable ECC)
  DEBUGASSERT(DSI_BASIC_CTL0_REG == 0x1ca0010);

  const uint32_t DSI_BASIC_CTL0 = CRC_En
      | ECC_En;
  DEBUGASSERT(DSI_BASIC_CTL0 == 0x30000);
  putreg32(DSI_BASIC_CTL0, DSI_BASIC_CTL0_REG);  // TODO: DMB

  // DSI_TRANS_START_REG: DSI Offset 0x60 (Undocumented)
  // Set to 10
  DEBUGASSERT(DSI_TRANS_START_REG == 0x1ca0060);
  putreg32(10, DSI_TRANS_START_REG);  // TODO: DMB

  // DSI_TRANS_ZERO_REG: DSI Offset 0x78 (Undocumented)
  // Set to 0
  DEBUGASSERT(DSI_TRANS_ZERO_REG == 0x1ca0078);
  putreg32(0, DSI_TRANS_ZERO_REG);  // TODO: DMB

  // Set Instructions (Undocumented)
  // DSI_INST_FUNC_REG(0): DSI Offset 0x20
  // Set to 0x1f
  // Index 0 is DSI_INST_ID_LP11
  ginfo("Set Instructions\n");
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_LP11) == 0x1ca0020);
  putreg32(0x1f, DSI_INST_FUNC_REG(DSI_INST_ID_LP11));  // TODO: DMB

  // DSI_INST_FUNC_REG(1): DSI Offset 0x24
  // Set to 0x1000 0001
  // Index 1 is DSI_INST_ID_TBA
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_TBA) == 0x1ca0024);
  putreg32(0x10000001, DSI_INST_FUNC_REG(DSI_INST_ID_TBA));  // TODO: DMB

  // DSI_INST_FUNC_REG(2): DSI Offset 0x28
  // Set to 0x2000 0010
  // Index 2 is DSI_INST_ID_HSC
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_HSC) == 0x1ca0028);
  putreg32(0x20000010, DSI_INST_FUNC_REG(DSI_INST_ID_HSC));  // TODO: DMB

  // DSI_INST_FUNC_REG(3): DSI Offset 0x2c
  // Set to 0x2000 000f
  // Index 3 is DSI_INST_ID_HSD
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_HSD) == 0x1ca002c);
  putreg32(0x2000000f, DSI_INST_FUNC_REG(DSI_INST_ID_HSD));  // TODO: DMB

  // DSI_INST_FUNC_REG(4): DSI Offset 0x30
  // Set to 0x3010 0001
  // Index 4 is DSI_INST_ID_LPDT
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_LPDT) == 0x1ca0030);
  putreg32(0x30100001, DSI_INST_FUNC_REG(DSI_INST_ID_LPDT));  // TODO: DMB

  // DSI_INST_FUNC_REG(5): DSI Offset 0x34
  // Set to 0x4000 0010
  // Index 5 is DSI_INST_ID_HSCEXIT
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_HSCEXIT) == 0x1ca0034);
  putreg32(0x40000010, DSI_INST_FUNC_REG(DSI_INST_ID_HSCEXIT));  // TODO: DMB

  // DSI_INST_FUNC_REG(6): DSI Offset 0x38
  // Set to 0xf
  // Index 6 is DSI_INST_ID_NOP
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_NOP) == 0x1ca0038);
  putreg32(0xf, DSI_INST_FUNC_REG(DSI_INST_ID_NOP));  // TODO: DMB

  // DSI_INST_FUNC_REG(7): DSI Offset 0x3c
  // Set to 0x5000 001f
  // Index 7 is DSI_INST_ID_DLY
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_DLY) == 0x1ca003c);
  putreg32(0x5000001f, DSI_INST_FUNC_REG(DSI_INST_ID_DLY));  // TODO: DMB

  // Configure Jump Instructions (Undocumented)
  // DSI_INST_JUMP_CFG_REG(0): DSI Offset 0x4c
  // Set to 0x56 0001    
  // Index 0 is DSI_INST_JUMP_CFG
  ginfo("Configure Jump Instructions\n");
  DEBUGASSERT(DSI_INST_JUMP_CFG_REG(DSI_INST_JUMP_CFG) == 0x1ca004c);
  putreg32(0x560001, DSI_INST_JUMP_CFG_REG(DSI_INST_JUMP_CFG));  // TODO: DMB

  // DSI_DEBUG_DATA_REG: DSI Offset 0x2f8
  // Set to 0xff
  DEBUGASSERT(DSI_DEBUG_DATA_REG == 0x1ca02f8);
  putreg32(0xff, DSI_DEBUG_DATA_REG);  // TODO: DMB

  // Set Video Start Delay
  // DSI_BASIC_CTL1_REG: DSI Offset 0x14 (A31 Page 846)
  // Set Video_Start_Delay (Bits 4 to 16) to 1468 (Line Delay)
  // Set Video_Precision_Mode_Align (Bit 2) to 1 (Fill Mode)
  // Set Video_Frame_Start (Bit 1) to 1 (Precision Mode)
  // Set DSI_Mode (Bit 0) to 1 (Video Mode)
  // TODO: Video_Start_Delay is actually 13 bits, not 8 bits as documented in the A31 User Manual
  ginfo("Set Video Start Delay\n");
  DEBUGASSERT(DSI_BASIC_CTL1_REG == 0x1ca0014);

  const uint32_t DSI_BASIC_CTL1 = Video_Start_Delay(1468)
      | Video_Precision_Mode_Align
      | Video_Frame_Start
      | DSI_Mode;
  DEBUGASSERT(DSI_BASIC_CTL1 == 0x5bc7);
  putreg32(DSI_BASIC_CTL1, DSI_BASIC_CTL1_REG);  // TODO: DMB

  // Set Burst (Undocumented)
  // DSI_TCON_DRQ_REG: DSI Offset 0x7c
  // Set to 0x1000 0007
  ginfo("Set Burst\n");
  DEBUGASSERT(DSI_TCON_DRQ_REG == 0x1ca007c);
  putreg32(0x10000007, DSI_TCON_DRQ_REG);  // TODO: DMB

  // Set Instruction Loop (Undocumented)
  // DSI_INST_LOOP_SEL_REG: DSI Offset 0x40
  // Set to 0x3000 0002
  ginfo("Set Instruction Loop\n");
  DEBUGASSERT(DSI_INST_LOOP_SEL_REG == 0x1ca0040);
  putreg32(0x30000002, DSI_INST_LOOP_SEL_REG);  // TODO: DMB

  // DSI_INST_LOOP_NUM_REG(0): DSI Offset 0x44
  // Set to 0x31 0031
  DEBUGASSERT(DSI_INST_LOOP_NUM_REG(0) == 0x1ca0044);
  putreg32(0x310031, DSI_INST_LOOP_NUM_REG(0));  // TODO: DMB

  // DSI_INST_LOOP_NUM_REG(1): DSI Offset 0x54
  // Set to 0x31 0031
  DEBUGASSERT(DSI_INST_LOOP_NUM_REG(1) == 0x1ca0054);
  putreg32(0x310031, DSI_INST_LOOP_NUM_REG(1));  // TODO: DMB

  // Set Pixel Format
  // DSI_PIXEL_PH_REG: DSI Offset 0x90 (A31 Page 848)
  // Set ECC (Bits 24 to 31) to 19
  // Set WC (Bits 8 to 23) to 2160 (Byte Numbers of PD in a Pixel Packet)
  // Set VC (Bits 6 to 7) to 0 (Virtual Channel)
  // Set DT (Bits 0 to 5) to 0x3E (24-bit Video Mode)
  ginfo("Set Pixel Format\n");
  DEBUGASSERT(DSI_PIXEL_PH_REG == 0x1ca0090);
  {
      const uint32_t ECC = 19   << 24;
      const uint32_t WC = 2160 << 8;
      const uint32_t VC  = A64_MIPI_DSI_VIRTUAL_CHANNEL << 6;
      const uint32_t DT  = 0x3E << 0;
      const uint32_t DSI_PIXEL_PH = ECC
          | WC
          | VC
          | DT;
      DEBUGASSERT(DSI_PIXEL_PH == 0x1308703e);
      putreg32(DSI_PIXEL_PH, DSI_PIXEL_PH_REG);  // TODO: DMB
  }

  // DSI_PIXEL_PF0_REG: DSI Offset 0x98 (A31 Page 849)
  // Set CRC_Force (Bits 0 to 15) to 0xffff (Force CRC to this value)
  DEBUGASSERT(DSI_PIXEL_PF0_REG == 0x1ca0098);
  putreg32(CRC_Force, DSI_PIXEL_PF0_REG);  // TODO: DMB

  // DSI_PIXEL_PF1_REG: DSI Offset 0x9c (A31 Page 849)
  // Set CRC_Init_LineN (Bits 16 to 31) to 0xffff (CRC initial to this value in transmitions except 1st one)
  // Set CRC_Init_Line0 (Bits 0 to 15) to 0xffff (CRC initial to this value in 1st transmition every frame)
  DEBUGASSERT(DSI_PIXEL_PF1_REG == 0x1ca009c);

  const uint32_t DSI_PIXEL_PF1 = CRC_Init_LineN(0xffff)
      | CRC_Init_Line0(0xffff);
  DEBUGASSERT(DSI_PIXEL_PF1 == 0xffffffff);
  putreg32(DSI_PIXEL_PF1, DSI_PIXEL_PF1_REG);  // TODO: DMB

  // DSI_PIXEL_CTL0_REG: DSI Offset 0x80 (A31 Page 847)
  // Set PD_Plug_Dis (Bit 16) to 1 (Disable PD plug before pixel bytes)
  // Set Pixel_Endian (Bit 4) to 0 (LSB first)
  // Set Pixel_Format (Bits 0 to 3) to 8 (24-bit RGB888)
  DEBUGASSERT(DSI_PIXEL_CTL0_REG == 0x1ca0080);

  const uint32_t DSI_PIXEL_CTL0 = PD_Plug_Dis
      | Pixel_Endian
      | Pixel_Format(8);
  DEBUGASSERT(DSI_PIXEL_CTL0 == 0x10008);
  putreg32(DSI_PIXEL_CTL0, DSI_PIXEL_CTL0_REG);  // TODO: DMB

  // Set Sync Timings
  // DSI_BASIC_CTL_REG: DSI Offset 0x0c (Undocumented)
  // Set to 0
  ginfo("Set Sync Timings\n");
  DEBUGASSERT(DSI_BASIC_CTL_REG == 0x1ca000c);
  putreg32(0x0, DSI_BASIC_CTL_REG);  // TODO: DMB

  // DSI_SYNC_HSS_REG: DSI Offset 0xb0 (A31 Page 850)
  // Set ECC (Bits 24 to 31) to 0x12
  // Set D1 (Bits 16 to 23) to 0
  // Set D0 (Bits 8 to 15) to 0
  // Set VC (Bits 6 to 7) to 0 (Virtual Channel)
  // Set DT (Bits 0 to 5) to 0x21 (HSS)
  DEBUGASSERT(DSI_SYNC_HSS_REG == 0x1ca00b0);
  {
      const uint32_t ECC = 0x12 << 24;
      const uint32_t D1 = 0    << 16;
      const uint32_t D0 = 0    << 8;
      const uint32_t VC  = A64_MIPI_DSI_VIRTUAL_CHANNEL << 6;
      const uint32_t DT  = 0x21 << 0;
      const uint32_t DSI_SYNC_HSS = ECC
          | D1
          | D0
          | VC
          | DT;
      DEBUGASSERT(DSI_SYNC_HSS == 0x12000021);
      putreg32(DSI_SYNC_HSS, DSI_SYNC_HSS_REG);  // TODO: DMB
  }

  // DSI_SYNC_HSE_REG: DSI Offset 0xb4 (A31 Page 850)
  // Set ECC (Bits 24 to 31) to 1
  // Set D1 (Bits 16 to 23) to 0
  // Set D0 (Bits 8 to 15) to 0
  // Set VC (Bits 6 to 7) to 0 (Virtual Channel)
  // Set DT (Bits 0 to 5) to 0x31 (HSE)
  DEBUGASSERT(DSI_SYNC_HSE_REG == 0x1ca00b4);
  {
      const uint32_t ECC = 1    << 24;
      const uint32_t D1 = 0    << 16;
      const uint32_t D0 = 0    << 8;
      const uint32_t VC  = A64_MIPI_DSI_VIRTUAL_CHANNEL << 6;
      const uint32_t DT  = 0x31 << 0;
      const uint32_t DSI_SYNC_HSE = ECC
          | D1
          | D0
          | VC
          | DT;
      DEBUGASSERT(DSI_SYNC_HSE == 0x1000031);
      putreg32(DSI_SYNC_HSE, DSI_SYNC_HSE_REG);  // TODO: DMB
  }

  // DSI_SYNC_VSS_REG: DSI Offset 0xb8 (A31 Page 851)
  // Set ECC (Bits 24 to 31) to 7
  // Set D1 (Bits 16 to 23) to 0
  // Set D0 (Bits 8 to 15) to 0
  // Set VC (Bits 6 to 7) to 0 (Virtual Channel)
  // Set DT (Bits 0 to 5) to 1 (VSS)
  DEBUGASSERT(DSI_SYNC_VSS_REG == 0x1ca00b8);
  {
      const uint32_t ECC = 7 << 24;
      const uint32_t D1 = 0 << 16;
      const uint32_t D0 = 0 << 8;
      const uint32_t VC  = A64_MIPI_DSI_VIRTUAL_CHANNEL << 6;
      const uint32_t DT  = 1 << 0;
      const uint32_t DSI_SYNC_VSS = ECC
          | D1
          | D0
          | VC
          | DT;
      DEBUGASSERT(DSI_SYNC_VSS == 0x7000001);
      putreg32(DSI_SYNC_VSS, DSI_SYNC_VSS_REG);  // TODO: DMB
  }

  // DSI_SYNC_VSE_REG: DSI Offset 0xbc (A31 Page 851)
  // Set ECC (Bits 24 to 31) to 0x14
  // Set D1 (Bits 16 to 23) to 0
  // Set D0 (Bits 8 to 15) to 0
  // Set VC (Bits 6 to 7) to 0 (Virtual Channel)
  // Set DT (Bits 0 to 5) to 0x11 (VSE)
  DEBUGASSERT(DSI_SYNC_VSE_REG == 0x1ca00bc);
  {
      const uint32_t ECC = 0x14 << 24;
      const uint32_t D1 = 0    << 16;
      const uint32_t D0 = 0    << 8;
      const uint32_t VC  = A64_MIPI_DSI_VIRTUAL_CHANNEL << 6;
      const uint32_t DT  = 0x11 << 0;
      const uint32_t DSI_SYNC_VSE = ECC
          | D1
          | D0
          | VC
          | DT;
      DEBUGASSERT(DSI_SYNC_VSE == 0x14000011);
      putreg32(DSI_SYNC_VSE, DSI_SYNC_VSE_REG);  // TODO: DMB
  }

  // Set Basic Size (Undocumented)
  // DSI_BASIC_SIZE0_REG: DSI Offset 0x18
  // Set Video_VBP (Bits 16 to 27) to 17
  // Set Video_VSA (Bits 0 to 11) to 10
  ginfo("Set Basic Size\n");
  DEBUGASSERT(DSI_BASIC_SIZE0_REG == 0x1ca0018);

  const uint32_t DSI_BASIC_SIZE0 = Video_VBP(17)
      | Video_VSA(10);
  DEBUGASSERT(DSI_BASIC_SIZE0 == 0x11000a);
  putreg32(DSI_BASIC_SIZE0, DSI_BASIC_SIZE0_REG);  // TODO: DMB

  // DSI_BASIC_SIZE1_REG: DSI Offset 0x1c
  // Set Video_VT (Bits 16 to 28) to 1485
  // Set Video_VACT (Bits 0 to 11) to 1440
  DEBUGASSERT(DSI_BASIC_SIZE1_REG == 0x1ca001c);

  const uint32_t DSI_BASIC_SIZE1 = Video_VT(1485)
      | Video_VACT(1440);
  DEBUGASSERT(DSI_BASIC_SIZE1 == 0x5cd05a0);
  putreg32(DSI_BASIC_SIZE1, DSI_BASIC_SIZE1_REG);  // TODO: DMB

  // Set Horizontal Blanking
  // DSI_BLK_HSA0_REG: DSI Offset 0xc0 (A31 Page 852)
  // Set HSA_PH (Bits 0 to 31) to 0x900 4a19
  ginfo("Set Horizontal Blanking\n");
  DEBUGASSERT(DSI_BLK_HSA0_REG == 0x1ca00c0);
  const uint32_t DSI_BLK_HSA0 = 0x9004a19;
  putreg32(DSI_BLK_HSA0, DSI_BLK_HSA0_REG);  // TODO: DMB

  // DSI_BLK_HSA1_REG: DSI Offset 0xc4 (A31 Page 852)
  // Set HSA_PF (Bits 16 to 31) to 0x50b4
  // Set HSA_PD (Bits 0 to 7) to 0
  DEBUGASSERT(DSI_BLK_HSA1_REG == 0x1ca00c4);

  const uint32_t DSI_BLK_HSA1 = HSA_PF(0x50b4)
      | HSA_PD(0);
  DEBUGASSERT(DSI_BLK_HSA1 == 0x50b40000);
  putreg32(DSI_BLK_HSA1, DSI_BLK_HSA1_REG);  // TODO: DMB

  // DSI_BLK_HBP0_REG: DSI Offset 0xc8 (A31 Page 852)
  // Set HBP_PH (Bits 0 to 31) to 0x3500 5419
  DEBUGASSERT(DSI_BLK_HBP0_REG == 0x1ca00c8);
  putreg32(0x35005419, DSI_BLK_HBP0_REG);  // TODO: DMB

  // DSI_BLK_HBP1_REG: DSI Offset 0xcc (A31 Page 852)
  // Set HBP_PF (Bits 16 to 31) to 0x757a
  // Set HBP_PD (Bits 0 to 7) to 0
  DEBUGASSERT(DSI_BLK_HBP1_REG == 0x1ca00cc);

  const uint32_t DSI_BLK_HBP1 = HBP_PF(0x757a)
      | HBP_PD(0);
  DEBUGASSERT(DSI_BLK_HBP1 == 0x757a0000);
  putreg32(DSI_BLK_HBP1, DSI_BLK_HBP1_REG);  // TODO: DMB

  // DSI_BLK_HFP0_REG: DSI Offset 0xd0 (A31 Page 852)
  // Set HFP_PH (Bits 0 to 31) to 0x900 4a19
  DEBUGASSERT(DSI_BLK_HFP0_REG == 0x1ca00d0);
  putreg32(0x9004a19,  DSI_BLK_HFP0_REG);  // TODO: DMB

  // DSI_BLK_HFP1_REG: DSI Offset 0xd4 (A31 Page 853)
  // Set HFP_PF (Bits 16 to 31) to 0x50b4
  // Set HFP_PD (Bits 0 to 7) to 0
  DEBUGASSERT(DSI_BLK_HFP1_REG == 0x1ca00d4);

  const uint32_t DSI_BLK_HFP1 = HFP_PF(0x50b4)
      | HFP_PD(0);
  DEBUGASSERT(DSI_BLK_HFP1 == 0x50b40000);
  putreg32(DSI_BLK_HFP1, DSI_BLK_HFP1_REG);  // TODO: DMB

  // DSI_BLK_HBLK0_REG: DSI Offset 0xe0 (A31 Page 853)
  // Set HBLK_PH (Bits 0 to 31) to 0xc09 1a19
  DEBUGASSERT(DSI_BLK_HBLK0_REG == 0x1ca00e0);
  putreg32(0xc091a19,  DSI_BLK_HBLK0_REG);  // TODO: DMB

  // DSI_BLK_HBLK1_REG: DSI Offset 0xe4 (A31 Page 853)
  // Set HBLK_PF (Bits 16 to 31) to 0x72bd
  // Set HBLK_PD (Bits 0 to 7) to 0
  DEBUGASSERT(DSI_BLK_HBLK1_REG == 0x1ca00e4);

  const uint32_t DSI_BLK_HBLK1 = HBLK_PF(0x72bd)
      | HBLK_PD(0);
  DEBUGASSERT(DSI_BLK_HBLK1 == 0x72bd0000);
  putreg32(DSI_BLK_HBLK1, DSI_BLK_HBLK1_REG);  // TODO: DMB

  // Set Vertical Blanking
  // DSI_BLK_VBLK0_REG: DSI Offset 0xe8 (A31 Page 854)
  // Set VBLK_PH (Bits 0 to 31) to 0x1a00 0019
  ginfo("Set Vertical Blanking\n");
  DEBUGASSERT(DSI_BLK_VBLK0_REG == 0x1ca00e8);
  putreg32(0x1a000019, DSI_BLK_VBLK0_REG);  // TODO: DMB

  // DSI_BLK_VBLK1_REG: DSI Offset 0xec (A31 Page 854)
  // Set VBLK_PF (Bits 16 to 31) to 0xffff
  // Set VBLK_PD (Bits 0 to 7) to 0
  DEBUGASSERT(DSI_BLK_VBLK1_REG == 0x1ca00ec);

  const uint32_t DSI_BLK_VBLK1 = VBLK_PF(0xffff)
      | VBLK_PD(0);
  DEBUGASSERT(DSI_BLK_VBLK1 == 0xffff0000);
  putreg32(DSI_BLK_VBLK1, DSI_BLK_VBLK1_REG);  // TODO: DMB

  return OK;
}

/// Start MIPI DSI HSC and HSD. (High Speed Clock Mode and High Speed Data Transmission)
/// Based on https://lupyuen.github.io/articles/dsi#appendix-start-mipi-dsi-hsc-and-hsd
int a64_mipi_dsi_start(void)
{
  // Start HSC (Undocumented)
  // DSI_INST_JUMP_SEL_REG: DSI Offset 0x48
  // Set to 0xf02
  ginfo("Start HSC\n");
  DEBUGASSERT(DSI_INST_JUMP_SEL_REG == 0x1ca0048);
  putreg32(0xf02, DSI_INST_JUMP_SEL_REG);  // TODO: DMB

  // Commit
  // DSI_BASIC_CTL0_REG: DSI Offset 0x10 (A31 Page 845)
  // Set INSTRU_EN (Bit 0) to 1 (Enable DSI Processing from Instruction 0)
  ginfo("Commit\n");
  DEBUGASSERT(DSI_BASIC_CTL0_REG == 0x1ca0010);
  DEBUGASSERT(INSTRU_EN == 0x1);
  modreg32(INSTRU_EN, INSTRU_EN, DSI_BASIC_CTL0_REG);  // TODO: DMB

  // (DSI_INST_FUNC_REG(n) is (0x020 + (n) * 0x04))

  // Instruction Function Lane (Undocumented)
  // DSI_INST_FUNC_REG(0): DSI Offset 0x20
  // Set DSI_INST_FUNC_LANE_CEN (Bit 4) to 0
  // Index 0 is DSI_INST_ID_LP11
  ginfo("Instruction Function Lane\n");
  DEBUGASSERT(DSI_INST_FUNC_REG(0) == 0x1ca0020);

  DEBUGASSERT(DSI_INST_FUNC_LANE_CEN == 0x10);
  modreg32(0x0, DSI_INST_FUNC_LANE_CEN, DSI_INST_FUNC_REG(0) );  // TODO: DMB

  // Wait 1 millisecond
  up_mdelay(1);

  // Start HSD (Undocumented)
  // DSI_INST_JUMP_SEL_REG: DSI Offset 0x48
  // Set to 0x63f0 7006
  ginfo("Start HSD\n");
  DEBUGASSERT(DSI_INST_JUMP_SEL_REG == 0x1ca0048);
  putreg32(0x63f07006, DSI_INST_JUMP_SEL_REG);  // TODO: DMB

  // Commit
  // DSI_BASIC_CTL0_REG: DSI Offset 0x10 (A31 Page 845)
  // Set INSTRU_EN (Bit 0) to 1 (Enable DSI Processing from Instruction 0)
  ginfo("Commit\n");
  DEBUGASSERT(DSI_BASIC_CTL0_REG == 0x1ca0010);
  DEBUGASSERT(INSTRU_EN == 0x1);
  modreg32(INSTRU_EN, INSTRU_EN, DSI_BASIC_CTL0_REG);  // TODO: DMB

  return OK;
}

//// TODO: Remove this test code
#include "../../pinephone-nuttx/test/test_a64_mipi_dsi.c" //// TODO
