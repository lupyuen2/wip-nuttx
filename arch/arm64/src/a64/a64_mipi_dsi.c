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

/// MIPI DSI Virtual Channel
#define VIRTUAL_CHANNEL 0

/// Base Address of Allwinner A64 CCU Controller (A64 Page 82)
#define CCU_BASE_ADDRESS 0x01C20000

/// Base Address of Allwinner A64 MIPI DSI Controller (A31 Page 842)
#define DSI_BASE_ADDRESS 0x01CA0000

/// Instru_En is Bit 0 of DSI_BASIC_CTL0_REG 
/// (DSI Configuration Register 0) at Offset 0x10
#define DSI_BASIC_CTL0_REG (DSI_BASE_ADDRESS + 0x10)
#define Instru_En (1 << 0)

/// DSI_INST_FUNC_REG(n) is (0x020 + (n) * 0x04)
#define DSI_INST_FUNC_REG(n) (DSI_BASE_ADDRESS + (0x020 + n * 0x04))

/// DSI_INST_JUMP_CFG_REG(n) is (0x04c + (n) * 0x04)
#define DSI_INST_JUMP_CFG_REG(n) (DSI_BASE_ADDRESS + (0x04c + n * 0x04))

/// DSI_INST_LOOP_NUM_REG(n) is (0x044 + (n) * 0x10)
#define DSI_INST_LOOP_NUM_REG(n) (DSI_BASE_ADDRESS + (0x044 + n * 0x10))

/************************************************************************************************
 * Private Data
 ************************************************************************************************/

/************************************************************************************************
 * Private Functions
 ************************************************************************************************/

/// Disable DSI Processing. See https://lupyuen.github.io/articles/dsi#transmit-packet-over-mipi-dsi
static void a64_disable_dsi_processing(void)
{
    // Set Instru_En to 0
    modreg32(0, Instru_En, DSI_BASIC_CTL0_REG);  // TODO: DMB
}

/// Enable DSI Processing. See https://lupyuen.github.io/articles/dsi#transmit-packet-over-mipi-dsi
static void a64_enable_dsi_processing(void)
{
  // Set Instru_En to 1
  modreg32(Instru_En, Instru_En, DSI_BASIC_CTL0_REG);  // TODO: DMB
}

/// Wait for transmit to complete. Returns 0 if completed, -1 if timeout.
/// See https://lupyuen.github.io/articles/dsi#transmit-packet-over-mipi-dsi
static int a64_wait_dsi_transmit(void)
{
  // Wait up to 5,000 microseconds
  int i;
  for (i = 0; i < 5000; i++)  // TODO
    {
      // To check whether the transmission is complete, we poll on Instru_En
      if ((getreg32(DSI_BASIC_CTL0_REG) & Instru_En) == 0)
        {
          // If Instru_En is 0, then transmission is complete
          return 0;
        }
      // Sleep 1 microsecond
      up_udelay(1);  // TODO
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
  _info("channel=%d, cmd=0x%x, txlen=%d\n", channel, cmd, (int) txlen); // TODO
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
  _info("packet: pktlen=%d\n", (int) pktlen); // TODO
  void dump_buffer(const uint8_t *data, size_t len); ////
  dump_buffer(pkt, pktlen);

  // Set the following bits to 1 in DSI_CMD_CTL_REG (DSI Low Power Control Register) at Offset 0x200:
  // RX_Overflow (Bit 26): Clear flag for "Receive Overflow"
  // RX_Flag (Bit 25): Clear flag for "Receive has started"
  // TX_Flag (Bit 9): Clear flag for "Transmit has started"
  // All other bits must be set to 0.
  const uint64_t DSI_CMD_CTL_REG = DSI_BASE_ADDRESS + 0x200;
  const uint32_t RX_Overflow = 1 << 26;
  const uint32_t RX_Flag     = 1 << 25;
  const uint32_t TX_Flag     = 1 << 9;
  putreg32(
      RX_Overflow | RX_Flag | TX_Flag,
      DSI_CMD_CTL_REG
  );

  // Write the Long Packet to DSI_CMD_TX_REG 
  // (DSI Low Power Transmit Package Register) at Offset 0x300 to 0x3FC
  const uint64_t DSI_CMD_TX_REG = DSI_BASE_ADDRESS + 0x300;
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
      DEBUGASSERT(addr <= DSI_BASE_ADDRESS + 0x3FC);
      modreg32(v, 0xFFFFFFFF, addr);  // TODO: DMB
      addr += 4;      
    }

  // Set Packet Length - 1 in Bits 0 to 7 (TX_Size) of
  // DSI_CMD_CTL_REG (DSI Low Power Control Register) at Offset 0x200
  modreg32(pktlen - 1, 0xFF, DSI_CMD_CTL_REG);  // TODO: DMB

  // Set DSI_INST_JUMP_SEL_REG (Offset 0x48, undocumented) 
  // to begin the Low Power Transmission (LPTX)
  const uint64_t DSI_INST_JUMP_SEL_REG = DSI_BASE_ADDRESS + 0x48;
  const uint64_t DSI_INST_ID_LPDT = 4;
  const uint64_t DSI_INST_ID_LP11 = 0;
  const uint64_t DSI_INST_ID_END  = 15;
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
  _info("Enable MIPI DSI Bus\n");
  const uint64_t BUS_CLK_GATING_REG0 = CCU_BASE_ADDRESS + 0x60;
  DEBUGASSERT(BUS_CLK_GATING_REG0 == 0x1c20060);

  const uint32_t MIPIDSI_GATING = 1 << 1;
  DEBUGASSERT(MIPIDSI_GATING == 2);
  modreg32(MIPIDSI_GATING, MIPIDSI_GATING, BUS_CLK_GATING_REG0);  // TODO: DMB

  // BUS_SOFT_RST_REG0: CCU Offset 0x2C0 (A64 Page 138)
  // Set MIPI_DSI_RST (Bit 1) to 1 (Deassert MIPI DSI Reset)
  const uint64_t BUS_SOFT_RST_REG0 = CCU_BASE_ADDRESS + 0x2C0;
  DEBUGASSERT(BUS_SOFT_RST_REG0 == 0x1c202c0);

  const uint32_t MIPI_DSI_RST = 1 << 1;
  DEBUGASSERT(MIPI_DSI_RST == 2);
  modreg32(MIPI_DSI_RST, MIPI_DSI_RST, BUS_SOFT_RST_REG0);  // TODO: DMB

  // Enable DSI Block
  // DSI_CTL_REG: DSI Offset 0x0 (A31 Page 843)
  // Set DSI_En (Bit 0) to 1 (Enable DSI)
  _info("Enable DSI Block\n");
  const uint64_t DSI_CTL_REG = DSI_BASE_ADDRESS + 0x0;
  DEBUGASSERT(DSI_CTL_REG == 0x1ca0000);

  const uint32_t DSI_En = 1 << 0;
  DEBUGASSERT(DSI_En == 1);
  putreg32(DSI_En, DSI_CTL_REG);  // TODO: DMB

  // DSI_BASIC_CTL0_REG: DSI Offset 0x10 (A31 Page 845)
  // Set CRC_En (Bit 17) to 1 (Enable CRC)
  // Set ECC_En (Bit 16) to 1 (Enable ECC)
  DEBUGASSERT(DSI_BASIC_CTL0_REG == 0x1ca0010);

  const uint32_t CRC_En = 1 << 17;
  const uint32_t ECC_En = 1 << 16;
  const uint32_t DSI_BASIC_CTL0 = CRC_En
      | ECC_En;
  DEBUGASSERT(DSI_BASIC_CTL0 == 0x30000);
  putreg32(DSI_BASIC_CTL0, DSI_BASIC_CTL0_REG);  // TODO: DMB

  // DSI_TRANS_START_REG: DSI Offset 0x60 (Undocumented)
  // Set to 10
  const uint64_t DSI_TRANS_START_REG = DSI_BASE_ADDRESS + 0x60;
  DEBUGASSERT(DSI_TRANS_START_REG == 0x1ca0060);
  putreg32(10, DSI_TRANS_START_REG);  // TODO: DMB

  // DSI_TRANS_ZERO_REG: DSI Offset 0x78 (Undocumented)
  // Set to 0
  const uint64_t DSI_TRANS_ZERO_REG = DSI_BASE_ADDRESS + 0x78;
  DEBUGASSERT(DSI_TRANS_ZERO_REG == 0x1ca0078);
  putreg32(0, DSI_TRANS_ZERO_REG);  // TODO: DMB

  // Set Instructions (Undocumented)
  // DSI_INST_FUNC_REG(0): DSI Offset 0x20
  // Set to 0x1f
  // Index 0 is DSI_INST_ID_LP11
  _info("Set Instructions\n");
  const uint32_t DSI_INST_ID_LP11 = 0;
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_LP11) == 0x1ca0020);
  putreg32(0x1f, DSI_INST_FUNC_REG(DSI_INST_ID_LP11));  // TODO: DMB

  // DSI_INST_FUNC_REG(1): DSI Offset 0x24
  // Set to 0x1000 0001
  // Index 1 is DSI_INST_ID_TBA
  const uint32_t DSI_INST_ID_TBA = 1;
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_TBA) == 0x1ca0024);
  putreg32(0x10000001, DSI_INST_FUNC_REG(DSI_INST_ID_TBA));  // TODO: DMB

  // DSI_INST_FUNC_REG(2): DSI Offset 0x28
  // Set to 0x2000 0010
  // Index 2 is DSI_INST_ID_HSC
  const uint32_t DSI_INST_ID_HSC = 2;
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_HSC) == 0x1ca0028);
  putreg32(0x20000010, DSI_INST_FUNC_REG(DSI_INST_ID_HSC));  // TODO: DMB

  // DSI_INST_FUNC_REG(3): DSI Offset 0x2c
  // Set to 0x2000 000f
  // Index 3 is DSI_INST_ID_HSD
  const uint32_t DSI_INST_ID_HSD = 3;
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_HSD) == 0x1ca002c);
  putreg32(0x2000000f, DSI_INST_FUNC_REG(DSI_INST_ID_HSD));  // TODO: DMB

  // DSI_INST_FUNC_REG(4): DSI Offset 0x30
  // Set to 0x3010 0001
  // Index 4 is DSI_INST_ID_LPDT
  const uint32_t DSI_INST_ID_LPDT = 4;
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_LPDT) == 0x1ca0030);
  putreg32(0x30100001, DSI_INST_FUNC_REG(DSI_INST_ID_LPDT));  // TODO: DMB

  // DSI_INST_FUNC_REG(5): DSI Offset 0x34
  // Set to 0x4000 0010
  // Index 5 is DSI_INST_ID_HSCEXIT
  const uint32_t DSI_INST_ID_HSCEXIT = 5;
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_HSCEXIT) == 0x1ca0034);
  putreg32(0x40000010, DSI_INST_FUNC_REG(DSI_INST_ID_HSCEXIT));  // TODO: DMB

  // DSI_INST_FUNC_REG(6): DSI Offset 0x38
  // Set to 0xf
  // Index 6 is DSI_INST_ID_NOP
  const uint32_t DSI_INST_ID_NOP = 6;
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_NOP) == 0x1ca0038);
  putreg32(0xf, DSI_INST_FUNC_REG(DSI_INST_ID_NOP));  // TODO: DMB

  // DSI_INST_FUNC_REG(7): DSI Offset 0x3c
  // Set to 0x5000 001f
  // Index 7 is DSI_INST_ID_DLY
  const uint32_t DSI_INST_ID_DLY = 7;
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_DLY) == 0x1ca003c);
  putreg32(0x5000001f, DSI_INST_FUNC_REG(DSI_INST_ID_DLY));  // TODO: DMB

  // Configure Jump Instructions (Undocumented)
  // DSI_INST_JUMP_CFG_REG(0): DSI Offset 0x4c
  // Set to 0x56 0001    
  // Index 0 is DSI_INST_JUMP_CFG
  _info("Configure Jump Instructions\n");
  const uint32_t DSI_INST_JUMP_CFG = 0;
  DEBUGASSERT(DSI_INST_JUMP_CFG_REG(DSI_INST_JUMP_CFG) == 0x1ca004c);
  putreg32(0x560001, DSI_INST_JUMP_CFG_REG(DSI_INST_JUMP_CFG));  // TODO: DMB

  // DSI_DEBUG_DATA_REG: DSI Offset 0x2f8
  // Set to 0xff
  const uint64_t DSI_DEBUG_DATA_REG = DSI_BASE_ADDRESS + 0x2f8;
  DEBUGASSERT(DSI_DEBUG_DATA_REG == 0x1ca02f8);
  putreg32(0xff, DSI_DEBUG_DATA_REG);  // TODO: DMB

  // Set Video Start Delay
  // DSI_BASIC_CTL1_REG: DSI Offset 0x14 (A31 Page 846)
  // Set Video_Start_Delay (Bits 4 to 16) to 1468 (Line Delay)
  // Set Video_Precision_Mode_Align (Bit 2) to 1 (Fill Mode)
  // Set Video_Frame_Start (Bit 1) to 1 (Precision Mode)
  // Set DSI_Mode (Bit 0) to 1 (Video Mode)
  // TODO: Video_Start_Delay is actually 13 bits, not 8 bits as documented in the A31 User Manual
  _info("Set Video Start Delay\n");
  const uint64_t DSI_BASIC_CTL1_REG = DSI_BASE_ADDRESS + 0x14;
  DEBUGASSERT(DSI_BASIC_CTL1_REG == 0x1ca0014);

  const uint32_t Video_Start_Delay = 1468 << 4;
  const uint32_t Video_Precision_Mode_Align  = 1    << 2;
  const uint32_t Video_Frame_Start  = 1    << 1;
  const uint32_t DSI_Mode  = 1    << 0;
  const uint32_t DSI_BASIC_CTL1 = Video_Start_Delay
      | Video_Precision_Mode_Align
      | Video_Frame_Start
      | DSI_Mode;
  DEBUGASSERT(DSI_BASIC_CTL1 == 0x5bc7);
  putreg32(DSI_BASIC_CTL1, DSI_BASIC_CTL1_REG);  // TODO: DMB

  // Set Burst (Undocumented)
  // DSI_TCON_DRQ_REG: DSI Offset 0x7c
  // Set to 0x1000 0007
  _info("Set Burst\n");
  const uint64_t DSI_TCON_DRQ_REG = DSI_BASE_ADDRESS + 0x7c;
  DEBUGASSERT(DSI_TCON_DRQ_REG == 0x1ca007c);
  putreg32(0x10000007, DSI_TCON_DRQ_REG);  // TODO: DMB

  // Set Instruction Loop (Undocumented)
  // DSI_INST_LOOP_SEL_REG: DSI Offset 0x40
  // Set to 0x3000 0002
  _info("Set Instruction Loop\n");
  const uint64_t DSI_INST_LOOP_SEL_REG = DSI_BASE_ADDRESS + 0x40;
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
  _info("Set Pixel Format\n");
  const uint64_t DSI_PIXEL_PH_REG = DSI_BASE_ADDRESS + 0x90;
  DEBUGASSERT(DSI_PIXEL_PH_REG == 0x1ca0090);
  {
      const uint32_t ECC = 19   << 24;
      const uint32_t WC = 2160 << 8;
      const uint32_t VC  = VIRTUAL_CHANNEL << 6;
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
  const uint64_t DSI_PIXEL_PF0_REG = DSI_BASE_ADDRESS + 0x98;
  DEBUGASSERT(DSI_PIXEL_PF0_REG == 0x1ca0098);
  const uint32_t CRC_Force = 0xffff;
  putreg32(CRC_Force, DSI_PIXEL_PF0_REG);  // TODO: DMB

  // DSI_PIXEL_PF1_REG: DSI Offset 0x9c (A31 Page 849)
  // Set CRC_Init_LineN (Bits 16 to 31) to 0xffff (CRC initial to this value in transmitions except 1st one)
  // Set CRC_Init_Line0 (Bits 0 to 15) to 0xffff (CRC initial to this value in 1st transmition every frame)
  const uint64_t DSI_PIXEL_PF1_REG = DSI_BASE_ADDRESS + 0x9c;
  DEBUGASSERT(DSI_PIXEL_PF1_REG == 0x1ca009c);

  const uint32_t CRC_Init_LineN = 0xffff << 16;
  const uint32_t CRC_Init_Line0 = 0xffff << 0;
  const uint32_t DSI_PIXEL_PF1 = CRC_Init_LineN
      | CRC_Init_Line0;
  DEBUGASSERT(DSI_PIXEL_PF1 == 0xffffffff);
  putreg32(DSI_PIXEL_PF1, DSI_PIXEL_PF1_REG);  // TODO: DMB

  // DSI_PIXEL_CTL0_REG: DSI Offset 0x80 (A31 Page 847)
  // Set PD_Plug_Dis (Bit 16) to 1 (Disable PD plug before pixel bytes)
  // Set Pixel_Endian (Bit 4) to 0 (LSB first)
  // Set Pixel_Format (Bits 0 to 3) to 8 (24-bit RGB888)
  const uint64_t DSI_PIXEL_CTL0_REG = DSI_BASE_ADDRESS + 0x80;
  DEBUGASSERT(DSI_PIXEL_CTL0_REG == 0x1ca0080);

  const uint32_t PD_Plug_Dis = 1 << 16;
  const uint32_t Pixel_Endian  = 0 << 4;
  const uint32_t Pixel_Format  = 8 << 0;
  const uint32_t DSI_PIXEL_CTL0 = PD_Plug_Dis
      | Pixel_Endian
      | Pixel_Format;
  DEBUGASSERT(DSI_PIXEL_CTL0 == 0x10008);
  putreg32(DSI_PIXEL_CTL0, DSI_PIXEL_CTL0_REG);  // TODO: DMB

  // Set Sync Timings
  // DSI_BASIC_CTL_REG: DSI Offset 0x0c (Undocumented)
  // Set to 0
  _info("Set Sync Timings\n");
  const uint64_t DSI_BASIC_CTL_REG = DSI_BASE_ADDRESS + 0x0c;
  DEBUGASSERT(DSI_BASIC_CTL_REG == 0x1ca000c);
  putreg32(0x0, DSI_BASIC_CTL_REG);  // TODO: DMB

  // DSI_SYNC_HSS_REG: DSI Offset 0xb0 (A31 Page 850)
  // Set ECC (Bits 24 to 31) to 0x12
  // Set D1 (Bits 16 to 23) to 0
  // Set D0 (Bits 8 to 15) to 0
  // Set VC (Bits 6 to 7) to 0 (Virtual Channel)
  // Set DT (Bits 0 to 5) to 0x21 (HSS)
  const uint64_t DSI_SYNC_HSS_REG = DSI_BASE_ADDRESS + 0xb0;
  DEBUGASSERT(DSI_SYNC_HSS_REG == 0x1ca00b0);
  {
      const uint32_t ECC = 0x12 << 24;
      const uint32_t D1 = 0    << 16;
      const uint32_t D0 = 0    << 8;
      const uint32_t VC  = VIRTUAL_CHANNEL << 6;
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
  const uint64_t DSI_SYNC_HSE_REG = DSI_BASE_ADDRESS + 0xb4;
  DEBUGASSERT(DSI_SYNC_HSE_REG == 0x1ca00b4);
  {
      const uint32_t ECC = 1    << 24;
      const uint32_t D1 = 0    << 16;
      const uint32_t D0 = 0    << 8;
      const uint32_t VC  = VIRTUAL_CHANNEL << 6;
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
  const uint64_t DSI_SYNC_VSS_REG = DSI_BASE_ADDRESS + 0xb8;
  DEBUGASSERT(DSI_SYNC_VSS_REG == 0x1ca00b8);
  {
      const uint32_t ECC = 7 << 24;
      const uint32_t D1 = 0 << 16;
      const uint32_t D0 = 0 << 8;
      const uint32_t VC  = VIRTUAL_CHANNEL << 6;
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
  const uint64_t DSI_SYNC_VSE_REG = DSI_BASE_ADDRESS + 0xbc;
  DEBUGASSERT(DSI_SYNC_VSE_REG == 0x1ca00bc);
  {
      const uint32_t ECC = 0x14 << 24;
      const uint32_t D1 = 0    << 16;
      const uint32_t D0 = 0    << 8;
      const uint32_t VC  = VIRTUAL_CHANNEL << 6;
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
  _info("Set Basic Size\n");
  const uint64_t DSI_BASIC_SIZE0_REG = DSI_BASE_ADDRESS + 0x18;
  DEBUGASSERT(DSI_BASIC_SIZE0_REG == 0x1ca0018);

  const uint32_t Video_VBP = 17 << 16;
  const uint32_t Video_VSA = 10 << 0;
  const uint32_t DSI_BASIC_SIZE0 = Video_VBP
      | Video_VSA;
  DEBUGASSERT(DSI_BASIC_SIZE0 == 0x11000a);
  putreg32(DSI_BASIC_SIZE0, DSI_BASIC_SIZE0_REG);  // TODO: DMB

  // DSI_BASIC_SIZE1_REG: DSI Offset 0x1c
  // Set Video_VT (Bits 16 to 28) to 1485
  // Set Video_VACT (Bits 0 to 11) to 1440
  const uint64_t DSI_BASIC_SIZE1_REG = DSI_BASE_ADDRESS + 0x1c;
  DEBUGASSERT(DSI_BASIC_SIZE1_REG == 0x1ca001c);

  const uint32_t Video_VT = 1485 << 16;
  const uint32_t Video_VACT = 1440 << 0;
  const uint32_t DSI_BASIC_SIZE1 = Video_VT
      | Video_VACT;
  DEBUGASSERT(DSI_BASIC_SIZE1 == 0x5cd05a0);
  putreg32(DSI_BASIC_SIZE1, DSI_BASIC_SIZE1_REG);  // TODO: DMB

  // Set Horizontal Blanking
  // DSI_BLK_HSA0_REG: DSI Offset 0xc0 (A31 Page 852)
  // Set HSA_PH (Bits 0 to 31) to 0x900 4a19
  _info("Set Horizontal Blanking\n");
  const uint64_t DSI_BLK_HSA0_REG = DSI_BASE_ADDRESS + 0xc0;
  DEBUGASSERT(DSI_BLK_HSA0_REG == 0x1ca00c0);
  const uint32_t DSI_BLK_HSA0 = 0x9004a19;
  putreg32(DSI_BLK_HSA0, DSI_BLK_HSA0_REG);  // TODO: DMB

  // DSI_BLK_HSA1_REG: DSI Offset 0xc4 (A31 Page 852)
  // Set HSA_PF (Bits 16 to 31) to 0x50b4
  // Set HSA_PD (Bits 0 to 7) to 0
  const uint64_t DSI_BLK_HSA1_REG = DSI_BASE_ADDRESS + 0xc4;
  DEBUGASSERT(DSI_BLK_HSA1_REG == 0x1ca00c4);

  const uint32_t HSA_PF = 0x50b4 << 16;
  const uint32_t HSA_PD  = 0      << 0;
  const uint32_t DSI_BLK_HSA1 = HSA_PF
      | HSA_PD;
  DEBUGASSERT(DSI_BLK_HSA1 == 0x50b40000);
  putreg32(DSI_BLK_HSA1, DSI_BLK_HSA1_REG);  // TODO: DMB

  // DSI_BLK_HBP0_REG: DSI Offset 0xc8 (A31 Page 852)
  // Set HBP_PH (Bits 0 to 31) to 0x3500 5419
  const uint64_t DSI_BLK_HBP0_REG = DSI_BASE_ADDRESS + 0xc8;
  DEBUGASSERT(DSI_BLK_HBP0_REG == 0x1ca00c8);
  putreg32(0x35005419, DSI_BLK_HBP0_REG);  // TODO: DMB

  // DSI_BLK_HBP1_REG: DSI Offset 0xcc (A31 Page 852)
  // Set HBP_PF (Bits 16 to 31) to 0x757a
  // Set HBP_PD (Bits 0 to 7) to 0
  const uint64_t DSI_BLK_HBP1_REG = DSI_BASE_ADDRESS + 0xcc;
  DEBUGASSERT(DSI_BLK_HBP1_REG == 0x1ca00cc);

  const uint32_t HBP_PF = 0x757a << 16;
  const uint32_t HBP_PD  = 0      << 0;
  const uint32_t DSI_BLK_HBP1 = HBP_PF
      | HBP_PD;
  DEBUGASSERT(DSI_BLK_HBP1 == 0x757a0000);
  putreg32(DSI_BLK_HBP1, DSI_BLK_HBP1_REG);  // TODO: DMB

  // DSI_BLK_HFP0_REG: DSI Offset 0xd0 (A31 Page 852)
  // Set HFP_PH (Bits 0 to 31) to 0x900 4a19
  const uint64_t DSI_BLK_HFP0_REG = DSI_BASE_ADDRESS + 0xd0;
  DEBUGASSERT(DSI_BLK_HFP0_REG == 0x1ca00d0);
  putreg32(0x9004a19,  DSI_BLK_HFP0_REG);  // TODO: DMB

  // DSI_BLK_HFP1_REG: DSI Offset 0xd4 (A31 Page 853)
  // Set HFP_PF (Bits 16 to 31) to 0x50b4
  // Set HFP_PD (Bits 0 to 7) to 0
  const uint64_t DSI_BLK_HFP1_REG = DSI_BASE_ADDRESS + 0xd4;
  DEBUGASSERT(DSI_BLK_HFP1_REG == 0x1ca00d4);

  const uint32_t HFP_PF = 0x50b4 << 16;
  const uint32_t HFP_PD  = 0      << 0;
  const uint32_t DSI_BLK_HFP1 = HFP_PF
      | HFP_PD;
  DEBUGASSERT(DSI_BLK_HFP1 == 0x50b40000);
  putreg32(DSI_BLK_HFP1, DSI_BLK_HFP1_REG);  // TODO: DMB

  // DSI_BLK_HBLK0_REG: DSI Offset 0xe0 (A31 Page 853)
  // Set HBLK_PH (Bits 0 to 31) to 0xc09 1a19
  const uint64_t DSI_BLK_HBLK0_REG = DSI_BASE_ADDRESS + 0xe0;
  DEBUGASSERT(DSI_BLK_HBLK0_REG == 0x1ca00e0);
  putreg32(0xc091a19,  DSI_BLK_HBLK0_REG);  // TODO: DMB

  // DSI_BLK_HBLK1_REG: DSI Offset 0xe4 (A31 Page 853)
  // Set HBLK_PF (Bits 16 to 31) to 0x72bd
  // Set HBLK_PD (Bits 0 to 7) to 0
  const uint64_t DSI_BLK_HBLK1_REG = DSI_BASE_ADDRESS + 0xe4;
  DEBUGASSERT(DSI_BLK_HBLK1_REG == 0x1ca00e4);

  const uint32_t HBLK_PF = 0x72bd << 16;
  const uint32_t HBLK_PD  = 0      << 0;
  const uint32_t DSI_BLK_HBLK1 = HBLK_PF
      | HBLK_PD;
  DEBUGASSERT(DSI_BLK_HBLK1 == 0x72bd0000);
  putreg32(DSI_BLK_HBLK1, DSI_BLK_HBLK1_REG);  // TODO: DMB

  // Set Vertical Blanking
  // DSI_BLK_VBLK0_REG: DSI Offset 0xe8 (A31 Page 854)
  // Set VBLK_PH (Bits 0 to 31) to 0x1a00 0019
  _info("Set Vertical Blanking\n");
  const uint64_t DSI_BLK_VBLK0_REG = DSI_BASE_ADDRESS + 0xe8;
  DEBUGASSERT(DSI_BLK_VBLK0_REG == 0x1ca00e8);
  putreg32(0x1a000019, DSI_BLK_VBLK0_REG);  // TODO: DMB

  // DSI_BLK_VBLK1_REG: DSI Offset 0xec (A31 Page 854)
  // Set VBLK_PF (Bits 16 to 31) to 0xffff
  // Set VBLK_PD (Bits 0 to 7) to 0
  const uint64_t DSI_BLK_VBLK1_REG = DSI_BASE_ADDRESS + 0xec;
  DEBUGASSERT(DSI_BLK_VBLK1_REG == 0x1ca00ec);

  const uint32_t VBLK_PF = 0xffff << 16;
  const uint32_t VBLK_PD  = 0      << 0;
  const uint32_t DSI_BLK_VBLK1 = VBLK_PF
      | VBLK_PD;
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
  _info("Start HSC\n");
  const uint64_t DSI_INST_JUMP_SEL_REG = DSI_BASE_ADDRESS + 0x48;
  DEBUGASSERT(DSI_INST_JUMP_SEL_REG == 0x1ca0048);
  putreg32(0xf02, DSI_INST_JUMP_SEL_REG);  // TODO: DMB

  // Commit
  // DSI_BASIC_CTL0_REG: DSI Offset 0x10 (A31 Page 845)
  // Set Instru_En (Bit 0) to 1 (Enable DSI Processing from Instruction 0)
  _info("Commit\n");
  DEBUGASSERT(DSI_BASIC_CTL0_REG == 0x1ca0010);
  DEBUGASSERT(Instru_En == 0x1);
  modreg32(Instru_En, Instru_En, DSI_BASIC_CTL0_REG);  // TODO: DMB

  // (DSI_INST_FUNC_REG(n) is (0x020 + (n) * 0x04))

  // Instruction Function Lane (Undocumented)
  // DSI_INST_FUNC_REG(0): DSI Offset 0x20
  // Set DSI_INST_FUNC_LANE_CEN (Bit 4) to 0
  // Index 0 is DSI_INST_ID_LP11
  _info("Instruction Function Lane\n");
  DEBUGASSERT(DSI_INST_FUNC_REG(0) == 0x1ca0020);

  const uint32_t DSI_INST_FUNC_LANE_CEN = 1 << 4;
  DEBUGASSERT(DSI_INST_FUNC_LANE_CEN == 0x10);
  modreg32(0x0, DSI_INST_FUNC_LANE_CEN, DSI_INST_FUNC_REG(0) );  // TODO: DMB

  // Wait 1,000 microseconds
  up_udelay(1000);

  // Start HSD (Undocumented)
  // DSI_INST_JUMP_SEL_REG: DSI Offset 0x48
  // Set to 0x63f0 7006
  _info("Start HSD\n");
  DEBUGASSERT(DSI_INST_JUMP_SEL_REG == 0x1ca0048);
  putreg32(0x63f07006, DSI_INST_JUMP_SEL_REG);  // TODO: DMB

  // Commit
  // DSI_BASIC_CTL0_REG: DSI Offset 0x10 (A31 Page 845)
  // Set Instru_En (Bit 0) to 1 (Enable DSI Processing from Instruction 0)
  _info("Commit\n");
  DEBUGASSERT(DSI_BASIC_CTL0_REG == 0x1ca0010);
  DEBUGASSERT(Instru_En == 0x1);
  modreg32(Instru_En, Instru_En, DSI_BASIC_CTL0_REG);  // TODO: DMB

  return OK;
}

///////////////////////////////////////////////////////////////////////////////
// TODO: Move to nuttx/arch/arm64/src/common/arm64_udelay.c
// Based on nuttx/arch/arm/src/common/arm_udelay.c

#define CONFIG_BOARD_LOOPSPER100USEC ((CONFIG_BOARD_LOOPSPERMSEC+5)/10)
#define CONFIG_BOARD_LOOPSPER10USEC  ((CONFIG_BOARD_LOOPSPERMSEC+50)/100)
#define CONFIG_BOARD_LOOPSPERUSEC    ((CONFIG_BOARD_LOOPSPERMSEC+500)/1000)

/****************************************************************************
 * Name: up_udelay
 *
 * Description:
 *   Delay inline for the requested number of microseconds.  NOTE:  Because
 *   of all of the setup, several microseconds will be lost before the actual
 *   timing loop begins.  Thus, the delay will always be a few microseconds
 *   longer than requested.
 *
 *   *** NOT multi-tasking friendly ***
 *
 * ASSUMPTIONS:
 *   The setting CONFIG_BOARD_LOOPSPERMSEC has been calibrated
 *
 ****************************************************************************/

void up_udelay(useconds_t microseconds)
{
  volatile int i;

  /* We'll do this a little at a time because we expect that the
   * CONFIG_BOARD_LOOPSPERUSEC is very inaccurate during to truncation in
   * the divisions of its calculation.  We'll use the largest values that
   * we can in order to prevent significant error buildup in the loops.
   */

  while (microseconds > 1000)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPERMSEC; i++)
        {
        }

      microseconds -= 1000;
    }

  while (microseconds > 100)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPER100USEC; i++)
        {
        }

      microseconds -= 100;
    }

  while (microseconds > 10)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPER10USEC; i++)
        {
        }

      microseconds -= 10;
    }

  while (microseconds > 0)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPERUSEC; i++)
        {
        }

      microseconds--;
    }
}

///////////////////////////////////////////////////////////////////////////////
// TODO: Remove this test code

void dump_buffer(const uint8_t *data, size_t len);

/// Write the DCS Command to MIPI DSI
static int write_dcs(FAR const uint8_t *buf, size_t len)
{
  int ret = -1;
  _info("writeDcs: len=%d\n", (int) len);
  dump_buffer(buf, len);
  assert(len > 0);

  // Do DCS Short Write or Long Write depending on command length
  switch (len)
  {
    // DCS Short Write (without parameter)
    case 1:
      ret = a64_mipi_dsi_write(VIRTUAL_CHANNEL, 
        MIPI_DSI_DCS_SHORT_WRITE, 
        buf, len);
      break;

    // DCS Short Write (with parameter)
    case 2:
      ret = a64_mipi_dsi_write(VIRTUAL_CHANNEL, 
        MIPI_DSI_DCS_SHORT_WRITE_PARAM, 
        buf, len);
      break;

    // DCS Long Write
    default:
      ret = a64_mipi_dsi_write(VIRTUAL_CHANNEL, 
        MIPI_DSI_DCS_LONG_WRITE, 
        buf, len);
      break;
  };
  _info("ret=%d\n", ret);
  DEBUGASSERT(ret == len);

  return OK;
}

/// Initialise the ST7703 LCD Controller in Xingbangda XBD599 LCD Panel.
/// See https://lupyuen.github.io/articles/dsi#initialise-lcd-controller
int pinephone_panel_init(void) {
  int ret;
  _info("panel_init: start\n");

  // Most of these commands are documented in the ST7703 Datasheet:
  // https://files.pine64.org/doc/datasheet/pinephone/ST7703_DS_v01_20160128.pdf

  // Command #1
  const uint8_t cmd1[] = { 
      0xB9,  // SETEXTC (Page 131): Enable USER Command
      0xF1,  // Enable User command
      0x12,  // (Continued)
      0x83   // (Continued)
  };
  ret = write_dcs(cmd1, sizeof(cmd1));
  assert(ret == OK);

  // Command #2
  const uint8_t cmd2[] = { 
      0xBA,  // SETMIPI (Page 144): Set MIPI related register
      0x33,  // Virtual Channel = 0 (VC_Main = 0) ; Number of Lanes = 4 (Lane_Number = 3)
      0x81,  // LDO = 1.7 V (DSI_LDO_SEL = 4) ; Terminal Resistance = 90 Ohm (RTERM = 1)
      0x05,  // MIPI Low High Speed driving ability = x6 (IHSRX = 5)
      0xF9,  // TXCLK speed in DSI LP mode = fDSICLK / 16 (Tx_clk_sel = 2)
      0x0E,  // Min HFP number in DSI mode = 14 (HFP_OSC = 14)
      0x0E,  // Min HBP number in DSI mode = 14 (HBP_OSC = 14)
      0x20,  // Undocumented
      0x00,  // Undocumented
      0x00,  // Undocumented
      0x00,  // Undocumented
      0x00,  // Undocumented
      0x00,  // Undocumented
      0x00,  // Undocumented
      0x00,  // Undocumented
      0x44,  // Undocumented
      0x25,  // Undocumented
      0x00,  // Undocumented
      0x91,  // Undocumented
      0x0a,  // Undocumented
      0x00,  // Undocumented
      0x00,  // Undocumented
      0x02,  // Undocumented
      0x4F,  // Undocumented
      0x11,  // Undocumented
      0x00,  // Undocumented
      0x00,  // Undocumented
      0x37   // Undocumented
  };
  ret = write_dcs(cmd2, sizeof(cmd2));
  assert(ret == OK);

  // Command #3
  const uint8_t cmd3[] = { 
      0xB8,  // SETPOWER_EXT (Page 142): Set display related register
      0x25,  // External power IC or PFM: VSP = FL1002, VSN = FL1002 (PCCS = 2) ; VCSW1 / VCSW2 Frequency for Pumping VSP / VSN = 1/4 Hsync (ECP_DC_DIV = 5)
      0x22,  // VCSW1/VCSW2 soft start time = 15 ms (DT = 2) ; Pumping ratio of VSP / VSN with VCI = x2 (XDK_ECP = 1)
      0x20,  // PFM operation frequency FoscD = Fosc/1 (PFM_DC_DIV = 0)
      0x03   // Enable power IC pumping frequency synchronization = Synchronize with external Hsync (ECP_SYNC_EN = 1) ; Enable VGH/VGL pumping frequency synchronization = Synchronize with external Hsync (VGX_SYNC_EN = 1)
  };
  ret = write_dcs(cmd3, sizeof(cmd3));
  assert(ret == OK);

  // Command #4
  const uint8_t cmd4[] = { 
      0xB3,  // SETRGBIF (Page 134): Control RGB I/F porch timing for internal use
      0x10,  // Vertical back porch HS number in Blank Frame Period  = Hsync number 16 (VBP_RGB_GEN = 16)
      0x10,  // Vertical front porch HS number in Blank Frame Period = Hsync number 16 (VFP_RGB_GEN = 16)
      0x05,  // HBP OSC number in Blank Frame Period = OSC number 5 (DE_BP_RGB_GEN = 5)
      0x05,  // HFP OSC number in Blank Frame Period = OSC number 5 (DE_FP_RGB_GEN = 5)
      0x03,  // Undocumented
      0xFF,  // Undocumented
      0x00,  // Undocumented
      0x00,  // Undocumented
      0x00,  // Undocumented
      0x00   // Undocumented
  };
  ret = write_dcs(cmd4, sizeof(cmd4));
  assert(ret == OK);

  // Command #5
  const uint8_t cmd5[] = { 
      0xC0,  // SETSCR (Page 147): Set related setting of Source driving
      0x73,  // Source OP Amp driving period for positive polarity in Normal Mode: Source OP Period = 115*4/Fosc (N_POPON = 115)
      0x73,  // Source OP Amp driving period for negative polarity in Normal Mode: Source OP Period = 115*4/Fosc (N_NOPON = 115)
      0x50,  // Source OP Amp driving period for positive polarity in Idle mode: Source OP Period   = 80*4/Fosc (I_POPON = 80)
      0x50,  // Source OP Amp dirivng period for negative polarity in Idle Mode: Source OP Period   = 80*4/Fosc (I_NOPON = 80)
      0x00,  // (SCR Bits 24-31 = 0x00)
      0xC0,  // (SCR Bits 16-23 = 0xC0) 
      0x08,  // Gamma bias current fine tune: Current xIbias   = 4 (SCR Bits 9-13 = 4) ; (SCR Bits  8-15 = 0x08) 
      0x70,  // Source and Gamma bias current core tune: Ibias = 1 (SCR Bits 0-3 = 0) ; Source bias current fine tune: Current xIbias = 7 (SCR Bits 4-8 = 7) ; (SCR Bits  0-7  = 0x70)
      0x00   // Undocumented
  };
  ret = write_dcs(cmd5, sizeof(cmd5));
  assert(ret == OK);

  // Command #6
  const uint8_t cmd6[] = { 
      0xBC,  // SETVDC (Page 146): Control NVDDD/VDDD Voltage
      0x4E   // NVDDD voltage = -1.8 V (NVDDD_SEL = 4) ; VDDD voltage = 1.9 V (VDDD_SEL = 6)
  };
  ret = write_dcs(cmd6, sizeof(cmd6));
  assert(ret == OK);

  // Command #7
  const uint8_t cmd7[] = { 
      0xCC,  // SETPANEL (Page 154): Set display related register
      0x0B   // Enable reverse the source scan direction (SS_PANEL = 1) ; Normal vertical scan direction (GS_PANEL = 0) ; Normally black panel (REV_PANEL = 1) ; S1:S2:S3 = B:G:R (BGR_PANEL = 1)
  };
  ret = write_dcs(cmd7, sizeof(cmd7));
  assert(ret == OK);

  // Command #8
  const uint8_t cmd8[] = { 
      0xB4,  // SETCYC (Page 135): Control display inversion type
      0x80   // Extra source for Zig-Zag Inversion = S2401 (ZINV_S2401_EN = 1) ; Row source data dislocates = Even row (ZINV_G_EVEN_EN = 0) ; Disable Zig-Zag Inversion (ZINV_EN = 0) ; Enable Zig-Zag1 Inversion (ZINV2_EN = 0) ; Normal mode inversion type = Column inversion (N_NW = 0)
  };
  ret = write_dcs(cmd8, sizeof(cmd8));
  assert(ret == OK);

  // Command #9
  const uint8_t cmd9[] = { 
      0xB2,  // SETDISP (Page 132): Control the display resolution
      0xF0,  // Gate number of vertical direction = 480 + (240*4) (NL = 240)
      0x12,  // (RES_V_LSB = 0) ; Non-display area source output control: Source output = VSSD (BLK_CON = 1) ; Channel number of source direction = 720RGB (RESO_SEL = 2)
      0xF0   // Source voltage during Blanking Time when accessing Sleep-Out / Sleep-In command = GND (WHITE_GND_EN = 1) ; Blank timing control when access sleep out command: Blank Frame Period = 7 Frames (WHITE_FRAME_SEL = 7) ; Source output refresh control: Refresh Period = 0 Frames (ISC = 0)
  };
  ret = write_dcs(cmd9, sizeof(cmd9));
  assert(ret == OK);

  // Command #10
  const uint8_t cmd10[] = { 
      0xE3,  // SETEQ (Page 159): Set EQ related register
      0x00,  // Temporal spacing between HSYNC and PEQGND = 0*4/Fosc (PNOEQ = 0)
      0x00,  // Temporal spacing between HSYNC and NEQGND = 0*4/Fosc (NNOEQ = 0)
      0x0B,  // Source EQ GND period when Source up to positive voltage   = 11*4/Fosc (PEQGND = 11)
      0x0B,  // Source EQ GND period when Source down to negative voltage = 11*4/Fosc (NEQGND = 11)
      0x10,  // Source EQ VCI period when Source up to positive voltage   = 16*4/Fosc (PEQVCI = 16)
      0x10,  // Source EQ VCI period when Source down to negative voltage = 16*4/Fosc (NEQVCI = 16)
      0x00,  // Temporal period of PEQVCI1 = 0*4/Fosc (PEQVCI1 = 0)
      0x00,  // Temporal period of NEQVCI1 = 0*4/Fosc (NEQVCI1 = 0)
      0x00,  // (Reserved)
      0x00,  // (Reserved)
      0xFF,  // (Undocumented)
      0x00,  // (Reserved)
      0xC0,  // White pattern to protect GOA glass (ESD_DET_DATA_WHITE = 1) ; Enable ESD detection function to protect GOA glass (ESD_WHITE_EN = 1)
      0x10   // No Need VSYNC (additional frame) after Sleep-In command to display sleep-in blanking frame then into Sleep-In State (SLPIN_OPTION = 1) ; Enable video function detection (VEDIO_NO_CHECK_EN = 0) ; Disable ESD white pattern scanning voltage pull ground (ESD_WHITE_GND_EN = 0) ; ESD detection function period = 0 Frames (ESD_DET_TIME_SEL = 0)
  };
  ret = write_dcs(cmd10, sizeof(cmd10));
  assert(ret == OK);

  // Command #11
  const uint8_t cmd11[] = { 
      0xC6,  // Undocumented
      0x01,  // Undocumented
      0x00,  // Undocumented
      0xFF,  // Undocumented
      0xFF,  // Undocumented
      0x00   // Undocumented
  };
  ret = write_dcs(cmd11, sizeof(cmd11));
  assert(ret == OK);

  // Command #12
  const uint8_t cmd12[] = { 
      0xC1,  // SETPOWER (Page 149): Set related setting of power
      0x74,  // VGH Voltage Adjustment = 17 V (VBTHS = 7) ; VGL Voltage Adjustment = -11 V (VBTLS = 4)
      0x00,  // Enable VGH feedback voltage detection. Output voltage = VBTHS (FBOFF_VGH = 0) ; Enable VGL feedback voltage detection. Output voltage = VBTLS (FBOFF_VGL = 0)
      0x32,  // VSPROUT Voltage = (VRH[5:0] x 0.05 + 3.3) x (VREF/4.8) if VREF [4]=0 (VRP = 50)
      0x32,  // VSNROUT Voltage = (VRH[5:0] x 0.05 + 3.3) x (VREF/5.6) if VREF [4]=1 (VRN = 50)
      0x77,  // Undocumented
      0xF1,  // Enable VGL voltage Detect Function = VGL voltage Abnormal (VGL_DET_EN = 1) ; Enable VGH voltage Detect Function = VGH voltage Abnormal (VGH_DET_EN = 1) ; Enlarge VGL Voltage at "FBOFF_VGL=1" = "VGL=-15V" (VGL_TURBO = 1) ; Enlarge VGH Voltage at "FBOFF_VGH=1" = "VGH=20V" (VGH_TURBO = 1) ; (APS = 1)
      0xFF,  // Left side VGH stage 1 pumping frequency  = 1.5 MHz (VGH1_L_DIV = 15) ; Left side VGL stage 1 pumping frequency  = 1.5 MHz (VGL1_L_DIV = 15)
      0xFF,  // Right side VGH stage 1 pumping frequency = 1.5 MHz (VGH1_R_DIV = 15) ; Right side VGL stage 1 pumping frequency = 1.5 MHz (VGL1_R_DIV = 15)
      0xCC,  // Left side VGH stage 2 pumping frequency  = 2.6 MHz (VGH2_L_DIV = 12) ; Left side VGL stage 2 pumping frequency  = 2.6 MHz (VGL2_L_DIV = 12)
      0xCC,  // Right side VGH stage 2 pumping frequency = 2.6 MHz (VGH2_R_DIV = 12) ; Right side VGL stage 2 pumping frequency = 2.6 MHz (VGL2_R_DIV = 12)
      0x77,  // Left side VGH stage 3 pumping frequency  = 4.5 MHz (VGH3_L_DIV = 7)  ; Left side VGL stage 3 pumping frequency  = 4.5 MHz (VGL3_L_DIV = 7)
      0x77   // Right side VGH stage 3 pumping frequency = 4.5 MHz (VGH3_R_DIV = 7)  ; Right side VGL stage 3 pumping frequency = 4.5 MHz (VGL3_R_DIV = 7)
  };
  ret = write_dcs(cmd12, sizeof(cmd12));
  assert(ret == OK);

  // Command #13
  const uint8_t cmd13[] = { 
      0xB5,  // SETBGP (Page 136): Internal reference voltage setting
      0x07,  // VREF Voltage: 4.2 V (VREF_SEL = 7)
      0x07   // NVREF Voltage: 4.2 V (NVREF_SEL = 7)
  };
  ret = write_dcs(cmd13, sizeof(cmd13));
  assert(ret == OK);

  // Command #14
  const uint8_t cmd14[] = { 
      0xB6,  // SETVCOM (Page 137): Set VCOM Voltage
      0x2C,  // VCOMDC voltage at "GS_PANEL=0" = -0.67 V (VCOMDC_F = 0x2C)
      0x2C   // VCOMDC voltage at "GS_PANEL=1" = -0.67 V (VCOMDC_B = 0x2C)
  };
  ret = write_dcs(cmd14, sizeof(cmd14));
  assert(ret == OK);

  // Command #15
  const uint8_t cmd15[] = { 
      0xBF,  // Undocumented
      0x02,  // Undocumented
      0x11,  // Undocumented
      0x00   // Undocumented
  };
  ret = write_dcs(cmd15, sizeof(cmd15));
  assert(ret == OK);

  // Command #16
  const uint8_t cmd16[] = { 
      0xE9,  // SETGIP1 (Page 163): Set forward GIP timing
      0x82,  // SHR0, SHR1, CHR, CHR2 refer to Internal DE (REF_EN = 1) ; (PANEL_SEL = 2)
      0x10,  // Starting position of GIP STV group 0 = 4102 HSYNC (SHR0 Bits 8-12 = 0x10)
      0x06,  // (SHR0 Bits 0-7  = 0x06)
      0x05,  // Starting position of GIP STV group 1 = 1442 HSYNC (SHR1 Bits 8-12 = 0x05)
      0xA2,  // (SHR1 Bits 0-7  = 0xA2)
      0x0A,  // Distance of STV rising edge and HYSNC  = 10*2  Fosc (SPON  Bits 0-7 = 0x0A)
      0xA5,  // Distance of STV falling edge and HYSNC = 165*2 Fosc (SPOFF Bits 0-7 = 0xA5)
      0x12,  // STV0_1 distance with STV0_0 = 1 HSYNC (SHR0_1 = 1) ; STV0_2 distance with STV0_0 = 2 HSYNC (SHR0_2 = 2)
      0x31,  // STV0_3 distance with STV0_0 = 3 HSYNC (SHR0_3 = 3) ; STV1_1 distance with STV1_0 = 1 HSYNC (SHR1_1 = 1)
      0x23,  // STV1_2 distance with STV1_0 = 2 HSYNC (SHR1_2 = 2) ; STV1_3 distance with STV1_0 = 3 HSYNC (SHR1_3 = 3)
      0x37,  // STV signal high pulse width = 3 HSYNC (SHP = 3) ; Total number of STV signal = 7 (SCP = 7)
      0x83,  // Starting position of GIP CKV group 0 (CKV0_0) = 131 HSYNC (CHR = 0x83)
      0x04,  // Distance of CKV rising edge and HYSNC  = 4*2   Fosc (CON  Bits 0-7 = 0x04)
      0xBC,  // Distance of CKV falling edge and HYSNC = 188*2 Fosc (COFF Bits 0-7 = 0xBC)
      0x27,  // CKV signal high pulse width = 2 HSYNC (CHP = 2) ; Total period cycle of CKV signal = 7 HSYNC (CCP = 7)
      0x38,  // Extra gate counter at blanking area: Gate number = 56 (USER_GIP_GATE = 0x38)
      0x0C,  // Left side GIP output pad signal = ??? (CGTS_L Bits 16-21 = 0x0C)
      0x00,  // (CGTS_L Bits  8-15 = 0x00)
      0x03,  // (CGTS_L Bits  0-7  = 0x03)
      0x00,  // Normal polarity of Left side GIP output pad signal (CGTS_INV_L Bits 16-21 = 0x00)
      0x00,  // (CGTS_INV_L Bits  8-15 = 0x00)
      0x00,  // (CGTS_INV_L Bits  0-7  = 0x00)
      0x0C,  // Right side GIP output pad signal = ??? (CGTS_R Bits 16-21 = 0x0C)
      0x00,  // (CGTS_R Bits  8-15 = 0x00)
      0x03,  // (CGTS_R Bits  0-7  = 0x03)
      0x00,  // Normal polarity of Right side GIP output pad signal (CGTS_INV_R Bits 16-21 = 0x00)
      0x00,  // (CGTS_INV_R Bits  8-15 = 0x00)
      0x00,  // (CGTS_INV_R Bits  0-7  = 0x00)
      0x75,  // Left side GIP output pad signal = ??? (COS1_L = 7) ; Left side GIP output pad signal = ??? (COS2_L = 5)
      0x75,  // Left side GIP output pad signal = ??? (COS3_L = 7) ; (COS4_L = 5)
      0x31,  // Left side GIP output pad signal = ??? (COS5_L = 3) ; (COS6_L = 1)
      0x88,  // Reserved (Parameter 32)
      0x88,  // Reserved (Parameter 33)
      0x88,  // Reserved (Parameter 34)
      0x88,  // Reserved (Parameter 35)
      0x88,  // Reserved (Parameter 36)
      0x88,  // Left side GIP output pad signal  = ??? (COS17_L = 8) ; Left side GIP output pad signal  = ??? (COS18_L = 8)
      0x13,  // Left side GIP output pad signal  = ??? (COS19_L = 1) ; Left side GIP output pad signal  = ??? (COS20_L = 3)
      0x88,  // Left side GIP output pad signal  = ??? (COS21_L = 8) ; Left side GIP output pad signal  = ??? (COS22_L = 8)
      0x64,  // Right side GIP output pad signal = ??? (COS1_R  = 6) ; Right side GIP output pad signal = ??? (COS2_R  = 4)
      0x64,  // Right side GIP output pad signal = ??? (COS3_R  = 6) ; Right side GIP output pad signal = ??? (COS4_R  = 4)
      0x20,  // Right side GIP output pad signal = ??? (COS5_R  = 2) ; Right side GIP output pad signal = ??? (COS6_R  = 0)
      0x88,  // Reserved (Parameter 43)
      0x88,  // Reserved (Parameter 44)
      0x88,  // Reserved (Parameter 45)
      0x88,  // Reserved (Parameter 46)
      0x88,  // Reserved (Parameter 47)
      0x88,  // Right side GIP output pad signal = ??? (COS17_R = 8) ; Right side GIP output pad signal = ??? (COS18_R = 8)
      0x02,  // Right side GIP output pad signal = ??? (COS19_R = 0) ; Right side GIP output pad signal = ??? (COS20_R = 2)
      0x88,  // Right side GIP output pad signal = ??? (COS21_R = 8) ; Right side GIP output pad signal = ??? (COS22_R = 8)
      0x00,  // (TCON_OPT = 0x00)
      0x00,  // (GIP_OPT Bits 16-22 = 0x00)
      0x00,  // (GIP_OPT Bits  8-15 = 0x00)
      0x00,  // (GIP_OPT Bits  0-7  = 0x00)
      0x00,  // Starting position of GIP CKV group 1 (CKV1_0) = 0 HSYNC (CHR2 = 0x00)
      0x00,  // Distance of CKV1 rising edge and HYSNC  = 0*2 Fosc (CON2  Bits 0-7 = 0x00)
      0x00,  // Distance of CKV1 falling edge and HYSNC = 0*2 Fosc (COFF2 Bits 0-7 = 0x00)
      0x00,  // CKV1 signal high pulse width = 0 HSYNC (CHP2 = 0) ; Total period cycle of CKV1 signal = 0 HSYNC (CCP2 = 0)
      0x00,  // (CKS Bits 16-21 = 0x00)
      0x00,  // (CKS Bits  8-15 = 0x00)
      0x00,  // (CKS Bits  0-7  = 0x00)
      0x00,  // (COFF Bits 8-9 = 0) ; (CON Bits 8-9 = 0) ; (SPOFF Bits 8-9 = 0) ; (SPON Bits 8-9 = 0)
      0x00   // (COFF2 Bits 8-9 = 0) ; (CON2 Bits 8-9 = 0)
  };
  ret = write_dcs(cmd16, sizeof(cmd16));
  assert(ret == OK);

  // Command #17
  const uint8_t cmd17[] = { 
      0xEA,  // SETGIP2 (Page 170): Set backward GIP timing
      0x02,  // YS2 Signal Mode = INYS1/INYS2 (YS2_SEL = 0) ; YS2 Signal Mode = INYS1/INYS2 (YS1_SEL = 0) ; Don't reverse YS2 signal (YS2_XOR = 0) ; Don't reverse YS1 signal (YS1_XOR = 0) ; Enable YS signal function (YS_FLAG_EN = 1) ; Disable ALL ON function (ALL_ON_EN = 0)
      0x21,  // (GATE = 0x21)
      0x00,  // (CK_ALL_ON_EN = 0) ; (STV_ALL_ON_EN = 0) ; Timing of YS1 and YS2 signal = ??? (CK_ALL_ON_WIDTH1 = 0)
      0x00,  // Timing of YS1 and YS2 signal = ??? (CK_ALL_ON_WIDTH2 = 0)
      0x00,  // Timing of YS1 and YS2 signal = ??? (CK_ALL_ON_WIDTH3 = 0)
      0x00,  // (YS_FLAG_PERIOD = 0)
      0x00,  // (YS2_SEL_2 = 0) ; (YS1_SEL_2 = 0) ; (YS2_XOR_2 = 0) ; (YS_FLAG_EN_2 = 0) ; (ALL_ON_EN_2 = 0)
      0x00,  // Distance of GIP ALL On rising edge and DE = ??? (USER_GIP_GATE1_2 = 0)
      0x00,  // (CK_ALL_ON_EN_2 = 0) ; (STV_ALL_ON_EN_2 = 0) ; (CK_ALL_ON_WIDTH1_2 = 0)
      0x00,  // (CK_ALL_ON_WIDTH2_2 = 0)
      0x00,  // (CK_ALL_ON_WIDTH3_2 = 0)
      0x00,  // (YS_FLAG_PERIOD_2 = 0)
      0x02,  // (COS1_L_GS = 0) ; (COS2_L_GS = 2)
      0x46,  // (COS3_L_GS = 4) ; (COS4_L_GS = 6)
      0x02,  // (COS5_L_GS = 0) ; (COS6_L_GS = 2)
      0x88,  // Reserved (Parameter 16)
      0x88,  // Reserved (Parameter 17)
      0x88,  // Reserved (Parameter 18)
      0x88,  // Reserved (Parameter 19)
      0x88,  // Reserved (Parameter 20)
      0x88,  // (COS17_L_GS = 8) ; (COS18_L_GS = 8)
      0x64,  // (COS19_L_GS = 6) ; (COS20_L_GS = 4)
      0x88,  // (COS21_L_GS = 8) ; (COS22_L_GS = 8)
      0x13,  // (COS1_R_GS = 1) ; (COS2_R_GS = 3)
      0x57,  // (COS3_R_GS = 5) ; (COS4_R_GS = 7)
      0x13,  // (COS5_R_GS = 1) ; (COS6_R_GS = 3)
      0x88,  // Reserved (Parameter 27)
      0x88,  // Reserved (Parameter 28)
      0x88,  // Reserved (Parameter 29)
      0x88,  // Reserved (Parameter 30)
      0x88,  // Reserved (Parameter 31)
      0x88,  // (COS17_R_GS = 8) ; (COS18_R_GS = 8)
      0x75,  // (COS19_R_GS = 7) ; (COS20_R_GS = 5)
      0x88,  // (COS21_R_GS = 8) ; (COS22_R_GS = 8)
      0x23,  // GIP output EQ signal: P_EQ = Yes, N_EQ = No (EQOPT = 2) ;  GIP output EQ signal level: P_EQ = GND, N_EQ = GND (EQ_SEL = 3)
      0x14,  // Distance of EQ rising edge and HYSNC = 20 Fosc (EQ_DELAY = 0x14)
      0x00,  // Distance of EQ rising edge and HYSNC = 0 HSYNC (EQ_DELAY_HSYNC = 0)
      0x00,  // (HSYNC_TO_CL1_CNT10 Bits 8-9 = 0)
      0x02,  // GIP reference HSYNC between external HSYNC = 2 Fosc (HSYNC_TO_CL1_CNT10 Bits 0-7 = 2)
      0x00,  // Undocumented (Parameter 40)
      0x00,  // Undocumented (Parameter 41)
      0x00,  // Undocumented (Parameter 42)
      0x00,  // Undocumented (Parameter 43)
      0x00,  // Undocumented (Parameter 44)
      0x00,  // Undocumented (Parameter 45)
      0x00,  // Undocumented (Parameter 46)
      0x00,  // Undocumented (Parameter 47)
      0x00,  // Undocumented (Parameter 48)
      0x00,  // Undocumented (Parameter 49)
      0x00,  // Undocumented (Parameter 50)
      0x00,  // Undocumented (Parameter 51)
      0x00,  // Undocumented (Parameter 52)
      0x00,  // Undocumented (Parameter 53)
      0x00,  // Undocumented (Parameter 54)
      0x03,  // Undocumented (Parameter 55)
      0x0A,  // Undocumented (Parameter 56)
      0xA5,  // Undocumented (Parameter 57)
      0x00,  // Undocumented (Parameter 58)
      0x00,  // Undocumented (Parameter 59)
      0x00,  // Undocumented (Parameter 60)
      0x00   // Undocumented (Parameter 61)
  };
  ret = write_dcs(cmd17, sizeof(cmd17));
  assert(ret == OK);

  // Command #18
  const uint8_t cmd18[] = { 
      0xE0,  // SETGAMMA (Page 158): Set the gray scale voltage to adjust the gamma characteristics of the TFT panel
      0x00,  // (PVR0 = 0x00)
      0x09,  // (PVR1 = 0x09)
      0x0D,  // (PVR2 = 0x0D)
      0x23,  // (PVR3 = 0x23)
      0x27,  // (PVR4 = 0x27)
      0x3C,  // (PVR5 = 0x3C)
      0x41,  // (PPR0 = 0x41)
      0x35,  // (PPR1 = 0x35)
      0x07,  // (PPK0 = 0x07)
      0x0D,  // (PPK1 = 0x0D)
      0x0E,  // (PPK2 = 0x0E)
      0x12,  // (PPK3 = 0x12)
      0x13,  // (PPK4 = 0x13)
      0x10,  // (PPK5 = 0x10)
      0x12,  // (PPK6 = 0x12)
      0x12,  // (PPK7 = 0x12)
      0x18,  // (PPK8 = 0x18)
      0x00,  // (NVR0 = 0x00)
      0x09,  // (NVR1 = 0x09)
      0x0D,  // (NVR2 = 0x0D)
      0x23,  // (NVR3 = 0x23)
      0x27,  // (NVR4 = 0x27)
      0x3C,  // (NVR5 = 0x3C)
      0x41,  // (NPR0 = 0x41)
      0x35,  // (NPR1 = 0x35)
      0x07,  // (NPK0 = 0x07)
      0x0D,  // (NPK1 = 0x0D)
      0x0E,  // (NPK2 = 0x0E)
      0x12,  // (NPK3 = 0x12)
      0x13,  // (NPK4 = 0x13)
      0x10,  // (NPK5 = 0x10)
      0x12,  // (NPK6 = 0x12)
      0x12,  // (NPK7 = 0x12)
      0x18   // (NPK8 = 0x18)
  };
  ret = write_dcs(cmd18, sizeof(cmd18));
  assert(ret == OK);

  // Command #19    
  const uint8_t cmd19[] = { 
      0x11  // SLPOUT (Page 89): Turns off sleep mode (MIPI_DCS_EXIT_SLEEP_MODE)
  };
  ret = write_dcs(cmd19, sizeof(cmd19));
  assert(ret == OK);

  // Wait 120 milliseconds
  up_udelay(120 * 1000);

  // Command #20
  const uint8_t cmd20[] = { 
      0x29  // Display On (Page 97): Recover from DISPLAY OFF mode (MIPI_DCS_SET_DISPLAY_ON)
  };    
  ret = write_dcs(cmd20, sizeof(cmd20));
  assert(ret == OK);

  _info("panel_init: end\n");
  return OK;
}
