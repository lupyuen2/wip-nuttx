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
  return -1;
}

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

/// Write to MIPI DSI. See https://lupyuen.github.io/articles/dsi#transmit-packet-over-mipi-dsi
/// On Success: Return number of written bytes. On Error: Return negative error code
ssize_t a64_mipi_dsi_write(
    uint8_t channel,  // Virtual Channel ID
    uint8_t cmd,      // DCS Command
    FAR uint8_t *txbuf,  // Transmit Buffer
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
  _info("packet: len=%d\n", pktlen); // TODO
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
