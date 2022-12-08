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
static int waitForTransmit(void)
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

void *TODO_REMOVE_THIS[] = {waitForTransmit,a64_enable_dsi_processing, a64_disable_dsi_processing};  //// TODO

#ifdef TODO
/// Write the DCS Command to MIPI DSI
fn a64_write_dcs(buf: []const u8) void {
    debug("writeDcs: len={}", .{ buf.len });
    dump_buffer(&buf[0], buf.len);
    assert(buf.len > 0);

    // Do DCS Short Write or Long Write depending on command length
    const res = switch (buf.len) {

        // DCS Short Write (without parameter)
        1 => nuttx_mipi_dsi_dcs_write(null, VIRTUAL_CHANNEL, 
            MIPI_DSI_DCS_SHORT_WRITE, 
            &buf[0], buf.len),

        // DCS Short Write (with parameter)
        2 => nuttx_mipi_dsi_dcs_write(null, VIRTUAL_CHANNEL, 
            MIPI_DSI_DCS_SHORT_WRITE_PARAM, 
            &buf[0], buf.len),

        // DCS Long Write
        else => nuttx_mipi_dsi_dcs_write(null, VIRTUAL_CHANNEL, 
            MIPI_DSI_DCS_LONG_WRITE, 
            &buf[0], buf.len),
    };
    assert(res == buf.len);
}
#endif  // TODO

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#ifdef TODO
/// Write to MIPI DSI. See https://lupyuen.github.io/articles/dsi#transmit-packet-over-mipi-dsi
pub export fn a64_mipi_dsi_write(
    channel: u8,  // Virtual Channel ID
    cmd: u8,      // DCS Command
    buf: [*c]const u8,  // Transmit Buffer
    len: usize          // Buffer Length
) isize {  // On Success: Return number of written bytes. On Error: Return negative error code
    _ = dev;
    debug("mipi_dsi_dcs_write: channel={}, cmd=0x{x}, len={}", .{ channel, cmd, len });
    if (cmd == MIPI_DSI_DCS_SHORT_WRITE)       { assert(len == 1); }
    if (cmd == MIPI_DSI_DCS_SHORT_WRITE_PARAM) { assert(len == 2); }

    // Allocate Packet Buffer
    var pkt_buf = std.mem.zeroes([128]u8);

    // Compose Short or Long Packet depending on DCS Command
    const pkt = switch (cmd) {

        // For DCS Long Write: Compose Long Packet
        MIPI_DSI_DCS_LONG_WRITE =>
            composeLongPacket(&pkt_buf, channel, cmd, buf, len),

        // For DCS Short Write (with and without parameter):
        // Compose Short Packet
        MIPI_DSI_DCS_SHORT_WRITE,
        MIPI_DSI_DCS_SHORT_WRITE_PARAM =>
            composeShortPacket(&pkt_buf, channel, cmd, buf, len),

        // DCS Command not supported
        else => unreachable,
    };

    // Dump the packet
    debug("packet: len={}", .{ pkt.len });
    dump_buffer(&pkt[0], pkt.len);

    // Set the following bits to 1 in DSI_CMD_CTL_REG (DSI Low Power Control Register) at Offset 0x200:
    // RX_Overflow (Bit 26): Clear flag for "Receive Overflow"
    // RX_Flag (Bit 25): Clear flag for "Receive has started"
    // TX_Flag (Bit 9): Clear flag for "Transmit has started"
    // All other bits must be set to 0.
    const DSI_CMD_CTL_REG = DSI_BASE_ADDRESS + 0x200;
    const RX_Overflow = 1 << 26;
    const RX_Flag     = 1 << 25;
    const TX_Flag     = 1 << 9;
    putreg32(
        RX_Overflow | RX_Flag | TX_Flag,
        DSI_CMD_CTL_REG
    );

    // Write the Long Packet to DSI_CMD_TX_REG 
    // (DSI Low Power Transmit Package Register) at Offset 0x300 to 0x3FC
    const DSI_CMD_TX_REG = DSI_BASE_ADDRESS + 0x300;
    var addr: u64 = DSI_CMD_TX_REG;
    var i: usize = 0;
    while (i < pkt.len) : (i += 4) {
        // Fetch the next 4 bytes, fill with 0 if not available
        const b = [4]u32 {
            pkt[i],
            if (i + 1 < pkt.len) pkt[i + 1] else 0,
            if (i + 2 < pkt.len) pkt[i + 2] else 0,
            if (i + 3 < pkt.len) pkt[i + 3] else 0,
        };

        // Merge the next 4 bytes into a 32-bit value
        const v: u32 =
            b[0]
            + (b[1] << 8)
            + (b[2] << 16)
            + (b[3] << 24);

        // Write the 32-bit value
        assert(addr <= DSI_BASE_ADDRESS + 0x3FC);
        modreg32(v, 0xFFFF_FFFF, addr);  // TODO: DMB
        addr += 4;
    }

    // Set Packet Length - 1 in Bits 0 to 7 (TX_Size) of
    // DSI_CMD_CTL_REG (DSI Low Power Control Register) at Offset 0x200
    modreg32(@intCast(u32, pkt.len) - 1, 0xFF, DSI_CMD_CTL_REG);  // TODO: DMB

    // Set DSI_INST_JUMP_SEL_REG (Offset 0x48, undocumented) 
    // to begin the Low Power Transmission (LPTX)
    const DSI_INST_JUMP_SEL_REG = DSI_BASE_ADDRESS + 0x48;
    const DSI_INST_ID_LPDT = 4;
    const DSI_INST_ID_LP11 = 0;
    const DSI_INST_ID_END  = 15;
    putreg32(
        DSI_INST_ID_LPDT << (4 * DSI_INST_ID_LP11) |
        DSI_INST_ID_END  << (4 * DSI_INST_ID_LPDT),
        DSI_INST_JUMP_SEL_REG
    );

    // Disable DSI Processing then Enable DSI Processing
    disableDsiProcessing();
    enableDsiProcessing();

    // Wait for transmission to complete
    const res = waitForTransmit();
    if (res < 0) {
        disableDsiProcessing();
        return res;
    }

    // Return number of written bytes
    return @intCast(isize, len);
}
#endif  // TODO
