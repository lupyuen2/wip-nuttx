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
 *
 * "Understanding PinePhone's Display (MIPI DSI)"
 * https://lupyuen.github.io/articles/dsi
 *
 * "NuttX RTOS for PinePhone: Display Driver in Zig"
 * https://lupyuen.github.io/articles/dsi2
 *
 * "A31 Page" refers to Allwinner A31 User Manual
 * https://lupyuen.github.io/images/A31_User_Manual_v1.3_20150510.pdf
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
#include "mipi_dsi.h"
#include "a64_mipi_dsi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A64 CCU Registers and Bit Definitions ************************************/

/* Bus Clock Gating Register 0 (A64 Page 100) */
#define BUS_CLK_GATING_REG0 (A64_CCU_ADDR + 0x60)
#define MIPIDSI_GATING      (1 << 1)

/* Bus Software Reset Register 0 (A64 Page 138) */
#define BUS_SOFT_RST_REG0   (A64_CCU_ADDR + 0x2C0)
#define MIPI_DSI_RST        (1 << 1)

/* A64 MIPI DSI Registers and Bit Definitions *******************************/

/* DSI Control Register (A31 Page 843) */
#define DSI_CTL_REG (A64_DSI_ADDR + 0x0)
#define DSI_EN      (1 << 0)

/* DSI Basic Control Register (Undocumented) */
#define DSI_BASIC_CTL_REG (A64_DSI_ADDR + 0x0c)

/* DSI Configuration Register 0 (A31 Page 845) */
#define DSI_BASIC_CTL0_REG (A64_DSI_ADDR + 0x10)
#define INSTRU_EN          (1 << 0)
#define ECC_En             (1 << 16)
#define CRC_En             (1 << 17)

/* DSI Configuration Register 1 (A31 Page 846) */
#define DSI_BASIC_CTL1_REG (A64_DSI_ADDR + 0x14)
#define DSI_MODE                   (1 << 0)
#define VIDEO_FRAME_START          (1 << 1)
#define VIDEO_PRECISION_MODE_ALIGN (1 << 2)
#define VIDEO_START_DELAY(n)       (n << 4)

/* DSI Line Number Register 0 (A31 Page 847) */
#define DSI_BASIC_SIZE0_REG (A64_DSI_ADDR + 0x18)
#define VIDEO_VSA(n)        (n << 0)
#define VIDEO_VBP(n)        (n << 16)

/* DSI Line Number Register 1 (A31 Page 847) */
#define DSI_BASIC_SIZE1_REG (A64_DSI_ADDR + 0x1c)
#define VIDEO_VACT(n)       (n << 0)
#define VIDEO_VT(n)         (n << 16)

/* DSI Instruction Function Register (Undocumented) */
#define DSI_INST_FUNC_REG(n)   (A64_DSI_ADDR + 0x020 + n * 0x04)
#define DSI_INST_FUNC_LANE_CEN (1 << 4)

/* DSI Instruction Loop Select Register (Undocumented) */
#define DSI_INST_LOOP_SEL_REG  (A64_DSI_ADDR + 0x40)

/* DSI Instruction Loop Number Register (Undocumented) */
#define DSI_INST_LOOP_NUM_REG(n) (A64_DSI_ADDR + 0x44 + n * 0x10)

/* DSI Instruction Jump Select Register (Undocumented) */
#define DSI_INST_JUMP_SEL_REG (A64_DSI_ADDR + 0x48)
#define DSI_INST_ID_LP11      0
#define DSI_INST_ID_TBA       1
#define DSI_INST_ID_HSC       2
#define DSI_INST_ID_HSD       3
#define DSI_INST_ID_LPDT      4
#define DSI_INST_ID_HSCEXIT   5
#define DSI_INST_ID_NOP       6
#define DSI_INST_ID_DLY       7
#define DSI_INST_ID_END       15

/* DSI Instruction Jump Configuration Register (Undocumented) */
#define DSI_INST_JUMP_CFG_REG(n) (A64_DSI_ADDR + 0x4c + n * 0x04)
#define DSI_INST_JUMP_CFG        0

/* DSI Transfer Start Register (Undocumented) */
#define DSI_TRANS_START_REG (A64_DSI_ADDR + 0x60)

/* DSI Transfer Zero Register (Undocumented) */
#define DSI_TRANS_ZERO_REG  (A64_DSI_ADDR + 0x78)

/* DSI Timing Controller DRQ Register (Undocumented) */
#define DSI_TCON_DRQ_REG    (A64_DSI_ADDR + 0x7c)

/* DSI Pixel Format Register 0 (A31 Page 847) */
#define DSI_PIXEL_CTL0_REG (A64_DSI_ADDR + 0x80)
#define PIXEL_FORMAT(n) (n << 0)
#define PIXEL_ENDIAN    (0 << 4)
#define PD_PLUG_DIS     (1 << 16)

/* DSI Pixel Package Register 0 (A31 Page 848) */
#define DSI_PIXEL_PH_REG (A64_DSI_ADDR + 0x90)
#define PIXEL_DT(n)      (n << 0)
#define PIXEL_VC(n)      (n << 6)
#define PIXEL_WC(n)      (n << 8)
#define PIXEL_ECC(n)     (n << 24)

/* DSI Pixel Package Register 2 (A31 Page 849) */
#define DSI_PIXEL_PF0_REG (A64_DSI_ADDR + 0x98)
#define CRC_FORCE         0xffff

/* DSI Pixel Package Register 3 (A31 Page 849) */
#define DSI_PIXEL_PF1_REG (A64_DSI_ADDR + 0x9c)
#define CRC_INIT_LINE0(n) (n << 0)
#define CRC_INIT_LINEN(n) (n << 16)

/* DSI Sync Package Register 0 (A31 Page 850) */
#define DSI_SYNC_HSS_REG (A64_DSI_ADDR + 0xb0)
#define SYNC_ECC(n)      (n << 24)
#define SYNC_D1(n)       (n << 16)
#define SYNC_D0(n)       (n << 8)
#define SYNC_VC(n)       (n << 6)
#define SYNC_DT(n)       (n << 0)

/* DSI Sync Package Register 1 (A31 Page 850) */
#define DSI_SYNC_HSE_REG (A64_DSI_ADDR + 0xb4)

/* DSI Sync Package Register 2 (A31 Page 851) */
#define DSI_SYNC_VSS_REG (A64_DSI_ADDR + 0xb8)

/* DSI Sync Package Register 3 (A31 Page 851) */
#define DSI_SYNC_VSE_REG (A64_DSI_ADDR + 0xbc)

/* DSI Blank Package Register 0 (A31 Page 852) */
#define DSI_BLK_HSA0_REG (A64_DSI_ADDR + 0xc0)

/* DSI Blank Package Register 1 (A31 Page 852) */
#define DSI_BLK_HSA1_REG (A64_DSI_ADDR + 0xc4)
#define HSA_PD(n)        (n << 0)
#define HSA_PF(n)        (n << 16)

/* DSI Blank Package Register 2 (A31 Page 852) */
#define DSI_BLK_HBP0_REG (A64_DSI_ADDR + 0xc8)

/* DSI Blank Package Register 3 (A31 Page 852) */
#define DSI_BLK_HBP1_REG (A64_DSI_ADDR + 0xcc)
#define HBP_PD(n)        (n << 0)
#define HBP_PF(n)        (n << 16)

/* DSI Blank Package Register 4 (A31 Page 852) */
#define DSI_BLK_HFP0_REG (A64_DSI_ADDR + 0xd0)

/* DSI Blank Package Register 5 (A31 Page 853) */
#define DSI_BLK_HFP1_REG (A64_DSI_ADDR + 0xd4)
#define HFP_PD(n)        (n << 0)
#define HFP_PF(n)        (n << 16)

/* DSI Blank Package Register 6 (A31 Page 853) */
#define DSI_BLK_HBLK0_REG (A64_DSI_ADDR + 0xe0)

/* DSI Blank Package Register 7 (A31 Page 853) */
#define DSI_BLK_HBLK1_REG (A64_DSI_ADDR + 0xe4)
#define HBLK_PD(n)        (n << 0)
#define HBLK_PF(n)        (n << 16)

/* DSI Blank Package Register 8 (A31 Page 854) */
#define DSI_BLK_VBLK0_REG (A64_DSI_ADDR + 0xe8)

/* DSI Blank Package Register 9 (A31 Page 854) */
#define DSI_BLK_VBLK1_REG (A64_DSI_ADDR + 0xec)
#define VBLK_PD(n)        (n << 0)
#define VBLK_PF(n)        (n << 16)

/* DSI Low Power Control Register (A31 Page 854) */
#define DSI_CMD_CTL_REG (A64_DSI_ADDR + 0x200)
#define TX_FLAG         (1 << 9)
#define RX_FLAG         (1 << 25)
#define RX_OVERFLOW     (1 << 26)

/* DSI Debug Data Register (Undocumented) */
#define DSI_DEBUG_DATA_REG (A64_DSI_ADDR + 0x2f8)

/* DSI Low Power Transmit Package Register (A31 Page 856) */
#define DSI_CMD_TX_REG     (A64_DSI_ADDR + 0x300)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a64_disable_dsi_processing
 *
 * Description:
 *   Disable DSI Processing
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void a64_disable_dsi_processing(void)
{
  /* DSI Configuration Register 0 (A31 Page 845)
   * Set INSTRU_EN (Bit 0) to 0
   */

  modreg32(0, INSTRU_EN, DSI_BASIC_CTL0_REG);
}

/****************************************************************************
 * Name: a64_enable_dsi_processing
 *
 * Description:
 *   Enable DSI Processing
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void a64_enable_dsi_processing(void)
{
  /* DSI Configuration Register 0 (A31 Page 845)
   * Set INSTRU_EN (Bit 0) to 1
   */

  modreg32(INSTRU_EN, INSTRU_EN, DSI_BASIC_CTL0_REG);
}

/****************************************************************************
 * Name: a64_wait_dsi_transmit
 *
 * Description:
 *   Wait for DSI Transmission to complete.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK if completed, ERROR if timeout.
 *
 ****************************************************************************/

static int a64_wait_dsi_transmit(void)
{
  int i;

  /* Wait up to 5 milliseconds */

  for (i = 0; i < 5; i++)
    {
      /* To check whether the transmission is complete,
       * we poll on INSTRU_EN (Bit 0)
       */

      if ((getreg32(DSI_BASIC_CTL0_REG) & INSTRU_EN) == 0)
        {
          /* If INSTRU_EN is 0, then transmission is complete */

          return 0;
        }

      /* Sleep 1 millisecond and try again */

      up_mdelay(1);
    }

  gerr("timeout");
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aaaa
 *
 * Description:
 *   aaaa
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

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
  ginfo("channel=%d, cmd=0x%x, txlen=%ld\n", channel, cmd, txlen); // TODO
  DEBUGASSERT(txbuf != NULL);
  if (cmd == MIPI_DSI_DCS_SHORT_WRITE)
    {
      DEBUGASSERT(txlen == 1);
      if (txlen != 1) { return ERROR; }
    }
  if (cmd == MIPI_DSI_DCS_SHORT_WRITE_PARAM)
    {
      DEBUGASSERT(txlen == 2);
      if (txlen != 2) { return ERROR; }
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
  ginfo("pktlen=%ld\n", pktlen);
  ginfodumpbuffer("pkt", pkt, pktlen);

  // DSI Low Power Control Register (A31 Page 854)
  // Set the following bits to 1
  // RX_Overflow (Bit 26): Clear flag for "Receive Overflow"
  // RX_Flag (Bit 25): Clear flag for "Receive has started"
  // TX_Flag (Bit 9): Clear flag for "Transmit has started"
  // All other bits must be set to 0.
  putreg32(
      RX_OVERFLOW | RX_FLAG | TX_FLAG,
      DSI_CMD_CTL_REG
  );

  // Write the Long Packet to
  // DSI Low Power Transmit Package Register (A31 Page 856)
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
      modreg32(v, 0xFFFFFFFF, addr);
      addr += 4;
    }

  // Set Packet Length - 1 in Bits 0 to 7 (TX_Size) of
  // DSI Low Power Control Register (A31 Page 854)
  modreg32(pktlen - 1, 0xFF, DSI_CMD_CTL_REG);

  // DSI Instruction Jump Select Register (Undocumented)
  // Set to begin the Low Power Transmission (LPTX)
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

/****************************************************************************
 * Name: aaaa
 *
 * Description:
 *   aaaa
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

/// Enable MIPI DSI Block.
/// Based on https://lupyuen.github.io/articles/dsi#appendix-enable-mipi-dsi-block
int a64_mipi_dsi_enable(void)
{
  /* Enable MIPI DSI Bus ****************************************************/

  ginfo("Enable MIPI DSI Bus\n");

  // Bus Clock Gating Register 0 (A64 Page 100)
  // Set MIPIDSI_GATING (Bit 1) to 1 (Pass Gating Clock for MIPI DSI)
  DEBUGASSERT(BUS_CLK_GATING_REG0 == 0x1c20060);

  DEBUGASSERT(MIPIDSI_GATING == 2);
  modreg32(MIPIDSI_GATING, MIPIDSI_GATING, BUS_CLK_GATING_REG0);

  // Bus Software Reset Register 0 (A64 Page 138)
  // Set MIPI_DSI_RST (Bit 1) to 1 (Deassert MIPI DSI Reset)
  DEBUGASSERT(BUS_SOFT_RST_REG0 == 0x1c202c0);

  DEBUGASSERT(MIPI_DSI_RST == 2);
  modreg32(MIPI_DSI_RST, MIPI_DSI_RST, BUS_SOFT_RST_REG0);

  // Enable DSI Block
  ginfo("Enable DSI Block\n");

  // DSI Control Register (A31 Page 843)
  // Set DSI_En (Bit 0) to 1 (Enable DSI)
  DEBUGASSERT(DSI_CTL_REG == 0x1ca0000);

  DEBUGASSERT(DSI_EN == 1);
  putreg32(DSI_EN, DSI_CTL_REG);

  // DSI Configuration Register 0 (A31 Page 845)
  // Set CRC_En (Bit 17) to 1 (Enable CRC)
  // Set ECC_En (Bit 16) to 1 (Enable ECC)
  DEBUGASSERT(DSI_BASIC_CTL0_REG == 0x1ca0010);

  const uint32_t dsi_basic_ctl0 = CRC_En |
                                  ECC_En;
  DEBUGASSERT(dsi_basic_ctl0 == 0x30000);
  putreg32(dsi_basic_ctl0, DSI_BASIC_CTL0_REG);

  // DSI Transfer Start Register (Undocumented)
  // Set to 10
  DEBUGASSERT(DSI_TRANS_START_REG == 0x1ca0060);
  putreg32(10, DSI_TRANS_START_REG);

  // DSI Transfer Zero Register (Undocumented)
  // Set to 0
  DEBUGASSERT(DSI_TRANS_ZERO_REG == 0x1ca0078);
  putreg32(0, DSI_TRANS_ZERO_REG);

  // Set Instructions (Undocumented)
  ginfo("Set Instructions\n");

  // DSI Instruction Function Register (Undocumented)
  // Set DSI_INST_ID_LP11 to 0x1f
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_LP11) == 0x1ca0020);
  putreg32(0x1f, DSI_INST_FUNC_REG(DSI_INST_ID_LP11));

  // Set DSI_INST_ID_TBA to 0x1000 0001
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_TBA) == 0x1ca0024);
  putreg32(0x10000001, DSI_INST_FUNC_REG(DSI_INST_ID_TBA));

  // Set DSI_INST_ID_HSC to 0x2000 0010
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_HSC) == 0x1ca0028);
  putreg32(0x20000010, DSI_INST_FUNC_REG(DSI_INST_ID_HSC));

  // Set DSI_INST_ID_HSD to 0x2000 000f
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_HSD) == 0x1ca002c);
  putreg32(0x2000000f, DSI_INST_FUNC_REG(DSI_INST_ID_HSD));

  // Set DSI_INST_ID_LPDT to 0x3010 0001
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_LPDT) == 0x1ca0030);
  putreg32(0x30100001, DSI_INST_FUNC_REG(DSI_INST_ID_LPDT));

  // Set DSI_INST_ID_HSCEXIT to 0x4000 0010
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_HSCEXIT) == 0x1ca0034);
  putreg32(0x40000010, DSI_INST_FUNC_REG(DSI_INST_ID_HSCEXIT));

  // Set DSI_INST_ID_NOP to 0xf
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_NOP) == 0x1ca0038);
  putreg32(0xf, DSI_INST_FUNC_REG(DSI_INST_ID_NOP));

  // Set DSI_INST_ID_DLY to 0x5000 001f
  DEBUGASSERT(DSI_INST_FUNC_REG(DSI_INST_ID_DLY) == 0x1ca003c);
  putreg32(0x5000001f, DSI_INST_FUNC_REG(DSI_INST_ID_DLY));

  // Configure Jump Instructions (Undocumented)
  ginfo("Configure Jump Instructions\n");

  // DSI Instruction Jump Configuration Register (Undocumented)
  // Set DSI_INST_JUMP_CFG to 0x56 0001
  DEBUGASSERT(DSI_INST_JUMP_CFG_REG(DSI_INST_JUMP_CFG) == 0x1ca004c);
  putreg32(0x560001, DSI_INST_JUMP_CFG_REG(DSI_INST_JUMP_CFG));

  // DSI Debug Data Register (Undocumented)
  // Set to 0xff
  DEBUGASSERT(DSI_DEBUG_DATA_REG == 0x1ca02f8);
  putreg32(0xff, DSI_DEBUG_DATA_REG);

  // Set Video Start Delay
  ginfo("Set Video Start Delay\n");

  // DSI Configuration Register 1 (A31 Page 846)
  // Set Video_Start_Delay (Bits 4 to 16) to 1468 (Line Delay)
  // Set Video_Precision_Mode_Align (Bit 2) to 1 (Fill Mode)
  // Set Video_Frame_Start (Bit 1) to 1 (Precision Mode)
  // Set DSI_Mode (Bit 0) to 1 (Video Mode)
  // TODO: Video_Start_Delay is actually 13 bits, not 8 bits as documented in the A31 User Manual
  DEBUGASSERT(DSI_BASIC_CTL1_REG == 0x1ca0014);

  const uint32_t dsi_basic_ctl1 = VIDEO_START_DELAY(1468) |
                                  VIDEO_PRECISION_MODE_ALIGN |
                                  VIDEO_FRAME_START |
                                  DSI_MODE;
  DEBUGASSERT(dsi_basic_ctl1 == 0x5bc7);
  putreg32(dsi_basic_ctl1, DSI_BASIC_CTL1_REG);

  // Set Burst (Undocumented)
  ginfo("Set Burst\n");

  // DSI Timing Controller DRQ Register (Undocumented)
  // Set to 0x1000 0007
  DEBUGASSERT(DSI_TCON_DRQ_REG == 0x1ca007c);
  putreg32(0x10000007, DSI_TCON_DRQ_REG);

  // Set Instruction Loop (Undocumented)
  ginfo("Set Instruction Loop\n");

  // DSI Instruction Loop Select Register (Undocumented)
  // Set to 0x3000 0002
  DEBUGASSERT(DSI_INST_LOOP_SEL_REG == 0x1ca0040);
  putreg32(0x30000002, DSI_INST_LOOP_SEL_REG);

  // DSI Instruction Loop Number Register (Undocumented)
  // Set Register 0 to 0x31 0031
  DEBUGASSERT(DSI_INST_LOOP_NUM_REG(0) == 0x1ca0044);
  putreg32(0x310031, DSI_INST_LOOP_NUM_REG(0));

  // Set Register 1 to 0x31 0031
  DEBUGASSERT(DSI_INST_LOOP_NUM_REG(1) == 0x1ca0054);
  putreg32(0x310031, DSI_INST_LOOP_NUM_REG(1));

  // Set Pixel Format
  ginfo("Set Pixel Format\n");

  // DSI Pixel Package Register 0 (A31 Page 848)
  // Set ECC (Bits 24 to 31) to 19
  // Set WC (Bits 8 to 23) to 2160 (Byte Numbers of PD in a Pixel Packet)
  // Set VC (Bits 6 to 7) to 0 (Virtual Channel)
  // Set DT (Bits 0 to 5) to 0x3E (24-bit Video Mode)
  DEBUGASSERT(DSI_PIXEL_PH_REG == 0x1ca0090);
  const uint32_t dsi_pixel_ph = PIXEL_ECC(19) |
                                PIXEL_WC(2160) |
                                PIXEL_VC(A64_MIPI_DSI_VIRTUAL_CHANNEL) |
                                PIXEL_DT(0x3E);
  DEBUGASSERT(dsi_pixel_ph == 0x1308703e);
  putreg32(dsi_pixel_ph, DSI_PIXEL_PH_REG);

  // DSI Pixel Package Register 2 (A31 Page 849)
  // Set CRC_Force (Bits 0 to 15) to 0xffff (Force CRC to this value)
  DEBUGASSERT(DSI_PIXEL_PF0_REG == 0x1ca0098);
  putreg32(CRC_FORCE, DSI_PIXEL_PF0_REG);

  // DSI Pixel Package Register 3 (A31 Page 849)
  // Set CRC_Init_LineN (Bits 16 to 31) to 0xffff (CRC initial to this value in transmitions except 1st one)
  // Set CRC_Init_Line0 (Bits 0 to 15) to 0xffff (CRC initial to this value in 1st transmition every frame)
  DEBUGASSERT(DSI_PIXEL_PF1_REG == 0x1ca009c);

  const uint32_t DSI_PIXEL_PF1 = CRC_INIT_LINEN(0XFFFF)
      | CRC_INIT_LINE0(0XFFFF);
  DEBUGASSERT(DSI_PIXEL_PF1 == 0xffffffff);
  putreg32(DSI_PIXEL_PF1, DSI_PIXEL_PF1_REG);

  // DSI Pixel Format Register 0 (A31 Page 847)
  // Set PD_Plug_Dis (Bit 16) to 1 (Disable PD plug before pixel bytes)
  // Set Pixel_Endian (Bit 4) to 0 (LSB first)
  // Set Pixel_Format (Bits 0 to 3) to 8 (24-bit RGB888)
  DEBUGASSERT(DSI_PIXEL_CTL0_REG == 0x1ca0080);

  const uint32_t dsi_pixel_ctl0 = PD_PLUG_DIS |
                                  PIXEL_ENDIAN |
                                  PIXEL_FORMAT(8);
  DEBUGASSERT(dsi_pixel_ctl0 == 0x10008);
  putreg32(dsi_pixel_ctl0, DSI_PIXEL_CTL0_REG);

  // Set Sync Timings
  ginfo("Set Sync Timings\n");

  // DSI Basic Control Register (Undocumented)
  // Set to 0
  DEBUGASSERT(DSI_BASIC_CTL_REG == 0x1ca000c);
  putreg32(0x0, DSI_BASIC_CTL_REG);

  // DSI Sync Package Register 0 (A31 Page 850)
  // Set ECC (Bits 24 to 31) to 0x12
  // Set D1 (Bits 16 to 23) to 0
  // Set D0 (Bits 8 to 15) to 0
  // Set VC (Bits 6 to 7) to 0 (Virtual Channel)
  // Set DT (Bits 0 to 5) to 0x21 (HSS)
  DEBUGASSERT(DSI_SYNC_HSS_REG == 0x1ca00b0);
  const uint32_t dsi_sync_hss = SYNC_ECC(0x12) |
                                SYNC_D1(0) |
                                SYNC_D0(0) |
                                SYNC_VC(A64_MIPI_DSI_VIRTUAL_CHANNEL) |
                                SYNC_DT(0x21);
  DEBUGASSERT(dsi_sync_hss == 0x12000021);
  putreg32(dsi_sync_hss, DSI_SYNC_HSS_REG);

  // DSI Sync Package Register 1 (A31 Page 850)
  // Set ECC (Bits 24 to 31) to 1
  // Set D1 (Bits 16 to 23) to 0
  // Set D0 (Bits 8 to 15) to 0
  // Set VC (Bits 6 to 7) to 0 (Virtual Channel)
  // Set DT (Bits 0 to 5) to 0x31 (HSE)
  DEBUGASSERT(DSI_SYNC_HSE_REG == 0x1ca00b4);
  const uint32_t dsi_sync_hse = SYNC_ECC(1) |
                                SYNC_D1(0) |
                                SYNC_D0(0) |
                                SYNC_VC(A64_MIPI_DSI_VIRTUAL_CHANNEL) |
                                SYNC_DT(0x31);
  DEBUGASSERT(dsi_sync_hse == 0x1000031);
  putreg32(dsi_sync_hse, DSI_SYNC_HSE_REG);

  // DSI Sync Package Register 2 (A31 Page 851)
  // Set ECC (Bits 24 to 31) to 7
  // Set D1 (Bits 16 to 23) to 0
  // Set D0 (Bits 8 to 15) to 0
  // Set VC (Bits 6 to 7) to 0 (Virtual Channel)
  // Set DT (Bits 0 to 5) to 1 (VSS)
  DEBUGASSERT(DSI_SYNC_VSS_REG == 0x1ca00b8);
  const uint32_t dsi_sync_vss = SYNC_ECC(7) |
                                SYNC_D1(0) |
                                SYNC_D0(0) |
                                SYNC_VC(A64_MIPI_DSI_VIRTUAL_CHANNEL) |
                                SYNC_DT(1);
  DEBUGASSERT(dsi_sync_vss == 0x7000001);
  putreg32(dsi_sync_vss, DSI_SYNC_VSS_REG);

  // DSI Sync Package Register 3 (A31 Page 851)
  // Set ECC (Bits 24 to 31) to 0x14
  // Set D1 (Bits 16 to 23) to 0
  // Set D0 (Bits 8 to 15) to 0
  // Set VC (Bits 6 to 7) to 0 (Virtual Channel)
  // Set DT (Bits 0 to 5) to 0x11 (VSE)
  DEBUGASSERT(DSI_SYNC_VSE_REG == 0x1ca00bc);
  const uint32_t dsi_sync_vse = SYNC_ECC(0x14) |
                                SYNC_D1(0) |
                                SYNC_D0(0) |
                                SYNC_VC(A64_MIPI_DSI_VIRTUAL_CHANNEL) |
                                SYNC_DT(0x11);
  DEBUGASSERT(dsi_sync_vse == 0x14000011);
  putreg32(dsi_sync_vse, DSI_SYNC_VSE_REG);

  // Set Basic Size
  ginfo("Set Basic Size\n");

  // DSI Line Number Register 0 (A31 Page 847)
  // Set Video_VBP (Bits 16 to 27) to 17
  // Set Video_VSA (Bits 0 to 11) to 10
  DEBUGASSERT(DSI_BASIC_SIZE0_REG == 0x1ca0018);

  const uint32_t dsi_basic_size0 = VIDEO_VBP(17) |
                                   VIDEO_VSA(10);
  DEBUGASSERT(dsi_basic_size0 == 0x11000a);
  putreg32(dsi_basic_size0, DSI_BASIC_SIZE0_REG);

  // DSI Line Number Register 1 (A31 Page 847)
  // Set Video_VT (Bits 16 to 28) to 1485
  // Set Video_VACT (Bits 0 to 11) to 1440
  DEBUGASSERT(DSI_BASIC_SIZE1_REG == 0x1ca001c);

  const uint32_t dsi_basic_size1 = VIDEO_VT(1485) |
                                   VIDEO_VACT(1440);
  DEBUGASSERT(dsi_basic_size1 == 0x5cd05a0);
  putreg32(dsi_basic_size1, DSI_BASIC_SIZE1_REG);

  // Set Horizontal Blanking
  ginfo("Set Horizontal Blanking\n");

  // DSI Blank Package Register 0 (A31 Page 852)
  // Set HSA_PH (Bits 0 to 31) to 0x900 4a19
  DEBUGASSERT(DSI_BLK_HSA0_REG == 0x1ca00c0);
  const uint32_t DSI_BLK_HSA0 = 0x9004a19;
  putreg32(DSI_BLK_HSA0, DSI_BLK_HSA0_REG);

  // DSI Blank Package Register 1 (A31 Page 852)
  // Set HSA_PF (Bits 16 to 31) to 0x50b4
  // Set HSA_PD (Bits 0 to 7) to 0
  DEBUGASSERT(DSI_BLK_HSA1_REG == 0x1ca00c4);

  const uint32_t dsi_blk_hsa1 = HSA_PF(0x50b4) |
                                HSA_PD(0);
  DEBUGASSERT(dsi_blk_hsa1 == 0x50b40000);
  putreg32(dsi_blk_hsa1, DSI_BLK_HSA1_REG);

  // DSI Blank Package Register 2 (A31 Page 852)
  // Set HBP_PH (Bits 0 to 31) to 0x3500 5419
  DEBUGASSERT(DSI_BLK_HBP0_REG == 0x1ca00c8);
  putreg32(0x35005419, DSI_BLK_HBP0_REG);

  // DSI Blank Package Register 3 (A31 Page 852)
  // Set HBP_PF (Bits 16 to 31) to 0x757a
  // Set HBP_PD (Bits 0 to 7) to 0
  DEBUGASSERT(DSI_BLK_HBP1_REG == 0x1ca00cc);

  const uint32_t dsi_blk_hbp1 = HBP_PF(0x757a) |
                                HBP_PD(0);
  DEBUGASSERT(dsi_blk_hbp1 == 0x757a0000);
  putreg32(dsi_blk_hbp1, DSI_BLK_HBP1_REG);

  // DSI Blank Package Register 4 (A31 Page 852)
  // Set HFP_PH (Bits 0 to 31) to 0x900 4a19
  DEBUGASSERT(DSI_BLK_HFP0_REG == 0x1ca00d0);
  putreg32(0x9004a19,  DSI_BLK_HFP0_REG);

  // DSI Blank Package Register 5 (A31 Page 853)
  // Set HFP_PF (Bits 16 to 31) to 0x50b4
  // Set HFP_PD (Bits 0 to 7) to 0
  DEBUGASSERT(DSI_BLK_HFP1_REG == 0x1ca00d4);

  const uint32_t dsi_blk_hfp1 = HFP_PF(0x50b4) |
                                HFP_PD(0);
  DEBUGASSERT(dsi_blk_hfp1 == 0x50b40000);
  putreg32(dsi_blk_hfp1, DSI_BLK_HFP1_REG);

  // DSI Blank Package Register 6 (A31 Page 853)
  // Set HBLK_PH (Bits 0 to 31) to 0xc09 1a19
  DEBUGASSERT(DSI_BLK_HBLK0_REG == 0x1ca00e0);
  putreg32(0xc091a19,  DSI_BLK_HBLK0_REG);

  // DSI Blank Package Register 7 (A31 Page 853)
  // Set HBLK_PF (Bits 16 to 31) to 0x72bd
  // Set HBLK_PD (Bits 0 to 7) to 0
  DEBUGASSERT(DSI_BLK_HBLK1_REG == 0x1ca00e4);

  const uint32_t dsi_blk_hblk1 = HBLK_PF(0x72bd) |
                                 HBLK_PD(0);
  DEBUGASSERT(dsi_blk_hblk1 == 0x72bd0000);
  putreg32(dsi_blk_hblk1, DSI_BLK_HBLK1_REG);

  // Set Vertical Blanking
  ginfo("Set Vertical Blanking\n");

  // DSI Blank Package Register 8 (A31 Page 854)
  // Set VBLK_PH (Bits 0 to 31) to 0x1a00 0019
  DEBUGASSERT(DSI_BLK_VBLK0_REG == 0x1ca00e8);
  putreg32(0x1a000019, DSI_BLK_VBLK0_REG);

  // DSI Blank Package Register 9 (A31 Page 854)
  // Set VBLK_PF (Bits 16 to 31) to 0xffff
  // Set VBLK_PD (Bits 0 to 7) to 0
  DEBUGASSERT(DSI_BLK_VBLK1_REG == 0x1ca00ec);

  const uint32_t dsi_blk_vblk1 = VBLK_PF(0xffff) |
                                 VBLK_PD(0);
  DEBUGASSERT(dsi_blk_vblk1 == 0xffff0000);
  putreg32(dsi_blk_vblk1, DSI_BLK_VBLK1_REG);

  return OK;
}

/****************************************************************************
 * Name: aaaa
 *
 * Description:
 *   aaaa
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is always returned at present.
 *
 ****************************************************************************/

/// Start MIPI DSI HSC and HSD. (High Speed Clock Mode and High Speed Data Transmission)
/// Based on https://lupyuen.github.io/articles/dsi#appendix-start-mipi-dsi-hsc-and-hsd
int a64_mipi_dsi_start(void)
{
  // Start HSC (Undocumented)
  ginfo("Start HSC\n");

  // DSI Instruction Jump Select Register (Undocumented)
  // Set to 0xf02
  DEBUGASSERT(DSI_INST_JUMP_SEL_REG == 0x1ca0048);
  putreg32(0xf02, DSI_INST_JUMP_SEL_REG);

  // Commit
  ginfo("Commit\n");

  // DSI Configuration Register 0 (A31 Page 845)
  // Set INSTRU_EN (Bit 0) to 1 (Enable DSI Processing from Instruction 0)
  DEBUGASSERT(DSI_BASIC_CTL0_REG == 0x1ca0010);
  DEBUGASSERT(INSTRU_EN == 0x1);
  modreg32(INSTRU_EN, INSTRU_EN, DSI_BASIC_CTL0_REG);

  // (DSI_INST_FUNC_REG(n) is (0x020 + (n) * 0x04))

  // Instruction Function Lane (Undocumented)
  ginfo("Instruction Function Lane\n");

  // DSI Instruction Function Register (Undocumented)
  // Set DSI_INST_FUNC_LANE_CEN (Bit 4) to 0
  // Index 0 is DSI_INST_ID_LP11
  DEBUGASSERT(DSI_INST_FUNC_REG(0) == 0x1ca0020);

  DEBUGASSERT(DSI_INST_FUNC_LANE_CEN == 0x10);
  modreg32(0x0, DSI_INST_FUNC_LANE_CEN, DSI_INST_FUNC_REG(0) );

  // Wait 1 millisecond
  up_mdelay(1);

  // Start HSD (Undocumented)
  ginfo("Start HSD\n");

  // DSI Instruction Jump Select Register (Undocumented)
  // Set to 0x63f0 7006
  DEBUGASSERT(DSI_INST_JUMP_SEL_REG == 0x1ca0048);
  putreg32(0x63f07006, DSI_INST_JUMP_SEL_REG);

  // Commit
  ginfo("Commit\n");

  // DSI Configuration Register 0 (A31 Page 845)
  // Set INSTRU_EN (Bit 0) to 1 (Enable DSI Processing from Instruction 0)
  DEBUGASSERT(DSI_BASIC_CTL0_REG == 0x1ca0010);
  DEBUGASSERT(INSTRU_EN == 0x1);
  modreg32(INSTRU_EN, INSTRU_EN, DSI_BASIC_CTL0_REG);

  return OK;
}

//// TODO: Remove this test code
#include "../../pinephone-nuttx/test/test_a64_mipi_dsi.c" //// TODO
