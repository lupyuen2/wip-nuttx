/****************************************************************************
 * arch/arm64/src/a64/mipi_dsi.c
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
#include "mipi_dsi.h"

/************************************************************************************************
 * Private Data
 ************************************************************************************************/

/* From CRC-16-CCITT (x^16 + x^12 + x^5 + 1) */

static uint16_t crc16ccitt_tab[256] =
{
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};

/************************************************************************************************
 * Private Functions
 ************************************************************************************************/

/************************************************************************************************
 * Name: crc16ccitt
 *
 * Description:
 *   Return a 16-bit CRC-CCITT of the contents of the buffer.
 *
 ************************************************************************************************/

static uint16_t crc16ccitt(FAR const uint8_t *src, size_t len, uint16_t crc16val)
{
  size_t i;
  uint16_t v = crc16val;

  for (i = 0; i < len; i++)
    {
      v = (v >> 8)
          ^ crc16ccitt_tab[(v ^ src[i]) & 0xff];
    }

  return v;
}

/// Compute 16-bit Cyclic Redundancy Check (CRC).
/// See "12.3.6.13: Packet Footer", Page 210 of BL808 Reference Manual:
/// https://files.pine64.org/doc/datasheet/ox64/BL808_RM_en_1.0(open).pdf
static uint16_t compute_crc(FAR const uint8_t *data, size_t len)
{
  // Use CRC-16-CCITT (x^16 + x^12 + x^5 + 1)
  uint16_t crc = crc16ccitt(data, len, 0xffff);

  // debug("computeCrc: len={}, crc=0x{x}", .{ data.len, crc });
  // dump_buffer(&data[0], data.len);
  return crc;
}

/// Compute the Error Correction Code (ECC) (1 byte):
/// Allow single-bit errors to be corrected and 2-bit errors to be detected in the Packet Header
/// See "12.3.6.12: Error Correction Code", Page 208 of BL808 Reference Manual:
/// https://files.pine64.org/doc/datasheet/ox64/BL808_RM_en_1.0(open).pdf
static uint8_t compute_ecc(
  FAR const uint8_t *di_wc,  // Data Identifier + Word Count (3 bytes)
  size_t len  // Must be 3
) {
  DEBUGASSERT(len == 3);
  if (len != 3)
    {
      return 0;
    }

  // Combine DI and WC into a 24-bit word
  uint32_t di_wc_word = 
      di_wc[0] 
      | (di_wc[1] << 8)
      | (di_wc[2] << 16);

  // Extract the 24 bits from the word
  bool d[24];
  memset(d, 0, sizeof(d));
  int i;
  for (i = 0; i < 24; i++)
    {
      d[i] = di_wc_word & 1;
      di_wc_word >>= 1;
    }

  // Compute the ECC bits
  bool ecc[8];
  memset(ecc, 0, sizeof(ecc));
  ecc[7] = 0;
  ecc[6] = 0;
  ecc[5] = d[10] ^ d[11] ^ d[12] ^ d[13] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[21] ^ d[22] ^ d[23];
  ecc[4] = d[4]  ^ d[5]  ^ d[6]  ^ d[7]  ^ d[8]  ^ d[9]  ^ d[16] ^ d[17] ^ d[18] ^ d[19] ^ d[20] ^ d[22] ^ d[23];
  ecc[3] = d[1]  ^ d[2]  ^ d[3]  ^ d[7]  ^ d[8]  ^ d[9]  ^ d[13] ^ d[14] ^ d[15] ^ d[19] ^ d[20] ^ d[21] ^ d[23];
  ecc[2] = d[0]  ^ d[2]  ^ d[3]  ^ d[5]  ^ d[6]  ^ d[9]  ^ d[11] ^ d[12] ^ d[15] ^ d[18] ^ d[20] ^ d[21] ^ d[22];
  ecc[1] = d[0]  ^ d[1]  ^ d[3]  ^ d[4]  ^ d[6]  ^ d[8]  ^ d[10] ^ d[12] ^ d[14] ^ d[17] ^ d[20] ^ d[21] ^ d[22] ^ d[23];
  ecc[0] = d[0]  ^ d[1]  ^ d[2]  ^ d[4]  ^ d[5]  ^ d[7]  ^ d[10] ^ d[11] ^ d[13] ^ d[16] ^ d[20] ^ d[21] ^ d[22] ^ d[23];

  // Merge the ECC bits
  return ecc[0]
      | (ecc[1] << 1)
      | (ecc[2] << 2)
      | (ecc[3] << 3)
      | (ecc[4] << 4)
      | (ecc[5] << 5)
      | (ecc[6] << 6)
      | (ecc[7] << 7);
}

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

// Compose MIPI DSI Long Packet. See https://lupyuen.github.io/articles/dsi#long-packet-for-mipi-dsi
// Return the number of bytes in the composed packet.
// Returns -1 if pktlen too short
ssize_t mipi_dsi_long_packet(
  FAR uint8_t *pktbuf,    // Buffer for the Returned Long Packet
  size_t pktlen,    // Buffer Size for the Returned Long Packet
  uint8_t channel,  // Virtual Channel ID
  uint8_t cmd,      // DCS Command
  FAR const uint8_t *txbuf,  // Transmit Buffer
  size_t txlen          // Buffer Length
)
{
  _info("channel=%d, cmd=0x%x, txlen=%d\n", channel, cmd, (int) txlen); ////
  // Data Identifier (DI) (1 byte):
  // - Virtual Channel Identifier (Bits 6 to 7)
  // - Data Type (Bits 0 to 5)
  // (Virtual Channel should be 0, I think)
  DEBUGASSERT(channel < 4);
  DEBUGASSERT(cmd < (1 << 6));
  const uint8_t vc = channel;
  const uint8_t dt = cmd;
  const uint8_t di = (vc << 6) | dt;

  // Word Count (WC) (2 bytes)：
  // - Number of bytes in the Packet Payload
  const uint16_t wc = txlen;
  const uint8_t wcl = wc & 0xff;
  const uint8_t wch = wc >> 8;

  // Data Identifier + Word Count (3 bytes): For computing Error Correction Code (ECC)
  const uint8_t di_wc[3] = { di, wcl, wch };

  // Compute Error Correction Code (ECC) for Data Identifier + Word Count
  const uint8_t ecc = compute_ecc(di_wc, sizeof(di_wc));

  // Packet Header (4 bytes):
  // - Data Identifier + Word Count + Error Correction Code
  const uint8_t header[4] = { di_wc[0], di_wc[1], di_wc[2], ecc };

  // Packet Payload:
  // - Data (0 to 65,541 bytes):
  // Number of data bytes should match the Word Count (WC)
  DEBUGASSERT(txlen <= 65541);

  // Checksum (CS) (2 bytes):
  // - 16-bit Cyclic Redundancy Check (CRC) of the Payload (not the entire packet)
  const uint16_t cs = compute_crc(txbuf, txlen);
  const uint8_t csl = cs & 0xff;
  const uint8_t csh = cs >> 8;

  // Packet Footer (2 bytes)
  // - Checksum (CS)
  const uint8_t footer[2] = { csl, csh };

  // Packet:
  // - Packet Header (4 bytes)
  // - Payload (`len` bytes)
  // - Packet Footer (2 bytes)
  const size_t len = sizeof(header) + txlen + sizeof(footer);
  DEBUGASSERT(len <= pktlen);  // Increase `pkt` size
  if (len > pktlen)
    {
      return -1;
    }
  memcpy(pktbuf, header, sizeof(header)); // 4 bytes
  memcpy(pktbuf + sizeof(header), txbuf, txlen);  // txlen bytes
  memcpy(pktbuf + sizeof(header) + txlen, footer, sizeof(footer));  // 2 bytes

  // Return the packet length
  return len;
}

// Compose MIPI DSI Short Packet. See https://lupyuen.github.io/articles/dsi#appendix-short-packet-for-mipi-dsi
// Return the number of bytes in the composed packet.
// Returns -1 if pktlen too short
ssize_t mipi_dsi_short_packet(
  FAR uint8_t *pktbuf,    // Buffer for the Returned Short Packet
  size_t pktlen,    // Buffer Size for the Returned Short Packet
  uint8_t channel,  // Virtual Channel ID
  uint8_t cmd,      // DCS Command
  FAR const uint8_t *txbuf,  // Transmit Buffer
  size_t txlen          // Buffer Length
)
{
  _info("channel=%d, cmd=0x%x, txlen=%d\n", channel, cmd, (int) txlen); ////
  DEBUGASSERT(txlen == 1 || txlen == 2);

  // From BL808 Reference Manual (Page 201): https://files.pine64.org/doc/datasheet/ox64/BL808_RM_en_1.0(open).pdf
  //   A Short Packet consists of 8-bit data identification (DI),
  //   two bytes of commands or data, and 8-bit ECC.
  //   The length of a short packet is 4 bytes including ECC.
  // Thus a MIPI DSI Short Packet (compared with Long Packet)...
  // - Doesn't have Packet Payload and Packet Footer (CRC)
  // - Instead of Word Count (WC), the Packet Header now has 2 bytes of data
  // Everything else is the same.

  // Data Identifier (DI) (1 byte):
  // - Virtual Channel Identifier (Bits 6 to 7)
  // - Data Type (Bits 0 to 5)
  // (Virtual Channel should be 0, I think)
  DEBUGASSERT(channel < 4);
  DEBUGASSERT(cmd < (1 << 6));
  const uint8_t vc = channel;
  const uint8_t dt = cmd;
  const uint8_t di = (vc << 6) | dt;

  // Data (2 bytes), fill with 0 if Second Byte is missing
  const uint8_t data[2] =
    {
      txbuf[0],                     // First Byte
      (txlen == 2) ? txbuf[1] : 0,  // Second Byte
    };

  // Data Identifier + Data (3 bytes): For computing Error Correction Code (ECC)
  const uint8_t di_data[3] = { di, data[0], data[1] };

  // Compute Error Correction Code (ECC) for Data Identifier + Word Count
  const uint8_t ecc = compute_ecc(di_data, sizeof(di_data));

  // Packet Header (4 bytes):
  // - Data Identifier + Data + Error Correction Code
  const uint8_t header[4] = { di_data[0], di_data[1], di_data[2], ecc };

  // Packet:
  // - Packet Header (4 bytes)
  const size_t len = sizeof(header);
  DEBUGASSERT(len <= pktlen);  // Increase `pkt` size
  if (len > pktlen)
    {
      return -1;
    }
  memcpy(pktbuf, header, sizeof(header)); // 4 bytes

  // Return the packet length
  return len;
}

//// TODO: Remove this test code
#include "../../pinephone-nuttx/test/test_mipi_dsi.c" //// TODO
