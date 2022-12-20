/****************************************************************************
 * arch/arm64/src/a64/a64_rsb.c
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
 * "A80 Page" refers to Allwinner A80 User Manual
 * https://lupyuen.github.io/images/A80_User_Manual_v1.3.1_20150513.pdf
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
#include "a64_rsb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/// TODO: Reduced Serial Bus Base Address (R_RSB) (A64 Page 75)
#define R_RSB_BASE_ADDRESS 0x01f03400

/* A64 Reduced Serial Bus Registers and Bit Definitions *********************/

// RSB Control Register (RSB_CTRL) (A80 Page 923)
#define RSB_CTRL   (R_RSB_BASE_ADDRESS + 0x00)

// RSB Status Register (RSB_STAT) (A80 Page 924)
#define RSB_STAT   (R_RSB_BASE_ADDRESS + 0x0c)

// RSB Address Register (RSB_AR) (A80 Page 926)
#define RSB_AR     (R_RSB_BASE_ADDRESS + 0x10)

// RSB Data Buffer Register (RSB_DATA) (A80 Page 926)
#define RSB_DATA   (R_RSB_BASE_ADDRESS + 0x1c)

// RSB Command Register (RSB_CMD) (A80 Page 928)
#define RSB_CMD    (R_RSB_BASE_ADDRESS + 0x2c)

// RSB Device Address Register (RSB_DAR) (A80 Page 928)
#define RSB_DAR    (R_RSB_BASE_ADDRESS + 0x30)

/* A64 Reduced Serial Bus Commands ******************************************/

/// Read a byte from Reduced Serial Bus (A80 Page 918)
#define RSBCMD_RD8 0x8B

/// Write a byte to Reduced Serial Bus (A80 Page 918)
#define RSBCMD_WR8 0x4E

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/// Wait for Reduced Serial Bus Transaction to complete.
/// Returns -1 on error.
static int rsb_wait_trans(void) 
{
    // Wait for transaction to complete
    int tries = 100000;
    while (true) {
      // RSB Control Register (RSB_CTRL) (A80 Page 923)
      // At RSB Offset 0x0000
      // Wait for START_TRANS (Bit 7) to be 0 (Transaction Completed or Error)
      uint32_t reg = getreg32(RSB_CTRL); 
      if ((reg & (1 << 7)) == 0)
        { 
          break; 
        }

      // Check for transaction timeout
      tries -= 1;
      if (tries == 0)
        {
          _err("Transaction Timeout\n");  // TODO
          return -1;
        }
    }
    return 0;
}

/// Wait for Reduced Serial Bus and read the status.
/// Returns -1 on error.
static int rsb_wait_status(void)
{
  // Wait for transaction to complete or fail with error
  int ret = rsb_wait_trans();
  if (ret != 0)
    {
      return ret;
    }

  // RSB Status Register (RSB_STAT) (A80 Page 924)
  // At RSB Offset 0x000c
  // If TRANS_OVER (Bit 0) is 1, then RSB Transfer has completed without error
  uint32_t reg = getreg32(RSB_STAT);
  if (reg == 0x01)
    {
      return 0;
    }

  _err("Transaction Failed\n");  // TODO
  return -1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/// Read a byte from Reduced Serial Bus.
/// Returns -1 on error.
int a64_rsb_read(
    uint8_t rt_addr,  // Run-Time Address
    uint8_t reg_addr  // Register Address
)
{
  // RSB Command Register (RSB_CMD) (A80 Page 928)
  // At RSB Offset 0x002C
  // Set to 0x8B (RD8) to read one byte
  _info("rsb_read: rt_addr=0x%x, reg_addr=0x%x\n", rt_addr, reg_addr);  // TODO
  putreg32(RSBCMD_RD8, RSB_CMD);   

  // RSB Device Address Register (RSB_DAR) (A80 Page 928)
  // At RSB Offset 0x0030
  // Set RTA (Bits 16 to 23) to the Run-Time Address (0x2D for AXP803 PMIC)
  uint32_t rta = rt_addr << 16;
  putreg32(rta, RSB_DAR);   

  // RSB Address Register (RSB_AR) (A80 Page 926)
  // At RSB Offset 0x0010
  // Set to the Register Address that we’ll read from AXP803 PMIC
  putreg32(reg_addr, RSB_AR);    

  // RSB Control Register (RSB_CTRL) (A80 Page 923)
  // At RSB Offset 0x0000
  // Set START_TRANS (Bit 7) to 1 (Start Transaction)
  putreg32(0x80, RSB_CTRL);  

  // Wait for RSB Status
  int ret = rsb_wait_status();
  if (ret != 0) { return ret; }

  // RSB Data Buffer Register (RSB_DATA) (A80 Page 926)
  // At RSB Offset 0x001c
  // Contains the Register Value read from AXP803 PMIC
  return getreg8(RSB_DATA);
}

/// Write a byte to Reduced Serial Bus.
/// Returns -1 on error.
int a64_rsb_write(
    uint8_t rt_addr,  // Run-Time Address
    uint8_t reg_addr,  // Register Address
    uint8_t value  // Value to be written
)
{
  // RSB Command Register (RSB_CMD) (A80 Page 928)
  // At RSB Offset 0x002C
  // Set to 0x4E (WR8) to write one byte
  _info("rt_addr=0x%x, reg_addr=0x%x, value=0x%x\n", rt_addr, reg_addr, value);  // TODO
  putreg32(RSBCMD_WR8, RSB_CMD);   

  // RSB Device Address Register (RSB_DAR) (A80 Page 928)
  // At RSB Offset 0x0030
  // Set RTA (Bits 16 to 23) to the Run-Time Address (0x2D for AXP803 PMIC)
  uint32_t rta = rt_addr << 16;
  putreg32(rta, RSB_DAR);   

  // RSB Address Register (RSB_AR) (A80 Page 926)
  // At RSB Offset 0x0010
  // Set to the Register Address that we’ll write to AXP803 PMIC
  putreg32(reg_addr, RSB_AR);    

  // RSB Data Buffer Register (RSB_DATA) (A80 Page 926)
  // At RSB Offset 0x001c
  // Set to the Register Value that will be written to AXP803 PMIC
  putreg32(value, RSB_DATA);  

  // RSB Control Register (RSB_CTRL) (A80 Page 923)
  // At RSB Offset 0x0000
  // Set START_TRANS (Bit 7) to 1 (Start Transaction)
  putreg32(0x80, RSB_CTRL);  

  // Wait for RSB Status
  return rsb_wait_status();
}

#include "../../pinephone-nuttx/test/test_a64_rsb.c"  //// TODO: Remove Test Code
