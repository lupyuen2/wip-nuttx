/****************************************************************************
 * boards/risc-v/jh7110/star64/src/jh7110_appinit.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/drivers/ramdisk.h>
#include <sys/mount.h>
#include <sys/boardctl.h>
#include <arch/board/board_memorymap.h>
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Ramdisk Definition */

#define SECTORSIZE   512
#define NSECTORS(b)  (((b) + SECTORSIZE - 1) / SECTORSIZE)
#define RAMDISK_DEVICE_MINOR 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mount_ramdisk
 *
 * Description:
 *  Mount a ramdisk defined in the ld.script to /dev/ramX.  The ramdisk is
 *  intended to contain a romfs with applications which can be spawned at
 *  runtime.
 *
 * Returned Value:
 *   OK is returned on success.
 *   -ERRORNO is returned on failure.
 *
 ****************************************************************************/

int mount_ramdisk(void)
{
  int ret;
  struct boardioc_romdisk_s desc;

  desc.minor    = RAMDISK_DEVICE_MINOR;
  desc.nsectors = NSECTORS((ssize_t)__ramdisk_size);
  desc.sectsize = SECTORSIZE;
  desc.image    = __ramdisk_start;

  ret = boardctl(BOARDIOC_ROMDISK, (uintptr_t)&desc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Ramdisk register failed: %s\n", strerror(errno));
      syslog(LOG_ERR, "Ramdisk mountpoint /dev/ram%d\n",
             RAMDISK_DEVICE_MINOR);
      syslog(LOG_ERR, "Ramdisk length %lu, origin %lx\n",
             (ssize_t)__ramdisk_size, (uintptr_t)__ramdisk_start);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#ifdef CONFIG_BOARD_LATE_INITIALIZE
  /* Board initialization already performed by board_late_initialize() */

  return OK;
#else
  /* Perform board-specific initialization */

#ifdef CONFIG_NSH_ARCHINIT

  mount(NULL, "/proc", "procfs", 0, NULL);

#endif

  return OK;
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called after up_initialize() and board_early_initialize() and just
 *   before the initial application is started.  This additional
 *   initialization phase may be used, for example, to initialize board-
 *   specific device drivers for which board_early_initialize() is not
 *   suitable.
 *
 *   Waiting for events, use of I2C, SPI, etc are permissible in the context
 *   of board_late_initialize().  That is because board_late_initialize()
 *   will run on a temporary, internal kernel thread.
 *
 ****************************************************************************/

void board_late_initialize(void)
{
  /* Mount the RAM Disk */

  mount_ramdisk();

  /* Perform board-specific initialization */

#ifdef CONFIG_NSH_ARCHINIT

  mount(NULL, "/proc", "procfs", 0, NULL);

#endif

  // Verify that Video Output / Display Subsystem is down
  uint32_t val = getreg32(0x295C0000);
  DEBUGASSERT(val == 0);

  // Power Up the Power Management Unit for Video Output / Display Subsystem
  // TODO: Switch to constants
  putreg32(0x10, 0x1703000c);
  putreg32(0xff, 0x17030044);
  putreg32(0x05, 0x17030044);
  putreg32(0x50, 0x17030044);

  // Wait 50 milliseconds for Power Up
  up_mdelay(50);

  // Enable the Clocks for Video Output / Display Subsystem
  modifyreg32(0x13020028, 0, 1 << 31);  // Addr, Clear Bits, Set Bits
  modifyreg32(0x1302004c, 0, 1 << 31);
  modifyreg32(0x13020098, 0, 1 << 31);
  modifyreg32(0x1302009c, 0, 1 << 31);
  modifyreg32(0x130200e8, 0, 1 << 31);
  modifyreg32(0x130200f0, 0, 1 << 31);
  modifyreg32(0x130200f4, 0, 1 << 31);
  modifyreg32(0x130200f8, 0, 1 << 31);
  modifyreg32(0x130200fc, 0, 1 << 31);

  // Deassert the Resets for Video Output / Display Subsystem
  // Software RESET 1 Address Selector: Offset 0x2fc
  // Clear Bit 11: rstn_u0_dom_vout_top_rstn_dom_vout_top_rstn_vout_src
  modifyreg32(0x130202fc, 1 << 11, 0);  // Addr, Clear Bits, Set Bits

  // SYSCRG RESET Status 0: Offset 0x308
  // Clear Bit 26: rstn_u0_sft7110_noc_bus_reset_disp_axi_n
  modifyreg32(0x13020308, 1 << 26, 0);  // Addr, Clear Bits, Set Bits

  // Verify that Video Output / Display Subsystem is up
  val = getreg32(0x295C0000);
  DEBUGASSERT(val == 4);

  // Enable the Clocks for DC8200 Display Controller (HDMI)
  modifyreg32(0x295C0010, 0, 1 << 31);  // Addr, Clear Bits, Set Bits
  modifyreg32(0x295C0014, 0, 1 << 31);
  modifyreg32(0x295C0018, 0, 1 << 31);
  modifyreg32(0x295C001c, 0, 1 << 31);
  modifyreg32(0x295C0020, 0, 1 << 31);
  modifyreg32(0x295C003c, 0, 1 << 31);
  modifyreg32(0x295C0040, 0, 1 << 31);
  modifyreg32(0x295C0044, 0, 1 << 31);

  // Deassert the Resets for DC8200 Display Controller (HDMI)
  modifyreg32(
    0x295C0048,  // Addr
    (1 << 0) | (1 << 1) | (1 << 2) | (1 << 9),  // Clear Bits
    0  // Set Bits
  );

  // Verify that Hardware Revision and Chip ID are non-zero
  uint32_t revision = getreg32(0x29400024);
  uint32_t chip_id = getreg32(0x29400030);
  _info("revision=0x%x, chip_id=0x%x", revision, chip_id);
  DEBUGASSERT(revision != 0 && chip_id != 0);

  // Test HDMI
  // int test_hdmi(void);
  // int ret = test_hdmi();
  // DEBUGASSERT(ret == 0);
}

// Display Subsystem Base Address
// https://doc-en.rvspace.org/JH7110/TRM/JH7110_TRM/memory_map_display.html
#define DISPLAY_BASE_ADDRESS     (0x29400000)

// DC8200 AHB0 Base Address
// https://doc-en.rvspace.org/JH7110/TRM/JH7110_TRM/memory_map_display.html
#define DC8200_AHB0_BASE_ADDRESS (DISPLAY_BASE_ADDRESS + 0x000000)

// U0_HDMITX Base Address
// https://doc-en.rvspace.org/JH7110/TRM/JH7110_TRM/system_memory_map.html
#define U0_HDMITX_BASE_ADDRESS   (DISPLAY_BASE_ADDRESS + 0x190000)

// DOM VOUT Control Registers
// https://doc-en.rvspace.org/JH7110/TRM/JH7110_TRM/memory_map_display.html
#define VOUT_CRG_BASE_ADDRESS    (DISPLAY_BASE_ADDRESS + 0x1C0000)

// DOM VOUT Control Registers
// https://doc-en.rvspace.org/JH7110/TRM/JH7110_TRM/dom_vout_crg.html

#define clk_u0_dc8200_clk_axi   (VOUT_CRG_BASE_ADDRESS + 0x10)
#define clk_u0_dc8200_clk_core  (VOUT_CRG_BASE_ADDRESS + 0x14)
#define clk_u0_dc8200_clk_ahb   (VOUT_CRG_BASE_ADDRESS + 0x18)
#define clk_u0_dc8200_clk_pix0  (VOUT_CRG_BASE_ADDRESS + 0x1c)
#define clk_u0_dc8200_clk_pix1  (VOUT_CRG_BASE_ADDRESS + 0x20)
#define clk_u0_hdmi_tx_clk_mclk (VOUT_CRG_BASE_ADDRESS + 0x3c)
#define clk_u0_hdmi_tx_clk_bclk (VOUT_CRG_BASE_ADDRESS + 0x40)
#define clk_u0_hdmi_tx_clk_sys  (VOUT_CRG_BASE_ADDRESS + 0x44)
#define CLK_ICG (1 << 31)

// Note: Reset is actually at 295C0048, not 295C0038!
#define Software_RESET_assert0_addr_assert_sel (VOUT_CRG_BASE_ADDRESS + 0x48)
#define rstn_u0_dc8200_rstn_axi   (1 << 0)
#define rstn_u0_dc8200_rstn_ahb   (1 << 1)
#define rstn_u0_dc8200_rstn_core  (1 << 2)
#define rstn_u0_hdmi_tx_rstn_hdmi (1 << 9)

// #include "../../../../../hdmi/hdmi.c"
