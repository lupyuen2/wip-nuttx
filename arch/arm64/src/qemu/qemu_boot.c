/****************************************************************************
 * arch/arm64/src/qemu/qemu_boot.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/cache.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#ifdef CONFIG_LEGACY_PAGING
#  include <nuttx/page.h>
#endif

#include <arch/chip/chip.h>

#ifdef CONFIG_SMP
#include "arm64_smp.h"
#endif

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_mmu.h"
#include "qemu_boot.h"
#include "qemu_serial.h"

#ifdef CONFIG_DEVICE_TREE
#  include <nuttx/fdt.h>
#endif

//// TODO
extern uint8_t __ramdisk_start[];
extern uint8_t __ramdisk_size[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct arm_mmu_region g_mmu_regions[] =
{
  MMU_REGION_FLAT_ENTRY("DEVICE_REGION",
                        CONFIG_DEVICEIO_BASEADDR, CONFIG_DEVICEIO_SIZE,
                        MT_DEVICE_NGNRNE | MT_RW | MT_SECURE),

  MMU_REGION_FLAT_ENTRY("DRAM0_S0",
                        CONFIG_RAMBANK1_ADDR, CONFIG_RAMBANK1_SIZE,
                        MT_NORMAL | MT_RW | MT_SECURE),

  // MMU_REGION_FLAT_ENTRY("PCI_CFG",
  //                       CONFIG_PCI_CFG_BASEADDR, CONFIG_PCI_CFG_SIZE,
  //                       MT_NORMAL | MT_RW | MT_SECURE),

  // MMU_REGION_FLAT_ENTRY("PCI_MEM",
  //                       CONFIG_PCI_MEM_BASEADDR, CONFIG_PCI_MEM_SIZE,
  //                       MT_NORMAL | MT_RW | MT_SECURE),

  // MMU_REGION_FLAT_ENTRY("PCI_IO",
  //                       CONFIG_PCI_IO_BASEADDR, CONFIG_PCI_IO_SIZE,
  //                       MT_NORMAL | MT_RW | MT_SECURE),
};

const struct arm_mmu_config g_mmu_config =
{
  .num_regions = nitems(g_mmu_regions),
  .mmu_regions = g_mmu_regions,
};

#ifdef CONFIG_SYSLOG_RPMSG
static char g_syslog_rpmsg_buf[4096];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_copy_overlap
 *
 * Description:
 *   Copy an overlapping memory region.  dest overlaps with src + count.
 *
 * Input Parameters:
 *   dest  - Destination address
 *   src   - Source address
 *   count - Number of bytes to copy
 *
 ****************************************************************************/

static void qemu_copy_overlap(uint8_t *dest, const uint8_t *src,
                              size_t count)
{
  uint8_t *d = dest + count - 1;
  const uint8_t *s = src + count - 1;

  if (dest <= src)
    {
      _err("dest and src should overlap");
      PANIC();
    }

  while (count--)
    {
      volatile uint8_t c = *s;  /* Prevent compiler optimization */
      *d = c;
      d--;
      s--;
    }
} 

/****************************************************************************
 * Name: qemu_copy_ramdisk
 *
 * Description:
 *   Copy the RAM Disk from NuttX Image to RAM Disk Region.
 *
 ****************************************************************************/

static void qemu_copy_ramdisk(void)
{
  char header[8] __attribute__((aligned(8))) = "-rom1fs-";
  const uint8_t *limit = (uint8_t *)g_idle_topstack + (256 * 1024);
  uint8_t *ramdisk_addr = NULL;
  uint8_t *addr;
  uint32_t size;

  /* After _edata, search for "-rom1fs-". This is the RAM Disk Address.
   * Limit search to 256 KB after Idle Stack Top.
   */

  binfo("_edata=%p, _sbss=%p, _ebss=%p, idlestack_top=%p\n",
        (void *)_edata, (void *)_sbss, (void *)_ebss,
        (void *)g_idle_topstack);
  for (addr = g_idle_topstack; addr < limit; addr += 8)
    {
      if (addr == _edata) { _info("addr=%p, header=%p, sizeof(header)=%d\n", addr, header, sizeof(header)); } ////
      if (memcmp(addr, header, sizeof(header)) == 0)
        {
          ramdisk_addr = addr;
          break;
        }
    }

  /* Stop if RAM Disk is missing */

  binfo("ramdisk_addr=%p\n", ramdisk_addr);
  if (ramdisk_addr == NULL)
    {
      _err("Missing RAM Disk. Check the initrd padding.");
      PANIC();
    }

  /* RAM Disk must be after Idle Stack, to prevent overwriting */

  // if (ramdisk_addr <= (uint8_t *)g_idle_topstack)
  //   {
  //     const size_t pad = (size_t)g_idle_topstack - (size_t)ramdisk_addr;
  //     _err("RAM Disk must be after Idle Stack. Increase initrd padding "
  //           "by %d bytes.", pad);
  //     PANIC();
  //   }

  /* Read the Filesystem Size from the next 4 bytes (Big Endian) */

  size = (ramdisk_addr[8] << 24) + (ramdisk_addr[9] << 16) +
         (ramdisk_addr[10] << 8) + ramdisk_addr[11] + 0x1f0;
  binfo("size=%d\n", size);

  /* Filesystem Size must be less than RAM Disk Memory Region */

  if (size > (size_t)__ramdisk_size)
    {
      _err("RAM Disk Region too small. Increase by %ul bytes.\n",
            size - (size_t)__ramdisk_size);
      PANIC();
    }

  /* Copy the RAM Disk from NuttX Image to RAM Disk Region.
   * __ramdisk_start overlaps with ramdisk_addr + size.
   */

  qemu_copy_overlap(__ramdisk_start, ramdisk_addr, size);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SMP

/****************************************************************************
 * Name: arm64_get_mpid
 *
 * Description:
 *   The function from cpu index to get cpu mpid which is reading
 * from mpidr_el1 register. Different ARM64 Core will use different
 * Affn define, the mpidr_el1 value is not CPU number, So we need
 * to change CPU number to mpid and vice versa
 *
 ****************************************************************************/

uint64_t arm64_get_mpid(int cpu)
{
  return CORE_TO_MPID(cpu, 0);
}

/****************************************************************************
 * Name: arm64_get_cpuid
 *
 * Description:
 *   The function from mpid to get cpu id
 *
 ****************************************************************************/

int arm64_get_cpuid(uint64_t mpid)
{
  return MPID_TO_CORE(mpid);
}

#endif /* CONFIG_SMP */

/****************************************************************************
 * Name: arm64_el_init
 *
 * Description:
 *   The function called from arm64_head.S at very early stage for these
 * platform, it's use to:
 *   - Handling special hardware initialize routine which is need to
 *     run at high ELs
 *   - Initialize system software such as hypervisor or security firmware
 *     which is need to run at high ELs
 *
 ****************************************************************************/

void arm64_el_init(void)
{
}

/****************************************************************************
 * Name: arm64_chip_boot
 *
 * Description:
 *   Complete boot operations started in arm64_head.S
 *
 ****************************************************************************/

void arm64_chip_boot(void)
{
  /* Copy the RAM Disk */

  qemu_copy_ramdisk();

  /* MAP IO and DRAM, enable MMU. */

  *(volatile uint8_t *) 0x02500000 = 'B'; ////
  arm64_mmu_init(true);

  *(volatile uint8_t *) 0x02500000 = 'C'; ////
  arm64_enable_mte();

#ifdef CONFIG_DEVICE_TREE
  fdt_register((const char *)0x40000000);
#endif

#if defined(CONFIG_ARCH_CHIP_QEMU_WITH_HV) && defined(CONFIG_ARM64_PSCI)
  arm64_psci_init("hvc");
#elif defined(CONFIG_ARM64_PSCI)
  arm64_psci_init("smc");
#endif

  /* Perform board-specific device initialization. This would include
   * configuration of board specific resources such as GPIOs, LEDs, etc.
   */

   *(volatile uint8_t *) 0x02500000 = 'D'; ////
   qemu_board_initialize();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

   *(volatile uint8_t *) 0x02500000 = 'E'; ////
   arm64_earlyserialinit();
#endif

#ifdef CONFIG_SYSLOG_RPMSG
  *(volatile uint8_t *) 0x02500000 = 'F'; ////
  syslog_rpmsg_init_early(g_syslog_rpmsg_buf, sizeof(g_syslog_rpmsg_buf));
#endif

#ifdef CONFIG_ARCH_PERF_EVENTS
  *(volatile uint8_t *) 0x02500000 = 'G'; ////
  up_perf_init((void *)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
#endif
}
