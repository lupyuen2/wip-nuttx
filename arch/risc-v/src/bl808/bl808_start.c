/****************************************************************************
 * arch/risc-v/src/bl808/bl808_start.c
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
#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/serial/uart_16550.h>

#include <debug.h>

#include <arch/board/board.h>
#include <arch/board/board_memorymap.h>

#include "riscv_internal.h"
#include "chip.h"
#include "bl808_mm_init.h"
#include "bl808_memorymap.h"

////TODO
static void bl808_copy_ramdisk(void);
static void verify_image(uint8_t *addr);
static FAR void *local_memmove(FAR void *dest, FAR const void *src, size_t count);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) up_putc(c)
#else
#define showprogress(c)
#endif

/****************************************************************************
 * Extern Function Declarations
 ****************************************************************************/

extern void __trap_vec(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* NOTE: g_idle_topstack needs to point the top of the idle stack
 * for CPU0 and this value is used in up_initial_state()
 */

uintptr_t g_idle_topstack = BL808_IDLESTACK_TOP;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_clear_bss
 ****************************************************************************/

void bl808_clear_bss(void)
{
  uint32_t *dest;

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }
}

/****************************************************************************
 * Name: bl808_start
 ****************************************************************************/

void bl808_start_s(int mhartid)
{
  /* Configure FPU */

  riscv_fpuconfig();

  if (mhartid > 0)
    {
      goto cpux;
    }

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

  showprogress('C');

  /* Setup page tables for kernel and enable MMU */

  bl808_mm_init();

  /* Call nx_start() */

  nx_start();

cpux:

#ifdef CONFIG_SMP
  riscv_cpu_boot(mhartid);
#endif

  while (true)
    {
      asm("WFI");
    }
}

/****************************************************************************
 * Name: bl808_start
 ****************************************************************************/

void bl808_start(int mhartid)
{
  DEBUGASSERT(mhartid == 0); /* Only Hart 0 supported for now */

  if (0 == mhartid)
    {
      /* Copy the RAM Disk */

      bl808_copy_ramdisk();

      /* Clear the BSS */

      bl808_clear_bss();

      /* Initialize the per CPU areas */

      riscv_percpu_add_hart(mhartid);
    }

  /* Disable MMU */

  WRITE_CSR(satp, 0x0);

  /* Set the trap vector for S-mode */

  WRITE_CSR(stvec, (uintptr_t)__trap_vec);

  /* Start S-mode */

  bl808_start_s(mhartid);
}

void riscv_earlyserialinit(void)
{
  bl808_earlyserialinit();
}

void riscv_serialinit(void)
{
  bl808_serialinit();
}

////TODO
/* Ramdisk Load Address from U-Boot */

#define RAMDISK_ADDR_R  (0x46100000)

static void bl808_copy_ramdisk(void)
{
  /* Copy Ramdisk from U-Boot Ramdisk Load Address */
  // memcpy((void *)__ramdisk_start, (void *)RAMDISK_ADDR_R,
  //        (size_t)__ramdisk_size);

  // From https://docs.kernel.org/filesystems/romfs.html
  // After _edata, search for "-rom1fs-". This is the RAM Disk Address.
  // Limit search to 256 KB after Idle Stack Top.
  extern uint8_t _edata[];
  extern uint8_t _sbss[];
  extern uint8_t _ebss[];
  _info("_edata=%p, _sbss=%p, _ebss=%p, BL808_IDLESTACK_TOP=%p\n", (void *)_edata, (void *)_sbss, (void *)_ebss, BL808_IDLESTACK_TOP);
  const char *header = "-rom1fs-";
  uint8_t *ramdisk_addr = NULL;
  for (uint8_t *addr = _edata; addr < (uint8_t *)BL808_IDLESTACK_TOP + (256 * 1024); addr++)
    {
      if (memcmp(addr, header, strlen(header)) == 0)
        {
          ramdisk_addr = addr;
          break;
        }
    }

  // Stop if RAM Disk is missing
  _info("ramdisk_addr=%p\n", ramdisk_addr);
  if (ramdisk_addr == NULL) { _info("Missing RAM Disk. Check the initrd padding."); }
  DEBUGASSERT(ramdisk_addr != NULL);

  // RAM Disk must be after Idle Stack, to prevent overwriting
  if (ramdisk_addr <= (uint8_t *)BL808_IDLESTACK_TOP) { _info("RAM Disk must be after Idle Stack. Increase the initrd padding by %ul bytes.", (size_t)BL808_IDLESTACK_TOP - (size_t)ramdisk_addr); }
  DEBUGASSERT(ramdisk_addr > (uint8_t *)BL808_IDLESTACK_TOP);

  // Read the Filesystem Size from the next 4 bytes, in Big Endian
  // Add 0x1F0 to Filesystem Size
  const uint32_t size =
    (ramdisk_addr[8] << 24) + 
    (ramdisk_addr[9] << 16) + 
    (ramdisk_addr[10] << 8) + 
    ramdisk_addr[11] + 
    0x1F0;
  _info("size=%d\n", size);

  // Filesystem Size must be less than RAM Disk Memory Region
  DEBUGASSERT(size <= (size_t)__ramdisk_size);

  // _info("Before Copy: ramdisk_addr=%p\n", ramdisk_addr);////
  // verify_image(ramdisk_addr);////

  // Copy the Filesystem Size to RAM Disk Start
  // Warning: __ramdisk_start overlaps with ramdisk_addr + size
  // memmove is aliased to memcpy, so we implement memmove ourselves
  local_memmove((void *)__ramdisk_start, ramdisk_addr, size);

  // _info("After Copy: __ramdisk_start=%p\n", __ramdisk_start);////
  // verify_image(__ramdisk_start);////
}

// From libs/libc/string/lib_memmove.c
static FAR void *local_memmove(FAR void *dest, FAR const void *src, size_t count)
{
  FAR char *d;
  FAR char *s;

  // if (dest <= src)
  //   {
  //     tmp = (FAR char *) dest;
  //     s   = (FAR char *) src;

  //     while (count--)
  //       {
  //         *tmp++ = *s++;
  //       }
  //   }
  // else
    {
      DEBUGASSERT(dest > src); ////
      d = (FAR char *) dest + count;
      s = (FAR char *) src + count;

      while (count--)
        {
          d -= 1;
          s -= 1;
          ////TODO: Very strange. This needs to be volatile or C Compiler will replace this by memcpy.
          volatile char c = *s;
          *d = c;
        }
    }

  return dest;
}
