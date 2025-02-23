/****************************************************************************
 * arch/risc-v/src/sg2000/sg2000_start.c
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
#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/serial/uart_16550.h>

#include <debug.h>

#include <arch/board/board.h>
#include <arch/board/board_memorymap.h>

#include "riscv_internal.h"
#include "chip.h"
#include "sg2000_mm_init.h"
#include "sg2000_memorymap.h"

//// TODO
struct sbiret_s
{
  intreg_t    error;
  uintreg_t   value;
};
typedef struct sbiret_s sbiret_t;
static void sg2000_boot_secondary(void);
static int riscv_sbi_boot_secondary(uintreg_t hartid, uintreg_t addr);
static sbiret_t sbi_ecall(unsigned int extid, unsigned int fid,
                          uintreg_t parm0, uintreg_t parm1,
                          uintreg_t parm2, uintreg_t parm3,
                          uintreg_t parm4, uintreg_t parm5);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) up_putc(c)
#else
#define showprogress(c)
#endif

#define SBI_EXT_HSM (0x0048534D)
#define SBI_EXT_HSM_HART_START (0x0)

/****************************************************************************
 * Extern Function Declarations
 ****************************************************************************/

 extern void __start(void);
 extern void __trap_vec(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

int boot_hartid = -1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sg2000_copy_overlap
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

static void sg2000_copy_overlap(uint8_t *dest, const uint8_t *src,
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
 * Name: sg2000_copy_ramdisk
 *
 * Description:
 *   Copy the RAM Disk from NuttX Image to RAM Disk Region.
 *
 ****************************************************************************/

static void sg2000_copy_ramdisk(void)
{
  const char *header = "-rom1fs-";
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
  for (addr = _edata; addr < limit; addr++)
    {
      if (memcmp(addr, header, strlen(header)) == 0)
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

  if (ramdisk_addr <= (uint8_t *)g_idle_topstack)
    {
      const size_t pad = (size_t)g_idle_topstack - (size_t)ramdisk_addr;
      _err("RAM Disk must be after Idle Stack. Increase initrd padding "
            "by %ul bytes.", pad);
      PANIC();
    }

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

  sg2000_copy_overlap(__ramdisk_start, ramdisk_addr, size);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sg2000_clear_bss
 *
 * Description:
 *   Clear .bss.  We'll do this inline (vs. calling memset) just to be
 *   certain that there are no issues with the state of global variables.
 *
 ****************************************************************************/

void sg2000_clear_bss(void)
{
  uint32_t *dest;

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }
}

/****************************************************************************
 * Name: sg2000_start_s
 *
 * Description:
 *   Start the NuttX Kernel.  Assume that we are in RISC-V Supervisor Mode.
 *
 * Input Parameters:
 *   mhartid - Hart ID
 *
 ****************************************************************************/

void sg2000_start_s(int mhartid)
{
  /* Configure FPU */

  riscv_fpuconfig();

  if (mhartid != boot_hartid)
    {
      goto cpux;
    }

  /* Boot Hart starts here */

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

  showprogress('C');

  /* Setup page tables for kernel and enable MMU */

  sg2000_mm_init();

  /* Boot the other cores */

  sg2000_boot_secondary();

  /* Call nx_start() */

  nx_start();

cpux:

  /* Non-Boot Hart starts here */

  *(volatile uint8_t *) 0x50900000ul = 'H';
  *(volatile uint8_t *) 0x50900000ul = 'a';
  *(volatile uint8_t *) 0x50900000ul = 'r';
  *(volatile uint8_t *) 0x50900000ul = 't';
  *(volatile uint8_t *) 0x50900000ul = '0' + mhartid;
  *(volatile uint8_t *) 0x50900000ul = '\r';
  *(volatile uint8_t *) 0x50900000ul = '\n';

  riscv_cpu_boot(mhartid);

  while (true)
    {
      asm("WFI");
    }
}

/****************************************************************************
 * Name: sg2000_start
 *
 * Description:
 *   Start the NuttX Kernel.  Called by Boot Code.
 *
 * Input Parameters:
 *   mhartid - Hart ID
 *
 ****************************************************************************/

void sg2000_start(int mhartid)
{
  *(volatile uint8_t *) 0x50900000ul = 'H';
  *(volatile uint8_t *) 0x50900000ul = 'e';
  *(volatile uint8_t *) 0x50900000ul = 'l';
  *(volatile uint8_t *) 0x50900000ul = 'l';
  *(volatile uint8_t *) 0x50900000ul = 'o';
  *(volatile uint8_t *) 0x50900000ul = ' ';
  *(volatile uint8_t *) 0x50900000ul = 'N';
  *(volatile uint8_t *) 0x50900000ul = 'u';
  *(volatile uint8_t *) 0x50900000ul = 't';
  *(volatile uint8_t *) 0x50900000ul = 't';
  *(volatile uint8_t *) 0x50900000ul = 'X';
  *(volatile uint8_t *) 0x50900000ul = '!';
  *(volatile uint8_t *) 0x50900000ul = '\r';
  *(volatile uint8_t *) 0x50900000ul = '\n';

  *(volatile uint8_t *) 0x50900000ul = 'H';
  *(volatile uint8_t *) 0x50900000ul = 'a';
  *(volatile uint8_t *) 0x50900000ul = 'r';
  *(volatile uint8_t *) 0x50900000ul = 't';
  *(volatile uint8_t *) 0x50900000ul = '0' + mhartid;
  *(volatile uint8_t *) 0x50900000ul = '\r';
  *(volatile uint8_t *) 0x50900000ul = '\n';

  /* Init the globals once only. Remember the Boot Hart. */

  if (boot_hartid < 0)
    {
      boot_hartid = mhartid;

      /* Clear the BSS */

      sg2000_clear_bss();

      /* Copy the RAM Disk */

      sg2000_copy_ramdisk();

      /* Initialize the per CPU areas */

      riscv_percpu_add_hart(mhartid);
    }

  /* Disable MMU */

  WRITE_CSR(CSR_SATP, 0x0);

  /* Set the trap vector for S-mode */

  WRITE_CSR(CSR_STVEC, (uintptr_t)__trap_vec);

  /* Start S-mode */

  sg2000_start_s(mhartid);
}

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
  u16550_earlyserialinit();
}

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void riscv_serialinit(void)
{
  u16550_serialinit();
}

/****************************************************************************
 * Name: riscv_hartid_to_cpuid
 *
 * Description:
 *   Convert physical core number to logical core number.
 *
 ****************************************************************************/

int weak_function riscv_hartid_to_cpuid(int hart)
{
  /* Boot Hart is CPU 0. Renumber the Other Harts. */

  if (hart == boot_hartid)
    {
      return 0;
    }
  else if (hart < boot_hartid)
    {
      return hart + 1;
    }
  else
    {
      return hart;
    }
}

/****************************************************************************
 * Name: riscv_cpuid_to_hartid
 *
 * Description:
 *   Convert logical core number to physical core number.
 *
 ****************************************************************************/

int weak_function riscv_cpuid_to_hartid(int cpu)
{
  /* Boot Hart is CPU 0. Renumber the Other Harts. */

  if (cpu == 0)
    {
      return boot_hartid;
    }
  else if (cpu < boot_hartid + 1)
    {
      return cpu - 1;
    }
  else
    {
      return cpu;
    }
}

static void sg2000_boot_secondary(void)
{
  int i;

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      if (i == boot_hartid)
        {
          continue;
        }

      riscv_sbi_boot_secondary(i, (uintptr_t)&__start);
      break; ////
    }
}

static int riscv_sbi_boot_secondary(uintreg_t hartid, uintreg_t addr)
{
  sbiret_t ret = sbi_ecall(SBI_EXT_HSM, SBI_EXT_HSM_HART_START,
                           hartid, addr, 0, 0, 0, 0);

  if (ret.error < 0)
    {
      _err("Boot Hart %d failed\n", hartid);
      PANIC();
    }

  return 0;
}

static sbiret_t sbi_ecall(unsigned int extid, unsigned int fid,
                          uintreg_t parm0, uintreg_t parm1,
                          uintreg_t parm2, uintreg_t parm3,
                          uintreg_t parm4, uintreg_t parm5)
{
  register long r0 asm("a0") = (long)(parm0);
  register long r1 asm("a1") = (long)(parm1);
  register long r2 asm("a2") = (long)(parm2);
  register long r3 asm("a3") = (long)(parm3);
  register long r4 asm("a4") = (long)(parm4);
  register long r5 asm("a5") = (long)(parm5);
  register long r6 asm("a6") = (long)(fid);
  register long r7 asm("a7") = (long)(extid);
  sbiret_t ret;

  asm volatile
    (
     "ecall"
     : "+r"(r0), "+r"(r1)
     : "r"(r2), "r"(r3), "r"(r4), "r"(r5), "r"(r6), "r"(r7)
     : "memory"
     );

  ret.error = r0;
  ret.value = (uintreg_t)r1;

  return ret;
}
