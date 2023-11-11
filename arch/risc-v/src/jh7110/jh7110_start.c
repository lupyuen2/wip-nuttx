/****************************************************************************
 * arch/risc-v/src/jh7110/jh7110_start.c
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
#include "jh7110_mm_init.h"
#include "jh7110_memorymap.h"

////TODO
static void jh7110_copy_ramdisk(void);
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

uintptr_t g_idle_topstack = JH7110_IDLESTACK_TOP;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jh7110_clear_bss
 ****************************************************************************/

void jh7110_clear_bss(void)
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
 * Name: jh7110_start
 ****************************************************************************/

void jh7110_start_s(int mhartid)
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

  jh7110_mm_init();

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
 * Name: jh7110_start
 ****************************************************************************/

void jh7110_start(int mhartid)
{
  DEBUGASSERT(mhartid == 0); /* Only Hart 0 supported for now */

  if (0 == mhartid)
    {
      /* Copy the RAM Disk */

      jh7110_copy_ramdisk();

      /* Clear the BSS */

      jh7110_clear_bss();

      /* Initialize the per CPU areas */

      riscv_percpu_add_hart(mhartid);
    }

  /* Disable MMU */

  WRITE_CSR(satp, 0x0);

  /* Set the trap vector for S-mode */

  WRITE_CSR(stvec, (uintptr_t)__trap_vec);

  /* Start S-mode */

  jh7110_start_s(mhartid);
}

void riscv_earlyserialinit(void)
{
  bl602_earlyserialinit();
}

void riscv_serialinit(void)
{
  bl602_serialinit();
}

////TODO
/* Ramdisk Load Address from U-Boot */

#define RAMDISK_ADDR_R  (0x46100000)

static void jh7110_copy_ramdisk(void)
{
  /* Copy Ramdisk from U-Boot Ramdisk Load Address */
  // memcpy((void *)__ramdisk_start, (void *)RAMDISK_ADDR_R,
  //        (size_t)__ramdisk_size);

  // From https://docs.kernel.org/filesystems/romfs.html
  // After _edata, search for "-rom1fs-". This is the RAM Disk Address.
  extern uint8_t _edata[];
  extern uint8_t _sbss[];
  extern uint8_t _ebss[];
  _info("_edata=%p, _sbss=%p, _ebss=%p, JH7110_IDLESTACK_TOP=%p\n", (void *)_edata, (void *)_sbss, (void *)_ebss, JH7110_IDLESTACK_TOP);
  const char *header = "-rom1fs-";
  uint8_t *ramdisk_addr = NULL;
  for (uint8_t *addr = _edata; addr < (uint8_t *)JH7110_IDLESTACK_TOP + (65 * 1024); addr++)
    {
      if (memcmp(addr, header, strlen(header)) == 0)
        {
          ramdisk_addr = addr;
          break;
        }
    }
  _info("ramdisk_addr=%p\n", ramdisk_addr);
  if (ramdisk_addr == NULL) { _info("Missing RAM Disk"); }
  DEBUGASSERT(ramdisk_addr != NULL);  // Missing RAM Disk
  if (ramdisk_addr <= (uint8_t *)JH7110_IDLESTACK_TOP) { _info("RAM Disk must be after Idle Stack"); }
  DEBUGASSERT(ramdisk_addr > (uint8_t *)JH7110_IDLESTACK_TOP);  // RAM Disk must be after Idle Stack
  // ramdisk_addr = 0x50200000 + 0x200288 = 0x50400288
  // _ebss = 50407000
  // __kflash_start = 50200000

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

  _info("Before Copy: ramdisk_addr=%p\n", ramdisk_addr);////
  verify_image(ramdisk_addr);////

  // Copy the Filesystem Size to RAM Disk Start
  // Warning: __ramdisk_start overlaps with ramdisk_addr + size
  // memmove is aliased to memcpy, so we implement memmove ourselves
  local_memmove((void *)__ramdisk_start, ramdisk_addr, size);

  _info("After Copy: __ramdisk_start=%p\n", __ramdisk_start);////
  verify_image(__ramdisk_start);////
}

// grep --binary-files=text -b -o A initrd >/tmp/a
const uint32_t search_addr[] =
{
76654,
78005,
79250,
80482,
81623,
83164,
84603,
86000,
87210,
88767,
90290,
91697,
93104,
94283,
95539,
126251,
323058,
720393,
758145,
804369,
847750,
848662,
896722,
905434,
905602,
905626,
962939,
963865,
1037567,
1101946,
1103297,
1104542,
1105774,
1107121,
1108339,
1109617,
1111173,
1112696,
1114103,
1115510,
1116689,
1117945,
1130677,
1164598,
1164619,
1164977,
1252233,
1253867,
1254889,
1256085,
1257312,
1258427,
1259610,
1260855,
1262143,
1263394,
1265439,
1266357,
1267576,
1269252,
1270691,
1271914,
1273231,
1274536,
1276032,
1277536,
1278734,
1280036,
1281255,
1282447,
1283620,
1284739,
1285849,
1287899,
1288986,
1290163,
1291388,
1292559,
1293736,
1304010,
1352739,
1386719,
1389576,
1839794,
2343673,
2734505,
2877449,
2943582,
2944398,
3093569,
3127373,
3127421,
3179130,
3188633,
3228929,
3241025,
3708930,
3956573,
3956621,
3986249,
4114697,
4242101,
4270145,
4350971,
4351897,
4421558,
4481221,
4482572,
4483817,
4485049,
4486396,
4487897,
4489420,
4490827,
4492234,
4493413,
4494669,
4507446,
4541367,
4541388,
4541746,
4637364,
4638998,
4640020,
4641216,
4642443,
4643558,
4644741,
4645986,
4647274,
4648525,
4650570,
4651488,
4652707,
4654383,
4655822,
4657045,
4658362,
4659667,
4661163,
4662667,
4663865,
4665167,
4666386,
4667605,
4668851,
4670024,
4671143,
4672253,
4674303,
4675390,
4676567,
4677792,
4678963,
4680140,
4726672,
4760652,
4763509,
4787788,
5361481,
5542002,
5550474,
5556865,
6111530,
6113721,
6337414,
6340342,
6442665,
6558441,
6809697,
6809721,
6825142,
6825166,
6826798,
7479309,
7479333,
7533585,
7672939,
7716766,
7718117,
7719362,
7720594,
7721735,
7723247,
7724748,
7726271,
7727678,
7729085,
7730264,
7731520,
7985194,
7988913,
7992730,
};

static void verify_image(uint8_t *addr)
{
  for (int i = 0; i < sizeof(search_addr) / sizeof(search_addr[0]); i++)
    {
      const uint8_t *p = addr + search_addr[i] - 1;
      if (*p != 0x0A) { _info("No Match: %p\n", p); }
    }
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
