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
  memcpy((void *)__ramdisk_start, ramdisk_addr, size);

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
1037559,
1101938,
1103289,
1104534,
1105766,
1107113,
1108331,
1109609,
1111165,
1112688,
1114095,
1115502,
1116681,
1117937,
1130669,
1164590,
1164611,
1164969,
1252225,
1253859,
1254881,
1256077,
1257304,
1258419,
1259602,
1260847,
1262135,
1263386,
1265431,
1266349,
1267568,
1269244,
1270683,
1271906,
1273223,
1274528,
1276024,
1277528,
1278726,
1280028,
1281247,
1282439,
1283612,
1284731,
1285841,
1287891,
1288978,
1290155,
1291380,
1292551,
1293728,
1304002,
1352731,
1386711,
1389568,
1839786,
2343665,
2734497,
2877441,
2943574,
2944390,
3093561,
3127365,
3127413,
3179122,
3188625,
3228921,
3241017,
3708922,
3956565,
3956613,
3986241,
4114689,
4242093,
4270137,
4350955,
4351881,
4421534,
4481197,
4482548,
4483793,
4485025,
4486372,
4487873,
4489396,
4490803,
4492210,
4493389,
4494645,
4507422,
4541343,
4541364,
4541722,
4637340,
4638974,
4639996,
4641192,
4642419,
4643534,
4644717,
4645962,
4647250,
4648501,
4650546,
4651464,
4652683,
4654359,
4655798,
4657021,
4658338,
4659643,
4661139,
4662643,
4663841,
4665143,
4666362,
4667581,
4668827,
4670000,
4671119,
4672229,
4674279,
4675366,
4676543,
4677768,
4678939,
4680116,
4726648,
4760628,
4763485,
4787764,
5361457,
5541978,
5550450,
5556841,
6111506,
6113697,
6337390,
6340318,
6442641,
6558417,
6809673,
6809697,
6825118,
6825142,
6826774,
7479285,
7479309,
7533561,
7672915,
7716750,
7718101,
7719346,
7720578,
7721719,
7723231,
7724732,
7726255,
7727662,
7729069,
7730248,
7731504,
7985178,
7988897,
7992714,
};

static void verify_image(uint8_t *addr)
{
  for (int i = 0; i < sizeof(search_addr) / sizeof(search_addr[0]); i++)
    {
      const uint8_t *p = addr + search_addr[i] - 1;
      if (*p != 0x0A) { _info("No Match: %p\n", p); }
    }
}
