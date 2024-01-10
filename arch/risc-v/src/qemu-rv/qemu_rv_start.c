/****************************************************************************
 * arch/risc-v/src/qemu-rv/qemu_rv_start.c
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
#include <debug.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>
////#include <nuttx/serial/uart_16550.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"

#ifdef CONFIG_BUILD_KERNEL
#  include "qemu_rv_mm_init.h"
#endif

#ifdef CONFIG_DEVICE_TREE
#  include <nuttx/fdt.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#define showprogress(c) up_putc(c)
#else
#define showprogress(c)
#endif

#if defined (CONFIG_BUILD_KERNEL) && !defined (CONFIG_ARCH_USE_S_MODE)
#  error "Target requires kernel in S-mode, enable CONFIG_ARCH_USE_S_MODE"
#endif

/****************************************************************************
 * Extern Function Declarations
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
extern void __trap_vec(void);
extern void __trap_vec_m(void);
extern void up_mtimer_initialize(void);
#endif

/****************************************************************************
 * Name: qemu_rv_clear_bss
 ****************************************************************************/

void qemu_rv_clear_bss(void)
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
 * Public Data
 ****************************************************************************/

/* NOTE: g_idle_topstack needs to point the top of the idle stack
 * for CPU0 and this value is used in up_initial_state()
 */

uintptr_t g_idle_topstack = QEMU_RV_IDLESTACK_TOP;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_rv_start
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
void qemu_rv_start_s(int mhartid, const char *dtb)
#else
void qemu_rv_start(int mhartid, const char *dtb)
#endif
{
  /* Configure FPU */

  riscv_fpuconfig();

  if (mhartid > 0)
    {
      goto cpux;
    }

#ifndef CONFIG_BUILD_KERNEL
  qemu_rv_clear_bss();
#endif

#ifdef CONFIG_DEVICE_TREE
  fdt_register(dtb);
#endif

  showprogress('A');

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  showprogress('B');

  /* Do board initialization */

  showprogress('C');

#ifdef CONFIG_BUILD_KERNEL
  /* Setup page tables for kernel and enable MMU */

  qemu_rv_mm_init();
#endif

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

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Name: qemu_rv_start
 ****************************************************************************/

void qemu_rv_start(int mhartid, const char *dtb)
{
  /* NOTE: still in M-mode */

  if (0 == mhartid)
    {
      qemu_rv_clear_bss();

      /* Initialize the per CPU areas */

      riscv_percpu_add_hart(mhartid);
    }

  /* Disable MMU and enable PMP */

  WRITE_CSR(satp, 0x0);
  WRITE_CSR(pmpaddr0, 0x3fffffffffffffull);
  WRITE_CSR(pmpcfg0, 0xf);

  /* Set exception and interrupt delegation for S-mode */

  WRITE_CSR(medeleg, 0xffff);
  WRITE_CSR(mideleg, 0xffff);

  /* Allow to write satp from S-mode */

  CLEAR_CSR(mstatus, MSTATUS_TVM);

  /* Set mstatus to S-mode and enable SUM */

  CLEAR_CSR(mstatus, ~MSTATUS_MPP_MASK);
  SET_CSR(mstatus, MSTATUS_MPPS | SSTATUS_SUM);

  /* Set the trap vector for S-mode */

  WRITE_CSR(stvec, (uintptr_t)__trap_vec);

  /* Set the trap vector for M-mode */

  WRITE_CSR(mtvec, (uintptr_t)__trap_vec_m);

  if (0 == mhartid)
    {
      /* Only the primary CPU needs to initialize mtimer
       * before entering to S-mode
       */

      up_mtimer_initialize();
    }

  /* Set mepc to the entry */

  WRITE_CSR(mepc, (uintptr_t)qemu_rv_start_s);

  /* Set a0 to mhartid and a1 to dtb explicitly and enter to S-mode */

  asm volatile (
      "mv a0, %0 \n"
      "mv a1, %1 \n"
      "mret \n"
      :: "r" (mhartid), "r" (dtb)
  );
}
#endif

void riscv_earlyserialinit(void)
{
  _info("\n");////
  //// Previously: u16550_earlyserialinit();
}

void riscv_serialinit(void)
{
  _info("\n");////

  // Init the VirtIO Devices
  void qemu_virtio_register_mmio_devices(void);
  qemu_virtio_register_mmio_devices();

  //// Previously: u16550_serialinit();
}

/* Output Log:
 *  Executing task: cd nuttx && ../run.sh 

+ git pull
Already up-to-date.
+ git status
On branch tinyemu2
Your branch is up-to-date with 'origin/tinyemu2'.
Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git checkout -- <file>..." to discard changes in working directory)

        modified:   arch/risc-v/src/qemu-rv/qemu_rv_irq_dispatch.c
        modified:   boards/risc-v/qemu-rv/rv-virt/src/qemu_rv_appinit.c
        modified:   drivers/virtio/virtio-mmio.c

no changes added to commit (use "git add" and/or "git commit -a")
++ git rev-parse HEAD
+ hash1=befabcc71508f5d498c3092eb0d1a4288243b726
+ pushd ../apps
~/riscv/apps ~/riscv/nuttx
+ git pull
Already up-to-date.
+ git status
On branch tinyemu2
Your branch is up-to-date with 'origin/tinyemu2'.
nothing to commit, working tree clean
++ git rev-parse HEAD
+ hash2=cf27f085f56709ca5e1a31e4a91ca9e90dd69c79
+ popd
~/riscv/nuttx
+ echo NuttX Source: https://github.com/apache/nuttx/tree/befabcc71508f5d498c3092eb0d1a4288243b726
+ echo NuttX Apps: https://github.com/apache/nuttx-apps/tree/cf27f085f56709ca5e1a31e4a91ca9e90dd69c79
+ riscv64-unknown-elf-gcc -v
Using built-in specs.
COLLECT_GCC=riscv64-unknown-elf-gcc
COLLECT_LTO_WRAPPER=/Users/Luppy/riscv64-unknown-elf-toolchain-10.2.0-2020.12.8-x86_64-apple-darwin/bin/../libexec/gcc/riscv64-unknown-elf/10.2.0/lto-wrapper
Target: riscv64-unknown-elf
Configured with: /scratch/jenkins/workspace/tpp-freedom-tools/tpp01--build-binary-packages--parameterized/obj/x86_64-apple-darwin/build/riscv64-unknown-elf-gcc/riscv-gcc/configure --target=riscv64-unknown-elf --prefix=/scratch/jenkins/workspace/tpp-freedom-tools/tpp01--build-binary-packages--parameterized/obj/x86_64-apple-darwin/install/riscv64-unknown-elf-gcc-10.2.0-2020.12.8-x86_64-apple-darwin --with-pkgversion='SiFive GCC-Metal 10.2.0-2020.12.8' --with-bugurl=https://github.com/sifive/freedom-tools/issues --disable-shared --disable-threads --enable-languages=c,c++ --enable-tls --with-newlib --with-sysroot=/scratch/jenkins/workspace/tpp-freedom-tools/tpp01--build-binary-packages--parameterized/obj/x86_64-apple-darwin/install/riscv64-unknown-elf-gcc-10.2.0-2020.12.8-x86_64-apple-darwin/riscv64-unknown-elf --with-native-system-header-dir=/include --disable-libmudflap --disable-libssp --disable-libquadmath --disable-libgomp --disable-nls --disable-tm-clone-registry --src=../riscv-gcc --with-system-zlib --enable-checking=yes --enable-multilib --with-abi=lp64d --with-arch=rv64imafdc CFLAGS=-O2 CXXFLAGS=-O2 'CFLAGS_FOR_TARGET=-Os -mcmodel=medany' 'CXXFLAGS_FOR_TARGET=-Os -mcmodel=medany'
Thread model: single
Supported LTO compression algorithms: zlib
gcc version 10.2.0 (SiFive GCC-Metal 10.2.0-2020.12.8) 
+ build_nuttx
+ pushd ../nuttx
~/riscv/nuttx ~/riscv/nuttx
+ make -j 8
CPP:  /Users/Luppy/riscv/nuttx/boards/risc-v/qemu-rv/rv-virt/scripts/ld.script-> /Users/Luppy/riscv/nLD: nuttx
CP: nuttx.hex
+ popd
~/riscv/nuttx
+ riscv64-unknown-elf-size nuttx
   text    data     bss     dec     hex filename
 268577     833   10600  280010   445ca nuttx
+ riscv64-unknown-elf-objcopy -O binary nuttx nuttx.bin
+ cp .config nuttx.config
+ riscv64-unknown-elf-objdump --syms --source --reloc --demangle --line-numbers --wide --debugging nuttx
+ set +e
+ ../nxstyle arch/risc-v/src/bl808/bl808_timerisr.c
+ ../nxstyle arch/risc-v/src/common/riscv_mmu.h
+ ../nxstyle arch/risc-v/src/common/riscv_mmu.c
+ set -e
+ wget --output-document=nuttx.cfg https://raw.githubusercontent.com/lupyuen/nuttx-tinyemu/main/docs/root-riscv64.cfg
--2024-01-10 09:16:01--  https://raw.githubusercontent.com/lupyuen/nuttx-tinyemu/main/docs/root-riscv64.cfg
Resolving raw.githubusercontent.com (raw.githubusercontent.com)... 185.199.108.133, 185.199.109.133, 185.199.111.133, ...
Connecting to raw.githubusercontent.com (raw.githubusercontent.com)|185.199.108.133|:443... connected.
HTTP request sent, awaiting response... 200 OK
Length: 109 [text/plain]
Saving to: ‘nuttx.cfg’

nuttx.cfg                 100%[==================================>]     109  --.-KB/s    in 0s      

2024-01-10 09:16:01 (4.33 MB/s) - ‘nuttx.cfg’ saved [109/109]

+ cp nuttx.cfg ../nuttx-tinyemu/docs/tinyemu2/root-riscv64.cfg
+ cp nuttx.bin ../nuttx-tinyemu/docs/tinyemu2/
+ cp nuttx.S ../nuttx-tinyemu/docs/tinyemu2/
+ cp nuttx.hash ../nuttx-tinyemu/docs/tinyemu2/
+ cp nuttx.config ../nuttx-tinyemu/docs/tinyemu2/
+ echo http://localhost:8080
http://localhost:8080
+ temu nuttx.cfg
+ echo simple-http-server /Users/Luppy/riscv/nuttx-tinyemu/docs/
simple-http-server /Users/Luppy/riscv/nuttx-tinyemu/docs/
123Ariscv_earlyserialinit: 
BCnx_start: Entry
mm_initialize: Heap: name=Umem, start=0x80044e48 size=33272248
mm_addregion: [Umem] Region 1: base=0x800450f8 size=33271552
mm_malloc: Allocated 0x80045120, size 48
mm_malloc: Allocated 0x80045150, size 288
mm_malloc: Allocated 0x80045270, size 32
mm_malloc: Allocated 0x80045290, size 720
mm_malloc: Allocated 0x80045560, size 80
mm_malloc: Allocated 0x800455b0, size 64
mm_malloc: Allocated 0x800455f0, size 240
mm_malloc: Allocated 0x800456e0, size 464
mm_malloc: Allocated 0x800458b0, size 176
mm_malloc: Allocated 0x80045960, size 336
mm_malloc: Allocated 0x80045ab0, size 464
mm_malloc: Allocated 0x80045c80, size 464
mm_malloc: Allocated 0x80045e50, size 528
builtin_initialize: Registering Builtin Loader
elf_initialize: Registering ELF
riscv_serialinit: 
mm_malloc: Allocated 0x80046060, size 336
virtio_mmio_init_device: VIRTIO version: 2 device: 3 vendor: ffff
mm_malloc: Allocated 0x800461b0, size 48
test_queue: test_queue: 0x80046060
mm_malloc: Allocated 0x800461e0, size 80
mm_malloc: Allocated 0x80046230, size 80
mm_malloc: Allocated 0x80046280, size 80
mm_malloc: Allocated 0x800462d0, size 400
mm_malloc: Allocated 0x80046460, size 272
mm_malloc: Allocated 0x80046570, size 272
mm_malloc: Allocated 0x80046680, size 96
mm_malloc: Allocated 0x800466e0, size 368
mm_malloc: Allocated 0x80046850, size 12448
mm_malloc: Allocated 0x80046850, size 368
mm_malloc: Allocated 0x80048090, size 12448
up_enable_irq: irq=28, extirq=1
uart_register: Registering /dev/console
mm_malloc: Allocated 0x800469c0, size 80
virtio_register_serial_driver: ret1=0
virtio_register_serial_driver: ret2=0
mm_malloc: Allocated 0x80046a10, size 32
mm_malloc: Allocated 0x80046a30, size 160
mm_malloc: Allocated 0x80046ad0, size 32
mm_malloc: Allocated 0x80046af0, size 32
mm_malloc: Allocated 0x80046b10, size 32
nx_start_application: Starting init thread
task_spawn: name=nsh_main entry=0x800089d6 file_actions=0 attr=0x80044db8 argv=0x80044db0
mm_malloc: Allocated 0x80046b30, size 272
mm_malloc: Allocated 0x80046c40, size 288
mm_malloc: Allocated 0x80046d60, size 32
mm_malloc: Allocated 0x80048090, size 720
mm_malloc: Allocated 0x80046d80, size 32
mm_malloc: Allocated 0x80046da0, size 32
mm_malloc: Allocated 0x80046dc0, size 32
mm_malloc: Allocated 0x80046de0, size 32
mm_malloc: Allocated 0x80046e00, size 160
mm_malloc: Allocated 0x80048360, size 3088
mm_free: Freeing 0x80046ad0
mm_free: Freeing 0x80046b10
mm_free: Freeing 0x80046af0
mm_malloc: Allocated 0x80046ad0, size 80
test_virtio: 
test_queue: test_queue: 0
test_queue: TX index=1, entries=16
test_queue: RX index=0, entries=16
test_queue: test_queue: 0
test_queue: TX index=1, entries=16
test_queue: RX index=0, entries=16
test_queue: test_queue: 0
test_queue: TX index=1, entries=16
test_queue: RX index=0, entries=16
test_queue: test_queue: 0
test_queue: TX index=1, entries=16
test_queue: RX index=0, entries=16
test_queue: test_queue: 0
test_queue: TX index=1, entries=16
test_queue: RX index=0, entries=16
test_queue: test_queue: 0
test_queue: TX index=1, entries=16
test_queue: RX index=0, entries=16
test_queue: test_queue: 0
test_queue: TX index=1, entries=16
test_queue: RX index=0, entries=16
test_queue: test_queue: 0
test_queue: TX index=1, entries=16
test_queue: RX index=0, entries=16
test_queue: test_queue: 0
test_queue: TX index=1, entries=16
test_queue: RX index=0, entries=16
test_queue: test_queue: 0
test_queue: TX index=1, entries=16
test_queue: RX index=0, entries=16
mm_malloc: Allocated 0x8004a090, size 848

NuttShell (NSH) NuttX-12.3.0-RC1
nx_start: CPU0: Beginning Idle Loop
*/