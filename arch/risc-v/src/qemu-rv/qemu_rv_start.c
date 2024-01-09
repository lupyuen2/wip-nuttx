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

  // Register the VirtIO Serial Driver
  int virtio_register_serial_driver(void);
  int ret = virtio_register_serial_driver();
  DEBUGASSERT(ret >= 0);

  //// Previously: u16550_serialinit();
}

/* Output Log:
+ git pull
Already up-to-date.
+ git status
On branch tinyemu2
Your branch is up-to-date with 'origin/tinyemu2'.
nothing to commit, working tree clean
++ git rev-parse HEAD
+ hash1=3a85e8b60ac0ff7c24a1b1bb52ebb3978ea14f1a
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
+ echo NuttX Source: https://github.com/apache/nuttx/tree/3a85e8b60ac0ff7c24a1b1bb52ebb3978ea14f1a
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
Create version.h
CPP:  /Users/Luppy/riscv/nuttx/boards/risc-v/qemu-rv/rv-virt/scripts/ld.script-> /Users/Luppy/riscv/nLD: nuttx
CP: nuttx.hex
+ popd
~/riscv/nuttx
+ riscv64-unknown-elf-size nuttx
   text    data     bss     dec     hex filename
 267893     833   10592  279318   44316 nuttx
+ riscv64-unknown-elf-objcopy -O binary nuttx nuttx.bin
+ cp .config nuttx.config
+ riscv64-unknown-elf-objdump --syms --source --reloc --demangle --line-numbers --wide --debugging nuttx
+ set +e
+ ../nxstyle arch/risc-v/src/bl808/bl808_timerisr.c
+ ../nxstyle arch/risc-v/src/common/riscv_mmu.h
+ ../nxstyle arch/risc-v/src/common/riscv_mmu.c
+ set -e
+ wget --output-document=nuttx.cfg https://raw.githubusercontent.com/lupyuen/nuttx-tinyemu/main/docs/root-riscv64.cfg
--2024-01-09 19:23:43--  https://raw.githubusercontent.com/lupyuen/nuttx-tinyemu/main/docs/root-riscv64.cfg
Resolving raw.githubusercontent.com (raw.githubusercontent.com)... 185.199.108.133, 185.199.109.133, 185.199.110.133, ...
Connecting to raw.githubusercontent.com (raw.githubusercontent.com)|185.199.108.133|:443... connected.
HTTP request sent, awaiting response... 200 OK
Length: 109 [text/plain]
Saving to: ‘nuttx.cfg’

nuttx.cfg                 100%[==================================>]     109  --.-KB/s    in 0s      

2024-01-09 19:23:43 (4.33 MB/s) - ‘nuttx.cfg’ saved [109/109]

+ echo http://localhost:8080
http://localhost:8080
+ echo simple-http-server /Users/Luppy/riscv/nuttx-tinyemu/docs/
simple-http-server /Users/Luppy/riscv/nuttx-tinyemu/docs/
+ sleep 10
+ temu nuttx.cfg
123Ariscv_earlyserialinit: 
BCnx_start: Entry
mm_initialize: Heap: name=Umem, start=0x80044b90 size=33272944
mm_addregion: [Umem] Region 1: base=0x80044e38 size=33272256
mm_malloc: Allocated 0x80044e60, size 48
mm_malloc: Allocated 0x80044e90, size 288
mm_malloc: Allocated 0x80044fb0, size 32
mm_malloc: Allocated 0x80044fd0, size 720
mm_malloc: Allocated 0x800452a0, size 80
mm_malloc: Allocated 0x800452f0, size 64
mm_malloc: Allocated 0x80045330, size 240
mm_malloc: Allocated 0x80045420, size 464
mm_malloc: Allocated 0x800455f0, size 176
mm_malloc: Allocated 0x800456a0, size 336
mm_malloc: Allocated 0x800457f0, size 464
mm_malloc: Allocated 0x800459c0, size 464
mm_malloc: Allocated 0x80045b90, size 528
builtin_initialize: Registering Builtin Loader
elf_initialize: Registering ELF
riscv_serialinit: 
mm_malloc: Allocated 0x80045da0, size 336
virtio_mmio_init_device: VIRTIO version: 2 device: 3 vendor: ffff
mm_malloc: Allocated 0x80045ef0, size 48
mm_malloc: Allocated 0x80045f20, size 400
mm_malloc: Allocated 0x800460b0, size 272
mm_malloc: Allocated 0x800461c0, size 272
mm_malloc: Allocated 0x800462d0, size 96
mm_malloc: Allocated 0x80046330, size 368
mm_malloc: Allocated 0x800464a0, size 12448
mm_malloc: Allocated 0x800464a0, size 368
mm_malloc: Allocated 0x80048090, size 12448
uart_register: Registering /dev/ttyV0
mm_malloc: Allocated 0x80046610, size 80
mm_malloc: Allocated 0x80046660, size 80
mm_malloc: Allocated 0x800466b0, size 80
mm_malloc: Allocated 0x80046700, size 80
mm_malloc: Allocated 0x80046750, size 400
mm_malloc: Allocated 0x800468e0, size 272
mm_malloc: Allocated 0x800469f0, size 272
mm_malloc: Allocated 0x80046b00, size 96
mm_malloc: Allocated 0x80046b60, size 368
mm_malloc: Allocated 0x8004a090, size 12448
virtio_mmio_config_virtqueue: Virtio queue not ready
virtio_mmio_create_virtqueue: virtio_mmio_config_virtqueue failed, ret=-2
mm_free: Freeing 0x8004b000
mm_free: Freeing 0x80046b60
mm_free: Freeing 0x80046b00
virtio_serial_init: virtio_device_create_virtqueue failed, ret=-2
mm_free: Freeing 0x800469f0
mm_free: Freeing 0x800468e0
virtio_serial_probe: virtio_serial_init failed, ret=-2
mm_free: Freeing 0x80046750
group_setupidlefiles: ERROR: Failed to open stdin: -2
_assert: Current Version: NuttX  12.3.0-RC1 3a85e8b Jan  9 2024 19:23:38 risc-v
_assert: Assertion failed group_setupidlefiles(): at file: init/nx_start.c:666 task: Idle_Task process: Kernel 0x80000774
up_dump_register: EPC: 0000000080016ed4
up_dump_register: A0: 0000000080043010 A1: 000000000000029a A2: 00000000800269a0 A3: 000000000000007e
up_dump_register: A4: 0000000080042240 A5: 0000000000000002 A6: 0000000000000009 A7: 0000000080044b90
up_dump_register: T0: 000000000000002e T1: 000000000000006a T2: 00000000000001ff T3: 000000000000006c
up_dump_register: T4: 0000000000000068 T5: 0000000000000009 T6: 000000000000002a
up_dump_register: S0: 0000000000000000 S1: 0000000080042240 S2: 000000008004425c S3: 0000000000000000
up_dump_register: S4: 00000000800269a0 S5: 0000000080026958 S6: 8000000a00006008 S7: 0000000080044250
up_dump_register: S8: 000000000000029a S9: 0000000000000000 S10: 0000000000000000 S11: 0000000000000000
up_dump_register: SP: 00000000800449e0 FP: 0000000000000000 TP: 0000000000000000 RA: 0000000080016ed4
dump_stack: User Stack:
dump_stack:   base: 0x800443a0
dump_stack:   size: 00002032
dump_stack:     sp: 0x800449e0
stack_dump: 0x800449c0: 00000001 00000000 80042240 00000000 800449e0 00000000 80017024 00000000
stack_dump: 0x800449e0: 80000774 00000000 80026e55 00000000 00000000 00000000 00000000 00000000
stack_dump: 0x80044a00: 80042240 00000000 80043010 00000000 800269a0 00000000 80026958 00000000
stack_dump: 0x80044a20: 0000029a 00000000 7474754e 00000058 00000000 00000000 80042230 00000000
stack_dump: 0x80044a40: 8004425c 00000000 80044260 00000000 80044ab8 00000000 80018c92 2e323100
stack_dump: 0x80044a60: 2d302e33 00314352 80026e78 00000000 61330003 38653538 614a2062 3920206e
stack_dump: 0x80044a80: 32303220 39312034 3a33323a 00003833 80006eaa 00000000 0000000a 00000000
stack_dump: 0x80044aa0: fffffffe 736972ff 00762d63 00000000 80026e50 00000000 80044af0 00000000
stack_dump: 0x80044ac0: 8004425c 00000000 00000000 00000000 00000000 00000000 00000000 00000000
stack_dump: 0x80044ae0: 00000000 00000000 00000000 00000000 00000000 00000000 80042230 00000000
stack_dump: 0x80044b00: 8004425c 00000000 80044260 00000000 80042240 00000000 80005c44 00000000
stack_dump: 0x80044b20: 80042240 00000000 8000088e 00000000 80044b90 00000000 01fbb470 00000000
stack_dump: 0x80044b40: 800268d0 00000000 00000000 00000000 00000000 00000000 00000000 00000000
stack_dump: 0x80044b60: 00001040 00000000 80000624 00000000 00000000 00000000 00000000 00000000
stack_dump: 0x80044b80: 00000000 00000000 80000078 00000000 00000000 00000000 00000000 00000000
dump_tasks:    PID GROUP PRI POLICY   TYPE    NPX STATE   EVENT      SIGMASK          STACKBASE  STACKSIZE      USED   FILLED    COMMAND
dump_tasks:   ----   --- --- -------- ------- --- ------- ---------- ---------------- 0x80041a30      2048         0     0.0%    irq
dump_task:       0     0   0 FIFO     Kthread - Running            0000000000000000 0x800443a0      2032      1016    50.0%    Idle_Task
*/