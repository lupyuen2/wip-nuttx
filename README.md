# On-Demand Paging with Apache NuttX RTOS on Ox64 BL808 SBC

We're testing On-Demand Paging with Apache NuttX RTOS on Ox64 BL808 SBC: https://github.com/apache/nuttx/pull/11824

On QEMU 32-bit RISC-V: It runs like this: https://gist.github.com/lupyuen/6075f31b575b54108e60b028083c16f7

```yaml
+ qemu-system-riscv32 -semihosting -M virt,aclint=on -cpu rv32 -smp 8 -bios none -kernel nuttx -nographic
ABCnx_start: Entry
uart_register: Registering /dev/console
uart_register: Registering /dev/ttyS0
work_start_lowpri: Starting low-priority kernel worker thread(s)
nxtask_activate: lpwork pid=1,TCB=0x80407ab0
nxtask_activate: AppBringUp pid=2,TCB=0x80407e70
nx_start_application: Starting init task: /system/bin/init
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 0000000f, EPC: 80010698, MTVAL: c0001000
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 0000000f, EPC: 80010698, MTVAL: c0002000
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 0000000f, EPC: 80010698, MTVAL: c0003000
...
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 0000000f, EPC: 80012fe4, MTVAL: c0900c04
nxtask_activate: /system/bin/init pid=3,TCB=0x80408458
nxtask_exit: AppBringUp pid=2,TCB=0x80407e70

NuttShell (NSH) NuttX-12.4.0-RC0
nsh> nx_start: CPU0: Beginning Idle Loop
```

We see different results on Ox64 Device and Ox64 Emulator. Let's investigate...

# Test on Ox64 Device

Based on this NuttX Config: https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/boards/risc-v/bl808/ox64/configs/nsh_paging/defconfig#L73

Ox64 SBC fails to start NSH: https://gist.github.com/lupyuen/224cac41efa1db0bebda1414de49eed1

```text
ABCnx_start: Entry
uart_register: Registering /dev/console
work_start_lowpri: Starting low-priority kernel worker thread(s)
nxtask_activate: lpwork pid=1,TCB=0x504092f0
nxtask_activate: AppBringUp pid=2,TCB=0x50409900
nx_start_application: Starting init task: /system/bin/init

riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 000000000000000f, EPC: 000000005020a0b0, MTVAL: 0000000080001000
riscv_fillpage: ARCH_TEXT_SIZE=0x80000
riscv_fillpage: ARCH_TEXT_VEND=0x80080000
riscv_fillpage: vaddr=0x80001000
riscv_fillpage: FIX_ARCH_TEXT_VEND=0x80080000
riscv_fillpage: vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr <= ARCH_TEXT_VEND
riscv_fillpage: !paddr1
riscv_fillpage: mmu_ln_setentry1
riscv_fillpage: riscv_pgwipe1
riscv_fillpage: riscv_pgvaddr
riscv_fillpage: mm_pgalloc
riscv_fillpage: riscv_pgwipe2
riscv_fillpage: mmu_ln_setentry2
riscv_fillpage: return

riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 000000000000000f, EPC: 000000005020a0b0, MTVAL: 0000000080001000
riscv_fillpage: ARCH_TEXT_SIZE=0x80000
riscv_fillpage: ARCH_TEXT_VEND=0x80080000
riscv_fillpage: vaddr=0x80001000
riscv_fillpage: FIX_ARCH_TEXT_VEND=0x80080000
riscv_fillpage: vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr <= ARCH_TEXT_VEND
riscv_fillpage: riscv_pgvaddr
riscv_fillpage: mm_pgalloc
riscv_fillpage: riscv_pgwipe2
riscv_fillpage: mmu_ln_setentry2
riscv_fillpage: return
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 000000000000000f, EPC: 000000005020a0b0, MTVAL: 0000000080001000
```

riscv_fillpage is here: https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/arch/risc-v/src/common/riscv_exception.c#L97-L239

TODO: It loops forever at the same MCAUSE, EPC, MTVAL. Why?

We'll come back to Ox64 Device later, we'll fix Ox64 Emulator first. Why?

1.  Testing on Ox64 Emulator is a lot faster. No need to swap and swap and swap microSD Card!

1.  Ox64 Emulator behaves differently from Ox64 Device for On-Demand Paging. It's important that we make Ox64 Emulator work the exact same way as Ox64 Device, to prevent testing problems in future.

We jump to Ox64 Emulator for now...

# Test on Ox64 Emulator

Based on this NuttX Config: https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/boards/risc-v/bl808/ox64/configs/nsh_paging/defconfig#L73

Ox64 Emulator also fails to start NSH: https://gist.github.com/lupyuen/a9821b6867e98fb67c379f1fd842819a

```text
ABCnx_start: Entry
uart_register: Registering /dev/console
work_start_lowpri: Starting low-priority kernel worker thread(s)
nxtask_activate: lpwork pid=1,TCB=0x504092f0
nxtask_activate: AppBringUp pid=2,TCB=0x50409900
nx_start_application: Starting init task: /system/bin/init
raise_exception2: cause=15, tval=0x80001000, pc=0x5020a0b0
pc =000000005020a0b0 ra =0000000050211570 sp =000000005040c510 gp =0000000000000000
tp =0000000000000000 t0 =0000000000000003 t1 =0000000000000007 t2 =0000000000000020
s0 =000000000000aa00 s1 =0000000000000000 a0 =0000000080000020 a1 =0000000050ade000
a2 =000000000000aa00 a3 =000000000000000d a4 =0000000080001000 a5 =0000000000000fe1
a6 =00000000000006f0 a7 =0000000080000020 s2 =000000000000ab7c s3 =0000000000000055
s4 =000000005040ab20 s5 =000000005040a900 s6 =0000000080000020 s7 =000000005040a918
s8 =000000005040c6d8 s9 =0000000000000000 s10=00000000000006f0 s11=0000000000000000
t3 =000000005040c50c t4 =000000005040c500 t5 =00000000000000ff t6 =000000000000000f
priv=S mstatus=0000000a000400a2 cycles=97521621
 mideleg=0000000000000222 mie=0000000000000220 mip=0000000000000080
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 000000000000000f, EPC: 000000005020a0b0, MTVAL: 0000000080001000
riscv_fillpage: ARCH_TEXT_SIZE=0x80000
riscv_fillpage: ARCH_TEXT_VEND=0x80080000
riscv_fillpage: vaddr=0x80001000
riscv_fillpage: FIX_ARCH_TEXT_VEND=0x80080000
riscv_fillpage: vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr <= ARCH_TEXT_VEND
riscv_fillpage: !paddr1
riscv_fillpage: mmu_ln_setentry1
raise_exception2: cause=13, tval=0x30002084, pc=0x50200b48
pc =0000000050200b48 ra =0000000050200d04 sp =0000000050400830 gp =0000000000000000
tp =0000000000000000 t0 =000000000000002e t1 =000000000000006a t2 =00000000000001ff
s0 =0000000000000072 s1 =0000000200042120 a0 =0000000050400110 a1 =0000000000000072
a2 =000000000000000e a3 =0000000000000053 a4 =0000000030002000 a5 =000000000000000a
a6 =000000000000003f a7 =0000000000000014 s2 =0000000050400270 s3 =000000005021ad8e
s4 =000000005021ad80 s5 =0000000000000030 s6 =000000005021ab52 s7 =0000000000000a00
s8 =0000000050400988 s9 =0000000000000000 s10=0000000000000000 s11=0000000000000023
t3 =000000000000006c t4 =0000000000000068 t5 =0000000000000009 t6 =000000000000002a
priv=S mstatus=0000000a000401a0 cycles=113000467
 mideleg=0000000000000222 mie=0000000000000220 mip=00000000000000a0
raise_exception2: cause=13, tval=0x30002084, pc=0x50200b48
```

[riscv_fillpage is here](https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/arch/risc-v/src/common/riscv_exception.c#L97-L239)

Our NuttX UART Driver (0x50200b48) tries to read the UART Register (0x30002084) but fails (Load Page Fault, cause=13).

Why different from Ox64 Device? Is there a problem with our Ox64 TinyEMU Emulator?

Let's track down why mmu_ln_setentry caused the UART I/O to fail...

```text
riscv_fillpage: mmu_ln_setentry1: ptlevel=0x2, ptprev=0x50600000, paddr=0x5060c000, vaddr=0x80001000, MMU_UPGT_FLAGS=0
raise_exception2: cause=13, tval=0x30002084, pc=0x50200b48
```

# MMU Log for Ox64 With On-Demand Paging

_Why did mmu_ln_setentry cause UART I/O to fail?_

To understand the context, let's log all MMU Page Table Entries and updates to the MMU SATP Register...
- [Log the setting of all Page Table Entries in MMU](https://github.com/lupyuen2/wip-pinephone-nuttx/commit/8b24ef79d8d267d12af24e7d7b2dd8e8d82d8c98)
- [Log all updates to MMU SATP Register (Kernel Page Tables vs User Page Tables)](https://github.com/lupyuen2/wip-pinephone-nuttx/commit/6df7f09f200769bde075bd0ef2a942a64f2101cd)

We see this: https://gist.github.com/lupyuen/a9821b6867e98fb67c379f1fd842819a

```yaml
ABC
// Set Level 1 Page Table Entry for I/O Memory (0x0)
mmu_ln_setentry: ptlevel=0x1, lnvaddr=0x50406000, paddr=0, vaddr=0, mmuflags=0x9000000000000026

// Allocate Level 2 Page Tables for PLIC Memory (0xe000_0000)
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50404000, paddr=0xe0000000, vaddr=0xe0000000, mmuflags=0x9000000000000026
...
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50404000, paddr=0xefe00000, vaddr=0xefe00000, mmuflags=0x9000000000000026
mmu_ln_setentry: ptlevel=0x1, lnvaddr=0x50406000, paddr=0x50404000, vaddr=0xe0000000, mmuflags=0x20

// Allocate Level 3 Page Tables for Kernel Text (0x5020_0000)
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50405000, paddr=0x50402000, vaddr=0x50200000, mmuflags=0
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50402000, paddr=0x50200000, vaddr=0x50200000, mmuflags=0x2a
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50402000, paddr=0x50201000, vaddr=0x50201000, mmuflags=0x2a
...
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50402000, paddr=0x503ff000, vaddr=0x503ff000, mmuflags=0x2a

// Allocate Level 3 Page Tables for Kernel RAM (0x5040_0000)
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50405000, paddr=0x50403000, vaddr=0x50400000, mmuflags=0
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50403000, paddr=0x50400000, vaddr=0x50400000, mmuflags=0x26
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50403000, paddr=0x50401000, vaddr=0x50401000, mmuflags=0x26
...
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50405000, paddr=0x51800000, vaddr=0x51800000, mmuflags=0x26

// Set SATP Register to Kernel Page Tables at 0x5040_6000
mmu_write_satp: reg=0x8000000000050406
nx_start: Entry
uart_register: Registering /dev/console
work_start_lowpri: Starting low-priority kernel worker thread(s)
nxtask_activate: lpwork pid=1,TCB=0x504092f0
nxtask_activate: AppBringUp pid=2,TCB=0x50409900
nx_start_application: Starting init task: /system/bin/init

// Allocate Level 3 Page Tables for User Data (0x8010_0000)
mmu_ln_setentry: ptlevel=0x1, lnvaddr=0x50600000, paddr=0x50601000, vaddr=0x80100000, mmuflags=0
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50601000, paddr=0x50602000, vaddr=0x80100000, mmuflags=0
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50602000, paddr=0x50603000, vaddr=0x80100000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50602000, paddr=0x50604000, vaddr=0x80000000, mmuflags=0x1a
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50602000, paddr=0x50605000, vaddr=0x80101000, mmuflags=0x16

// Allocate Level 3 Page Tables for User Heap (0x8020_0000)
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50601000, paddr=0x50606000, vaddr=0x80200000, mmuflags=0
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50606000, paddr=0x50607000, vaddr=0x80200000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50606000, paddr=0x50608000, vaddr=0x80201000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50606000, paddr=0x50609000, vaddr=0x80202000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50606000, paddr=0x5060a000, vaddr=0x80203000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50606000, paddr=0x5060b000, vaddr=0x80204000, mmuflags=0x16

// Swap SATP Register to User Page Tables at 0x5060_0000
mmu_write_satp: reg=0x8000000000050600

// Page Fault for On-Demand Paging at User Text (0x8000_1000)
raise_exception2: cause=15, tval=0x80001000, pc=0x5020a124
pc =000000005020a124 ra =0000000050211602 sp =000000005040c510 gp =0000000000000000
tp =0000000000000000 t0 =000000000000002e t1 =0000000000000007 t2 =00000000000001ff
s0 =000000000000aa00 s1 =0000000000000000 a0 =0000000080000020 a1 =0000000050ade000
a2 =000000000000aa00 a3 =000000000000000d a4 =0000000080001000 a5 =0000000000000fe1
a6 =00000000000006f0 a7 =0000000080000020 s2 =000000000000ab7c s3 =0000000000000055
s4 =000000005040ab20 s5 =000000005040a900 s6 =0000000080000020 s7 =000000005040a918
s8 =000000005040c6d8 s9 =0000000000000000 s10=00000000000006f0 s11=0000000000000000
t3 =000000005040c50c t4 =000000005040c500 t5 =0000000000000009 t6 =000000000000002a
priv=S mstatus=0000000a000400a2 cycles=110089692
 mideleg=0000000000000222 mie=0000000000000220 mip=0000000000000080
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 000000000000000f, EPC: 000000005020a124, MTVAL: 0000000080001000
riscv_fillpage: ARCH_TEXT_SIZE=0x80000
riscv_fillpage: ARCH_TEXT_VEND=0x80080000
riscv_fillpage: vaddr=0x80001000
riscv_fillpage: FIX_ARCH_TEXT_VEND=0x80080000
riscv_fillpage: vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr <= ARCH_TEXT_VEND
riscv_fillpage: !paddr1

// Set Level 2 Page Table Entry for User Text (0x8000_1000)
riscv_fillpage: mmu_ln_setentry1: ptlevel=0x2, ptprev=0x50600000, paddr=0x5060c000, vaddr=0x80001000, MMU_UPGT_FLAGS=0
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50600000, paddr=0x5060c000, vaddr=0x80001000, mmuflags=0

// Page Fault for UART I/O Memory (0x3000_2084)
raise_exception2: cause=13, tval=0x30002084, pc=0x50200b66
pc =0000000050200b66 ra =0000000050200d22 sp =0000000050400830 gp =0000000000000000
tp =0000000000000000 t0 =000000000000002e t1 =000000000000006a t2 =00000000000001ff
s0 =0000000000000072 s1 =0000000200042120 a0 =0000000050400110 a1 =0000000000000072
a2 =000000000000000e a3 =0000000000000053 a4 =0000000030002000 a5 =000000000000000a
a6 =000000000000003f a7 =0000000000000000 s2 =0000000050400270 s3 =000000005021aed6
s4 =000000005021aec8 s5 =0000000000000030 s6 =000000005021ac9a s7 =0000000000000a00
s8 =0000000050400988 s9 =0000000000000000 s10=0000000000000000 s11=0000000000000023
t3 =000000000000006c t4 =0000000000000068 t5 =0000000000000009 t6 =000000000000002a
priv=S mstatus=0000000a000401a0 cycles=126009800
 mideleg=0000000000000222 mie=0000000000000220 mip=00000000000000a0
raise_exception2: cause=13, tval=0x30002084, pc=0x50200b66
pc =0000000050200b66 ra =0000000050200d22 sp =0000000050400710 gp =0000000000000000
tp =0000000000000000 t0 =000000000000002e t1 =000000000000006a t2 =00000000000001ff
s0 =000000000000005f s1 =0000000200042100 a0 =0000000050400110 a1 =000000000000005f
a2 =0000000000000007 a3 =0000000000000053 a4 =0000000030002000 a5 =000000000000000a
a6 =000000000000003f a7 =00000000504009d5 s2 =0000000050400270 s3 =000000005022bc1f
s4 =000000005022bc18 s5 =0000000000000030 s6 =000000005021b45a s7 =0000000000000a00
s8 =0000000050400868 s9 =0000000000000000 s10=0000000000000000 s11=0000000000000023
t3 =000000000000006c t4 =0000000000000068 t5 =0000000000000009 t6 =000000000000002a
priv=S mstatus=0000000a00040180 cycles=126011579
 mideleg=0000000000000222 mie=0000000000000220 mip=00000000000000a0
raise_exception2: cause=13, tval=0x30002084, pc=0x50200b66
pc =0000000050200b66 ra =00000000502
```

This looks interesting...

```yaml
mmu_ln_setentry: ptlevel=0x1, lnvaddr=0x50600000, paddr=0x50601000, vaddr=0x80100000, mmuflags=0
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50601000, paddr=0x50602000, vaddr=0x80100000, mmuflags=0
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50602000, paddr=0x50603000, vaddr=0x80100000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50602000, paddr=0x50604000, vaddr=0x80000000, mmuflags=0x1a
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50602000, paddr=0x50605000, vaddr=0x80101000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50601000, paddr=0x50606000, vaddr=0x80200000, mmuflags=0
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50606000, paddr=0x50607000, vaddr=0x80200000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50606000, paddr=0x50608000, vaddr=0x80201000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50606000, paddr=0x50609000, vaddr=0x80202000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50606000, paddr=0x5060a000, vaddr=0x80203000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50606000, paddr=0x5060b000, vaddr=0x80204000, mmuflags=0x16
...
// Swap SATP Register to User Page Tables at 0x5060_0000
mmu_write_satp: reg=0x8000000000050600
...
// Set Level 2 Page Table Entry for User Text (0x8000_1000)
riscv_fillpage: mmu_ln_setentry1: ptlevel=0x2, ptprev=0x50600000, paddr=0x5060c000, vaddr=0x80001000, MMU_UPGT_FLAGS=0
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50600000, paddr=0x5060c000, vaddr=0x80001000, mmuflags=0

// Page Fault for UART I/O Memory (0x3000_2084)
raise_exception2: cause=13, tval=0x30002084, pc=0x50200b66
  priv=S
```

At the top we see mmu_ln_setentry creating a...
- Level 2 Page Table
- Located at Physical Address 0x50601000
- Mapping to Virtual Address 0x80100000
- Adding to the Level 1 Page Table at 0x50600000
- mmuflags=0 means it's not a Leaf Page Table Entry

Then we see [riscv_fillpage](https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/arch/risc-v/src/common/riscv_exception.c#L97-L239) creating a...
- Level 3 Page Table
- Located at Physical Address 0x5060c000
- Mapping to Virtual Address 0x80001000
- Adding to the Level 1 Page Table at 0x50600000
- mmuflags=0 means it's not a Leaf Page Table Entry

TODO: Is this allowed?

TODO: How did [riscv_fillpage](https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/arch/risc-v/src/common/riscv_exception.c#L97-L239) get these addresses?

TODO: Is 0x5060c000 valid?

# UART I/O Fails on TinyEMU

_Why would UART I/O fail on TinyEMU but not Ox64? UART I/O is mapped as a Global Page Table Entry, it should be valid across all SATP Address Spaces (Supervisor Mode). Is there a bug in TinyEMU MMU?_

```yaml
// Page Fault for UART I/O Memory (0x3000_2084)
raise_exception2: cause=13, tval=0x30002084, pc=0x50200b66
  priv=S
```

Here's the MMU Address Translation from Virtual Address to Physical Address: [get_phys_addr](https://github.com/lupyuen/ox64-tinyemu/blob/mmu/riscv_cpu.c#L186-L304)

The MMU Bits defined are...

```c
#define PTE_V_MASK (1 << 0)  // Valid
#define PTE_U_MASK (1 << 4)  // User Mode
#define PTE_A_MASK (1 << 6)  // Accessed
#define PTE_D_MASK (1 << 7)  // Dirty
```

Compare the above with [Sv39 MMU](https://five-embeddev.com/riscv-priv-isa-manual/Priv-v1.12/supervisor.html#sec:sv39). The Global Bit is missing! (Bit 5)

So TinyMMU doesn't recognise [Global Page Table Entries](https://five-embeddev.com/riscv-priv-isa-manual/Priv-v1.12/supervisor.html#sec:sv32). Which is why UART I/O failed!

TODO: Support [Global Page Table Entries](https://five-embeddev.com/riscv-priv-isa-manual/Priv-v1.12/supervisor.html#sec:sv32) in TinyEMU

For Now: We do a workaround to allow UART I/O...
- [Workaround to allow UART I/O](https://github.com/lupyuen/ox64-tinyemu/commit/18e8de57464217841ea20330544e2992dd3eb43b)

Now Ox64 Emulator behaves like Ox64 Device yay! https://gist.github.com/lupyuen/58a994d511197163e117ea6243bfb346

```yaml
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 000000000000000f, EPC: 000000005020a126, MTVAL: 0000000080001000
riscv_fillpage: ARCH_TEXT_SIZE=0x80000
riscv_fillpage: ARCH_TEXT_VEND=0x80080000
riscv_fillpage: vaddr=0x80001000
riscv_fillpage: FIX_ARCH_TEXT_VEND=0x80080000
riscv_fillpage: vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr <= ARCH_TEXT_VEND
riscv_fillpage: ptlevel=0x2, ptprev=0x50600000, vaddr=0x80001000, mmu_ln_getentry=0x90000000000000e7
riscv_fillpage: !paddr1

riscv_fillpage: mmu_ln_setentry1: ptlevel=0x2, ptprev=0x50600000, paddr=0x5060c000, vaddr=0x80001000, MMU_UPGT_FLAGS=0
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50600000, paddr=0x5060c000, vaddr=0x80001000, mmuflags=0x0
riscv_fillpage: riscv_pgwipe1
riscv_fillpage: riscv_pgvaddr
riscv_fillpage: mm_pgalloc
riscv_fillpage: riscv_pgwipe2

riscv_fillpage: mmu_ln_setentry2: mmuflags=0x1e
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x5060c000, paddr=0x5060d000, vaddr=0x80001000, mmuflags=0x1e
riscv_fillpage: return

riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 000000000000000f, EPC: 000000005020a126, MTVAL: 0000000080001000
riscv_fillpage: ARCH_TEXT_SIZE=0x80000
riscv_fillpage: ARCH_TEXT_VEND=0x80080000
riscv_fillpage: vaddr=0x80001000
riscv_fillpage: FIX_ARCH_TEXT_VEND=0x80080000
riscv_fillpage: vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr <= ARCH_TEXT_VEND
riscv_fillpage: ptlevel=0x2, ptprev=0x50600000, vaddr=0x80001000, mmu_ln_getentry=0x14183001
riscv_fillpage: riscv_pgvaddr
riscv_fillpage: mm_pgalloc
riscv_fillpage: riscv_pgwipe2

riscv_fillpage: mmu_ln_setentry2: mmuflags=0x1e
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x5060c000, paddr=0x5060e000, vaddr=0x80001000, mmuflags=0x1e
riscv_fillpage: return
```

# On-Demand Paging stuck at 0x8000_1000

_Why is it looping forever at 0x8000\_1000?_

```yaml
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50601000, paddr=0x50606000, vaddr=0x80200000, mmuflags=0
```

From the above log, we see mmu_ln_setentry creating a...
- Level 2 Page Table
- Located at Physical Address 0x506_01000
- Mapping to Virtual Address 0x8010_0000
- Adding to the Level 1 Page Table at 0x5060_0000
- mmuflags=0 means it's not a Leaf Page Table Entry

```yaml
// Level 2 Page Table Entry not found for User Text (0x8000_1000)
riscv_fillpage: ptlevel=0x2, ptprev=0x50600000, vaddr=0x80001000, mmu_ln_getentry=0x90000000000000e7
riscv_fillpage: !paddr1

// Allocate the Level 3 Page Table for User Text (0x8000_1000)
riscv_fillpage: mmu_ln_setentry1: ptlevel=0x2, ptprev=0x50600000, paddr=0x5060c000, vaddr=0x80001000, MMU_UPGT_FLAGS=0
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50600000, paddr=0x5060c000, vaddr=0x80001000, mmuflags=0
```

Then we see [riscv_fillpage](https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/arch/risc-v/src/common/riscv_exception.c#L97-L239) creating a...
- Level 3 Page Table
- Located at Physical Address 0x5060_c000
- Mapping to Virtual Address 0x8000_1000
- Adding to the Level 1 Page Table at 0x5060_0000
- mmuflags=0 means it's not a Leaf Page Table Entry

```yaml
// Set Level 3 Page Table Entry for User Text (0x8000_1000)
riscv_fillpage: mmu_ln_setentry2: mmuflags=0x1e
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x5060c000, paddr=0x5060d000, vaddr=0x80001000, mmuflags=0x1e
```

Finally [riscv_fillpage](https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/arch/risc-v/src/common/riscv_exception.c#L97-L239) sets the...
- Level 3 Page Table Entry
- In the Level 2 Page Table at 0x5060_c000
- For Physical Address 0x5060_d000
- Mapping to Virtual Address 0x8000_1000
- mmuflags=0x1e means ???

TODO: Is this allowed?

TODO: How did [riscv_fillpage](https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/arch/risc-v/src/common/riscv_exception.c#L97-L239) get these addresses?

TODO: Is 0x5060c000 valid?

# MMU Log for Ox64 Without On-Demand Paging

Let's compare the above with Ox64 Without On-Demand Paging: https://gist.github.com/lupyuen/ef933ba72e983d7d49ef101e0816a714

```yaml
// Swap SATP Register to User Page Tables at 0x5040_6000
mmu_write_satp: reg=0x8000000000050406
nx_start: Entry
uart_register: Registering /dev/console
work_start_lowpri: Starting low-priority kernel worker thread(s)
nxtask_activate: lpwork pid=1,TCB=0x50409300
nxtask_activate: AppBringUp pid=2,TCB=0x50409910
nx_start_application: Starting init task: /system/bin/init

// Allocate Level 3 Page Table for User Data (0x8010_0000)
mmu_ln_setentry: ptlevel=0x1, lnvaddr=0x50600000, paddr=0x50601000, vaddr=0x80100000, mmuflags=0
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50601000, paddr=0x50602000, vaddr=0x80100000, mmuflags=0
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50602000, paddr=0x50603000, vaddr=0x80100000, mmuflags=0x16

// Set Level 3 Page Table Entry for User Text (0x8000_1000)
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50602000, paddr=0x50604000, vaddr=0x80000000, mmuflags=0x1a
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50602000, paddr=0x50605000, vaddr=0x80001000, mmuflags=0x1a
```

0x80001000 is actually a Level 3 Leaf Page Table Entry. With mmuflags=0x1a.

# Compare Ox64 With and Without On-Demand Paging

_Why is this different from Ox64 with On-Demand Paging?_

```yaml
// Ox64 Without On-Demand Paging:
// Level 3 Page Table at 0x5060_2000 is shared by 0x8000_0000 to 0x8010_1000

// Allocate the Level 3 Page Table for User Data (0x8010_0000)
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50601000, paddr=0x50602000, vaddr=0x80100000, mmuflags=0

// Set the Level 3 Page Table Entry for User Text (0x8000_1000)
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x50602000, paddr=0x50605000, vaddr=0x80001000, mmuflags=0x1a

// Ox64 With On-Demand Paging:
// Level 3 Page Table at 0x5060_2000 is shared by 0x8000_0000 to 0x8010_1000

// Allocate the Level 2 Page Table for User Data (0x8010_0000)
mmu_ln_setentry: ptlevel=0x1, lnvaddr=0x50600000, paddr=0x50601000, vaddr=0x80100000, mmuflags=0x0

// Level 3 Page Table Entry not found for User Text (0x8000_1000)
riscv_fillpage: ptlevel=0x2, ptprev=0x50600000, vaddr=0x80001000, mmu_ln_getentry=0x90000000000000e7
riscv_fillpage: !paddr1

// Allocate the Level 3 Page Table for User Text (0x8000_1000)
riscv_fillpage: mmu_ln_setentry1: ptlevel=0x2, ptprev=0x50600000, paddr=0x5060c000, vaddr=0x80001000, MMU_UPGT_FLAGS=0
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x50600000, paddr=0x5060c000, vaddr=0x80001000, mmuflags=0

// Set the Level 3 Page Table Entry for User Text (0x8000_1000)
riscv_fillpage: mmu_ln_setentry2: mmuflags=0x1e
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x5060c000, paddr=0x5060d000, vaddr=0x80001000, mmuflags=0x1e
```

Note that [64-bit RISC-V QEMU](https://gist.github.com/lupyuen/c7561fd8e5317868d8fd36313c3e7ce4) will use Level 3 Page Tables, unlike 32-bit RISC-V QEMU.

TODO: Why is the Level 3 Page Table different for With and Without On-Demand Paging?

Problem is that the Page Table Entry exists at Level 3. But we checked at Level 2 and failed...

```yaml
// Missing from Level 2
riscv_fillpage: ptlevel=0x2, ptprev=0x50600000, vaddr=0x80001000, mmu_ln_getentry=0x90000000000000e7

// But found at Level 3
riscv_fillpage: ptlevel2=0x3, ptprev=0x50600000, vaddr=0x80001000, mmu_ln_getentry2=0x14101421
```

TODO: Check for Page Table Entries at Levels 2 and 3

# Assume Level 3 Page Tables already allocated

Let's assume [Level 3 Page Tables are already allocated](https://github.com/lupyuen2/wip-pinephone-nuttx/commit/a97300be63765dfdc155cd13b06de55b3d615241).

Now we see this, still stuck at 0x8000_1000...

```yaml
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 000000000000000f, EPC: 000000005020a0f4, MTVAL: 0000000080001000
riscv_fillpage: vaddr=0x80001000
riscv_fillpage: ptlevel=0x2, ptprev=0x50600000, vaddr=0x80001000, mmu_ln_getentry=0x90000000000000e7
riscv_fillpage: ptlevel2=0x3, ptprev=0x50600000, vaddr=0x80001000, mmu_ln_getentry2=0x14101421
riscv_fillpage: riscv_pgvaddr
riscv_fillpage: mm_pgalloc
riscv_fillpage: riscv_pgwipe2

riscv_fillpage: mmu_ln_setentry2: mmuflags=0x1e
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0, paddr=0x5060c000, vaddr=0x80001000, mmuflags=0x1e
riscv_fillpage: return

riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 000000000000000f, EPC: 000000005020a0f4, MTVAL: 0000000080001000
```

TODO: Is this correct? ptlevel=0x3, lnvaddr=0, paddr=0x5060c000, vaddr=0x80001000, mmuflags=0x1e

# MMU Log for QEMU With On-Demand Paging

_How does Ox64 MMU compare with QEMU?_

We do the same logs for QEMU 32-bit RISC-V with On-Demand Paging...
- [NuttX Config for On-Demand Paging](https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/boards/risc-v/qemu-rv/rv-virt/configs/knsh32_paging/defconfig)
- [Log the internals of On-Demand Paging](https://github.com/lupyuen2/wip-pinephone-nuttx/commit/4cc8dde0c9143dd336c0a87bb2573d732f37661e)
- [Log the setting of all Page Table Entries in MMU](https://github.com/lupyuen2/wip-pinephone-nuttx/commit/73b730c367580de6b6b819043df107d1b2984a89)
- [Log all updates to MMU SATP](https://github.com/lupyuen2/wip-pinephone-nuttx/commit/e85babdad6506f927bbc5d2052c6f4f0029a764d)

Here's the MMU Log for QEMU 32-bit RISC-V with On-Demand Paging: https://gist.github.com/lupyuen/6075f31b575b54108e60b028083c16f7

[riscv_fillpage](https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/arch/risc-v/src/common/riscv_exception.c#L97-L239) allocates the User Text...

```yaml
// Init SATP Register to the Kernel Page Tables at 0x8040_1000
mmu_write_satp: reg=0x80080401
nx_start: Entry
uart_register: Registering /dev/console
uart_register: Registering /dev/ttyS0
work_start_lowpri: Starting low-priority kernel worker thread(s)
nxtask_activate: lpwork pid=1,TCB=0x80407ab0
nxtask_activate: AppBringUp pid=2,TCB=0x80407e70
nx_start_application: Starting init task: /system/bin/init

// Allocate Level 2 Page Tables for User Data Region 0xc010_0000
mmu_ln_setentry: ptlevel=0x1, lnvaddr=0x80800000, paddr=0x80801000, vaddr=0xc0100000, mmuflags=0x0
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80801000, paddr=0x80802000, vaddr=0xc0100000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80801000, paddr=0x80803000, vaddr=0xc0000000, mmuflags=0x1a
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80801000, paddr=0x80804000, vaddr=0xc0101000, mmuflags=0x16

// Allocate Level 2 Page Tables for User Heap Region 0xc080_0000
mmu_ln_setentry: ptlevel=0x1, lnvaddr=0x80800000, paddr=0x80805000, vaddr=0xc0800000, mmuflags=0x0
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80805000, paddr=0x80806000, vaddr=0xc0800000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80805000, paddr=0x80807000, vaddr=0xc0801000, mmuflags=0x16
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80805000, paddr=0x80808000, vaddr=0xc0802000, mmuflags=0x16

// Switch SATP Register to the User Page Tables at 0x8080_0000
mmu_write_satp: reg=0x80080800

// Page Fault for On-Demand Paging at 0xc000_1000 (User Text Region)
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 0000000f, EPC: 8001087e, MTVAL: c0001000
riscv_fillpage: ARCH_TEXT_SIZE=0x80000
riscv_fillpage: ARCH_TEXT_VEND=0xc0080000
riscv_fillpage: vaddr=0xc0001000
riscv_fillpage: vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr <= ARCH_TEXT_VEND
riscv_fillpage: riscv_pgvaddr
riscv_fillpage: mm_pgalloc
riscv_fillpage: riscv_pgwipe2
riscv_fillpage: mmu_ln_setentry2: mmuflags=0x1e

// Set the Level 2 Page Table Entry for 0xc000_1000 (User Text Region)
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80801000, paddr=0x80809000, vaddr=0xc0001000, mmuflags=0x1e
riscv_fillpage: return

// Page Fault for On-Demand Paging at 0xc000_2000 (User Text Region)
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 0000000f, EPC: 8001087e, MTVAL: c0002000
riscv_fillpage: ARCH_TEXT_SIZE=0x80000
riscv_fillpage: ARCH_TEXT_VEND=0xc0080000
riscv_fillpage: vaddr=0xc0002000
riscv_fillpage: vaddr >= CONFIG_ARCH_TEXT_VBASE && vaddr <= ARCH_TEXT_VEND
riscv_fillpage: riscv_pgvaddr
riscv_fillpage: mm_pgalloc
riscv_fillpage: riscv_pgwipe2
riscv_fillpage: mmu_ln_setentry2: mmuflags=0x1e

// Set the Level 2 Page Table Entry for 0xc000_2000 (User Text Region)
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80801000, paddr=0x8080a000, vaddr=0xc0002000, mmuflags=0x1e
riscv_fillpage: return
```

TODO: For Text: Why no mmu_ln_setentry1?

Then it allocates the User Heap...

```yaml
// Page Fault for On-Demand Paging at 0xc100_0ffc (User Heap Region)
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 0000000f, EPC: 8001123e, MTVAL: c1000ffc
riscv_fillpage: ARCH_TEXT_SIZE=0x80000
riscv_fillpage: ARCH_TEXT_VEND=0xc0080000
riscv_fillpage: vaddr=0xc1000000
riscv_fillpage: vaddr >= CONFIG_ARCH_HEAP_VBASE && vaddr <= ARCH_HEAP_VEND
riscv_fillpage: !paddr1
riscv_fillpage: mmu_ln_setentry1: ptlevel=0x1, ptprev=0x80800000, paddr=0x8081e000, vaddr=0xc1000000, MMU_UPGT_FLAGS=0

// Allocate the Level 2 Page Table for 0xc100_0000 (User Heap Region)
mmu_ln_setentry: ptlevel=0x1, lnvaddr=0x80800000, paddr=0x8081e000, vaddr=0xc1000000, mmuflags=0x0
riscv_fillpage: riscv_pgwipe1
riscv_fillpage: riscv_pgvaddr
riscv_fillpage: mm_pgalloc
riscv_fillpage: riscv_pgwipe2
riscv_fillpage: mmu_ln_setentry2

// Set the Level 2 Page Table Entry for 0xc100_0000 (User Heap Region)
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x8081e000, paddr=0x8081f000, vaddr=0xc1000000, mmuflags=0x16
riscv_fillpage: return
```

To allocate the User Heap, [riscv_fillpage](https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/arch/risc-v/src/common/riscv_exception.c#L97-L239) creates a...
- Level 2 Page Table
- Located at Physical Address 0x8081e000
- Mapping to Virtual Address 0xc1000000
- Adding to the Level 1 Page Table at 0x80800000

Then it creates a...
- Level 3 Page Table
- Located at Physical Address 0x8081f000
- Mapping to Virtual Address 0xc1000000
- Adding to the Level 2 Page Table at 0x8081e000

TODO: Compare the Ox64 and QEMU logs

# MMU Log for QEMU Without On-Demand Paging

We do the same logs for QEMU 32-bit RISC-V without On-Demand Paging...
- [NuttX Config Without On-Demand Paging](https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/boards/risc-v/qemu-rv/rv-virt/configs/knsh32/defconfig)
- [Enable Scheduler Logging](https://github.com/lupyuen2/wip-pinephone-nuttx/commit/5001cccd9a426485181cfb2acf035132fbd6a0eb)
- [Log the internals of On-Demand Paging](https://github.com/lupyuen2/wip-pinephone-nuttx/commit/4cc8dde0c9143dd336c0a87bb2573d732f37661e)
- [Log the setting of all Page Table Entries in MMU](https://github.com/lupyuen2/wip-pinephone-nuttx/commit/6422187397b6ca2a2df8507dca04c1d922555c4f)
- [Log all updates to MMU SATP](https://github.com/lupyuen2/wip-pinephone-nuttx/commit/e85babdad6506f927bbc5d2052c6f4f0029a764d)

Here's the MMU Log for [QEMU 32-bit RISC-V](https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/boards/risc-v/qemu-rv/rv-virt/configs/knsh32/defconfig) without On-Demand Paging: https://gist.github.com/lupyuen/1a473fac0f5aebc2c420cd0354573e5c

```yaml
// Switch SATP Register to the User Page Tables at 0x8040_4000
mmu_write_satp: reg=0x80080404
nx_start: Entry
uart_register: Registering /dev/console
uart_register: Registering /dev/ttyS0
work_start_lowpri: Starting low-priority kernel worker thread(s)
nxtask_activate: lpwork pid=1,TCB=0x80407a90
nx_start_application: Starting init task: /system/bin/init

// Allocate Level 2 Page Table for User Data Region 0xc010_0000
mmu_ln_setentry: ptlevel=0x1, lnvaddr=0x80800000, paddr=0x80801000, vaddr=0xc0100000, mmuflags=0x0

// Set the Level 2 Page Table Entry for User Data Region 0xc010_0000
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80801000, paddr=0x80802000, vaddr=0xc0100000, mmuflags=0x16

// Set the Level 2 Page Table Entry for User Text Region 0xc000_0000
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80801000, paddr=0x80803000, vaddr=0xc0000000, mmuflags=0x1a

// Set the Level 2 Page Table Entry for User Text Region 0xc000_1000
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80801000, paddr=0x80804000, vaddr=0xc0001000, mmuflags=0x1a
```

And here's the MMU Log for [QEMU 64-bit RISC-V](https://github.com/lupyuen2/wip-pinephone-nuttx/blob/on-demand-paging3/boards/risc-v/qemu-rv/rv-virt/configs/knsh64/defconfig) without On-Demand Paging: https://gist.github.com/lupyuen/c7561fd8e5317868d8fd36313c3e7ce4

```yaml
// Allocate Level 3 Page Table for User Data Region 0xc010_0000
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80401000, paddr=0x80402000, vaddr=0xc0100000, mmuflags=0x0

// Set the Level 3 Page Table Entry for User Data Region 0xc010_0000
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x80402000, paddr=0x80403000, vaddr=0xc0100000, mmuflags=0x16

// Set the Level 3 Page Table Entry for User Text Region 0xc000_0000
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x80402000, paddr=0x80404000, vaddr=0xc0000000, mmuflags=0x1a

// Set the Level 3 Page Table Entry for User Text Region 0xc000_1000
mmu_ln_setentry: ptlevel=0x3, lnvaddr=0x80402000, paddr=0x80405000, vaddr=0xc0001000, mmuflags=0x1a
```

_How does QEMU 32-bit compare with QEMU 64-bit?_

From the log above: 64-bit RISC-V will use Level 3 Page Tables. (32-bit RISC-V doesn't)

So Ox64 should also use Level 3 Page Tables for On-Demand Paging.

# Compare QEMU With and Without On-Demand Paging

We compare the MMU Logs for QEMU With and Without On-Demand Paging...

```yaml
// QEMU Without On-Demand Paging:

// Allocate Level 2 Page Table for User Data Region 0xc010_0000
mmu_ln_setentry: ptlevel=0x1, lnvaddr=0x80800000, paddr=0x80801000, vaddr=0xc0100000, mmuflags=0x0

// Set the Level 2 Page Table Entry for User Text Region 0xc000_1000
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80801000, paddr=0x80804000, vaddr=0xc0001000, mmuflags=0x1a

// QEMU With On-Demand Paging:

// Allocate Level 2 Page Table for User Data Region 0xc010_0000
mmu_ln_setentry: ptlevel=0x1, lnvaddr=0x80800000, paddr=0x80801000, vaddr=0xc0100000, mmuflags=0x0

// Page Fault for On-Demand Paging at 0xc000_1000 (User Text Region)
riscv_fillpage: EXCEPTION: Store/AMO page fault. MCAUSE: 0000000f, EPC: 8001087e, MTVAL: c0001000

// Set the Level 2 Page Table Entry for 0xc000_1000 (User Text Region)
mmu_ln_setentry: ptlevel=0x2, lnvaddr=0x80801000, paddr=0x80809000, vaddr=0xc0001000, mmuflags=0x1e
```

# TODO

TODO

<p align="center">
<img src="https://raw.githubusercontent.com/apache/nuttx/master/Documentation/_static/NuttX320.png" width="175">
</p>

![POSIX Badge](https://img.shields.io/badge/POSIX-Compliant-brightgreen?style=flat&label=POSIX)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue
)](https://nuttx.apache.org/docs/latest/introduction/licensing.html)
![Issues Tracking Badge](https://img.shields.io/badge/issue_track-github-blue?style=flat&label=Issue%20Tracking)
[![Contributors](https://img.shields.io/github/contributors/apache/nuttx
)](https://github.com/apache/nuttx/graphs/contributors)
[![GitHub Build Badge](https://github.com/apache/nuttx/workflows/Build/badge.svg)](https://github.com/apache/nuttx/actions/workflows/build.yml)
[![Documentation Badge](https://github.com/apache/nuttx/workflows/Build%20Documentation/badge.svg)](https://nuttx.apache.org/docs/latest/index.html)

Apache NuttX is a real-time operating system (RTOS) with an emphasis on
standards compliance and small footprint. Scalable from 8-bit to 64-bit
microcontroller environments, the primary governing standards in NuttX are POSIX
and ANSI standards. Additional standard APIs from Unix and other common RTOSs
(such as VxWorks) are adopted for functionality not available under these
standards, or for functionality that is not appropriate for deeply-embedded
environments (such as fork()).

For brevity, many parts of the documentation will refer to Apache NuttX as simply NuttX.

## Getting Started
First time on NuttX? Read the [Getting Started](https://nuttx.apache.org/docs/latest/quickstart/index.html) guide!
If you don't have a board available, NuttX has its own simulator that you can run on terminal.

## Documentation
You can find the current NuttX documentation on the [Documentation Page](https://nuttx.apache.org/docs/latest/).

Alternatively, you can build the documentation yourself by following the Documentation Build [Instructions](https://nuttx.apache.org/docs/latest/contributing/documentation.html).

The old NuttX documentation is still available in the [Apache wiki](https://cwiki.apache.org/NUTTX/NuttX).

## Supported Boards
NuttX supports a wide variety of platforms. See the full list on the [Supported Platforms](https://nuttx.apache.org/docs/latest/platforms/index.html) page.

## Contributing
If you wish to contribute to the NuttX project, read the [Contributing](https://nuttx.apache.org/docs/latest/contributing/index.html) guidelines for information on Git usage, coding standard, workflow and the NuttX principles.

## License
The code in this repository is under either the Apache 2 license, or a license compatible with the Apache 2 license. See the [License Page](https://nuttx.apache.org/docs/latest/introduction/licensing.html) for more information.
